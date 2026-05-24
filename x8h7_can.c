/**
 * X8H7 CAN driver
 */

#include "x8h7_can.h"

#include <linux/can/core.h>
#include <linux/can/dev.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/freezer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/wait.h>
#ifdef CONFIG_X8H7_CAN_DIAG_COUNTERS
#include <linux/debugfs.h>
#endif

#include "x8h7.h"

/* Phase 7.1.6.1 (Option c): gate TRACE-1a/2/3 in start_xmit (default 0 = off). TRACE-1 stays unconditional - bridge-decompose harness consumes it as T1 event source. */
#define DEBUG_X8H7_TX_TRACE 0

#define DRIVER_NAME "x8h7_can"

#ifdef CONFIG_X8H7_CAN_DIAG_COUNTERS
/* Phase 7.4.35 Patch 0085 - Tier 2 atomic counter rebuild for the motor-RX
 * dispatch path. ZERO kernel-log calls in this counter set (see
 * feedback_x8h7_p[r]_i[n]fo_cost.md: ~36 ms/call cost at LmP UART console
 * serialization is incompatible with the 500 Hz hot path).
 * Counters exposed read-only via debugfs at /sys/kernel/debug/x8h7_can/<name>;
 * bench polls via cat. */
static atomic_t motorrx_alloc_can_skb    = ATOMIC_INIT(0);  /* S1 - classic CAN alloc success */
static atomic_t motorrx_alloc_canfd_skb  = ATOMIC_INIT(0);  /* S5 - FD CAN alloc (defensive) */
static atomic_t motorrx_alloc_fail       = ATOMIC_INIT(0);  /* S2 - alloc NULL (cand_C direct) */
static atomic_t motorrx_netif_recv       = ATOMIC_INIT(0);  /* S3 - pre-dispatch */
static atomic_t motorrx_dispatch_skip    = ATOMIC_INIT(0);  /* S4 - tautology control */
static atomic_t motorrx_fd_promotion_branch_taken = ATOMIC_INIT(0); /* S6 - Phase 7.4.37 A3 pre-decision counter */
static struct dentry *x8h7_can_diag_dir  = NULL;
#endif

/* DEBUG HANDLING */
//#define DEBUG
#include "debug.h"
#ifdef DEBUG
  #define DBG_CAN_STATE(d, s) { \
    DBG_PRINT("device %s:CAN State: %d CAN controller %s\n", d, s, can_sts(s)); \
  }
#else
  #define DBG_CAN_STATE(i, s)
#endif

/**
 * Those parameters are valid for the STM32H7 CAN driver using
 * CAN classic. For CAN FD different values are needed.
 */
static const struct can_bittiming_const x8h7_can_bittiming_const = {
  .name      = DRIVER_NAME,   /* Name of the CAN controller hardware */
  .tseg1_min =   1,
  .tseg1_max = 256,
  .tseg2_min =   1,
  .tseg2_max = 128,
  .sjw_max   = 128,
  .brp_min   =   1,
  .brp_max   = 512,
  .brp_inc   =   1,
};

/* Data-phase bittiming constraints per ST RM0399 §56 FDCAN silicon limits
 * for the STM32H747. These are upper bounds the kernel uses for validation
 * of `ip link set canX ... dbitrate N` inputs. */
static const struct can_bittiming_const x8h7_can_data_bittiming_const = {
  .name      = DRIVER_NAME,
  .tseg1_min =  1,
  .tseg1_max = 32,    /* FDCAN data-phase DTSEG1 max per ST RM0399 §56 */
  .tseg2_min =  1,
  .tseg2_max = 16,    /* FDCAN data-phase DTSEG2 max per ST RM0399 §56 */
  .sjw_max   = 16,
  .brp_min   =  1,
  .brp_max   = 32,    /* FDCAN data-phase DBRP max per ST RM0399 §56 */
  .brp_inc   =  1,
};

static void x8h7_can_tx_work_handler(struct work_struct *ws);
static void x8h7_can_error_skb(struct net_device *net, int can_id, int data1);

/**
 */
static void x8h7_can_frame_to_tx_obj(struct canfd_frame const *cfd,
                                     union x8h7_can_frame_message *x8h7_can_msg,
                                     bool is_fd)
{
  memset(x8h7_can_msg->buf, 0, sizeof(x8h7_can_msg->buf));

  if (cfd->can_id & CAN_EFF_FLAG)
    x8h7_can_msg->field.id  = CAN_EFF_FLAG | (cfd->can_id & CAN_EFF_MASK);
  else
    x8h7_can_msg->field.id  =                (cfd->can_id & CAN_SFF_MASK);

  WARN_ON(cfd->len > X8H7_CAN_FRAME_MAX_DATA_LEN);
  x8h7_can_msg->field.len = (cfd->len <= X8H7_CAN_FRAME_MAX_DATA_LEN)
                              ? cfd->len : X8H7_CAN_FRAME_MAX_DATA_LEN;

  if (is_fd) {
    x8h7_can_msg->field.flags = X8H7_CAN_FLAG_FD;
    if (cfd->flags & CANFD_BRS)
      x8h7_can_msg->field.flags |= X8H7_CAN_FLAG_BRS;
    if (cfd->flags & CANFD_ESI)
      x8h7_can_msg->field.flags |= X8H7_CAN_FLAG_ESI;
  }

  memcpy(x8h7_can_msg->field.data, cfd->data, x8h7_can_msg->field.len);
}

/**
 */
static char* can_sts(enum can_state sts)
{
  switch(sts){
  case CAN_STATE_ERROR_ACTIVE : return "is error active";
  case CAN_STATE_ERROR_WARNING: return "is error active, warning level is reached";
  case CAN_STATE_ERROR_PASSIVE: return "is error passive";
  case CAN_STATE_BUS_OFF      : return "went into Bus Off";
  case CAN_STATE_STOPPED      : return "is in stopped mode";
  case CAN_STATE_SLEEPING     : return "is in Sleep mode";
  default                     : return "is unknown state";
  }
}

/**
 */
static void x8h7_can_status(struct x8h7_can_priv *priv, u8 intf, u8 eflag)
{
  struct net_device  *net = priv->net;
  int                 can_id = 0;
  int                 data1 = 0;

  //DBG_PRINT("\n");

  if (intf & X8H7_CAN_STS_INT_ERR)
  {
    /* Handle overflow counters */
    if (eflag & X8H7_CAN_STS_FLG_RX_OVR)
    {
      net->stats.rx_over_errors++;
      net->stats.rx_errors++;
      can_id |= CAN_ERR_CRTL;
      data1 |= CAN_ERR_CRTL_RX_OVERFLOW;
      x8h7_can_error_skb(net, can_id, data1);
    }
    if (eflag & X8H7_CAN_STS_FLG_TX_OVR)
    {
      net->stats.tx_fifo_errors++;
      net->stats.tx_errors++;
      can_id |= CAN_ERR_CRTL;
      data1 |= CAN_ERR_CRTL_TX_OVERFLOW;
      x8h7_can_error_skb(net, can_id, data1);
    }
  }

  if (intf & X8H7_CAN_STS_INT_TX_COMPLETE) {
    /* Phase 7.4.42 Patch 0089 — TX batching demux.
     * H7's HAL_FDCAN_TxBufferCompleteCallback emits one x8_msg per IRQ but
     * BufferIndexes (carried as a bitmask in the second byte of x8_msg by
     * stm32h7-fw/src/can_handler.c:542) may indicate multiple TX buffers
     * completed simultaneously. The simplest correct approach: free ALL
     * outstanding echo skbs on any TX_COMPLETE. By the time any frame in
     * a batch completes, all batched frames have been TX'd by H7 (they
     * share the same SPI exchange and H7 dispatches them sequentially to
     * FDCAN TX buffers). Subsequent TX_COMPLETE IRQs for the same batch
     * find batch_outstanding==0 and become no-ops. */
    int batch_count;
    unsigned long flags;
    int i;

    spin_lock_irqsave(&priv->ring_lock, flags);
    batch_count = atomic_read(&priv->batch_outstanding);
    atomic_set(&priv->batch_outstanding, 0);
    spin_unlock_irqrestore(&priv->ring_lock, flags);

    DBG_PRINT("TX COMPLETE (batch=%d)", batch_count);

    /* Free echo skbs for all idx in 0..batch_count-1. can_get_echo_skb
     * tolerates being called on already-empty slots (returns NULL); the
     * count is the upper bound on what could be queued. */
    for (i = 0; i < batch_count; i++) {
      can_get_echo_skb(net, i, NULL);
    }

    net->stats.tx_packets += (batch_count > 0) ? batch_count : 1;
    net->stats.tx_bytes += priv->tx_len;
    priv->tx_len = 0;

    netif_wake_queue(net);
  }

  if (intf & X8H7_CAN_STS_INT_TX_ABORT_COMPLETE)
  {
    DBG_PRINT("TX ABORT COMPLETE");
  }

  if (intf & X8H7_CAN_STS_INT_TX_FIFO_EMPTY)
  {
    DBG_PRINT("TX FIFO EMPTY");
  }
}

/**
 */
static void x8h7_can_hook(void *arg, x8h7_pkt_t *pkt)
{
  struct x8h7_can_priv  *priv = (struct x8h7_can_priv*)arg;

#if DEBUG_X8H7_TX_TRACE
  pr_info("[x8h7_can_TRACE-7] hook: ENTRY periph=%u opcode=0x%02X size=%u\n",
          priv->periph, pkt->opcode, pkt->size);
#endif

  switch(pkt->opcode) {
  case X8H7_CAN_OC_RECV:
    if (pkt->size < X8H7_CAN_HEADER_SIZE) {
      DBG_ERROR("received packed is too short (%d)\n", pkt->size);
      return;
    } else {
      struct sk_buff     *skb;
      struct canfd_frame *cfd;
      bool is_fd;
      bool needs_fd_alloc;
      union x8h7_can_frame_message x8h7_can_msg;

      /* Copy the full header (6 bytes) first to read the flags byte. */
      memcpy(x8h7_can_msg.buf, pkt->data, X8H7_CAN_HEADER_SIZE);
      is_fd = (x8h7_can_msg.field.flags & X8H7_CAN_FLAG_FD) != 0;

      /* Phase 7.4.37 A1: local_bh_disable around alloc + dispatch for
       * __this_cpu_* preempt-safety in irq/37-x8h7 kthread context (see
       * RESEARCH §5.1 + §3.4). A3: unified FD-promotion — promote on FD
       * flag OR wire len > CAN_MAX_DLEN per candidate (e). */
      local_bh_disable();
      needs_fd_alloc = is_fd || (x8h7_can_msg.field.len > CAN_MAX_DLEN);
#ifdef CONFIG_X8H7_CAN_DIAG_COUNTERS
      atomic_inc(&motorrx_fd_promotion_branch_taken);           /* S6 */
#endif

      if (needs_fd_alloc && (priv->can.ctrlmode & CAN_CTRLMODE_FD)) {
        skb = alloc_canfd_skb(priv->net, &cfd);
#ifdef CONFIG_X8H7_CAN_DIAG_COUNTERS
        if (skb) atomic_inc(&motorrx_alloc_canfd_skb);          /* S5 */
#endif
      } else {
        skb = alloc_can_skb(priv->net, (struct can_frame **)&cfd);
#ifdef CONFIG_X8H7_CAN_DIAG_COUNTERS
        if (skb) atomic_inc(&motorrx_alloc_can_skb);            /* S1 */
#endif
      }
      if (!skb) {
#ifdef CONFIG_X8H7_CAN_DIAG_COUNTERS
        atomic_inc(&motorrx_alloc_fail);                         /* S2 */
#endif
        netdev_warn(priv->net, "cannot allocate RX skb (FD=%d)\n", is_fd);
        priv->net->stats.rx_dropped++;
        local_bh_enable();                                       /* Phase 7.4.37 A1 close (error path) */
        return;
      }

      /* canfd_frame and can_frame share can_id + len at the same offsets,
       * so this assignment works for both classic and FD skbs. */
      cfd->can_id = x8h7_can_msg.field.id;
      cfd->len    = x8h7_can_msg.field.len;
      if (cfd->len > X8H7_CAN_FRAME_MAX_DATA_LEN)
        cfd->len = X8H7_CAN_FRAME_MAX_DATA_LEN;

      if (is_fd) {
        cfd->flags = 0;
        if (x8h7_can_msg.field.flags & X8H7_CAN_FLAG_BRS)
          cfd->flags |= CANFD_BRS;
        if (x8h7_can_msg.field.flags & X8H7_CAN_FLAG_ESI)
          cfd->flags |= CANFD_ESI;
      }

      /* Copy data bytes (payload follows the 6-byte header on the wire). */
      memcpy(cfd->data, pkt->data + X8H7_CAN_HEADER_SIZE, cfd->len);

      DBG_PRINT("received data %X len=%d FD=%d, copying to %p\n",
                cfd->can_id, cfd->len, is_fd, cfd->data);

      priv->net->stats.rx_packets++;
      priv->net->stats.rx_bytes += cfd->len;
#ifdef CONFIG_X8H7_CAN_DIAG_COUNTERS
      atomic_inc(&motorrx_netif_recv);                          /* S3 - pre-dispatch */
#endif
      /* Phase 7.4.32 Patch 0084 — Plan 01 verdict CONFIRMED_NET_RX_SUCCESS (13/13)
       * routes RX dispatch via netif_receive_skb (synchronous; bypasses backlog
       * softirq scheduling). CAVEAT: documented softirq-only; pre-patch dmesg
       * validated by Plan 02 Task 3. See patches/x8h7/0084-README.md §3. */
      netif_receive_skb(skb);
#ifdef CONFIG_X8H7_CAN_DIAG_COUNTERS
      atomic_inc(&motorrx_dispatch_skip);                       /* S4 - tautology control */
#endif
      local_bh_enable();                                        /* Phase 7.4.37 A1 close */
    }
    break;
  case X8H7_CAN_OC_STS:
    DBG_PRINT("received status %02X %02X\n", pkt->data[0], pkt->data[1]);
    x8h7_can_status(priv, pkt->data[0], pkt->data[1]);
    break;
  }
}

#ifdef CONFIG_X8H7_CAN_DIAG_COUNTERS
/* Phase 7.4.35 Patch 0085 - debugfs read handlers for the 5 atomic counters.
 * Each handler reads its respective atomic_t via atomic_read() and returns it
 * as a u64 (debugfs canonical format). All files are read-only mode 0444. */
static int motorrx_alloc_can_skb_get(void *data, u64 *val) {
    *val = (u64)atomic_read(&motorrx_alloc_can_skb);
    return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(motorrx_alloc_can_skb_fops,
                         motorrx_alloc_can_skb_get, NULL, "%llu\n");

static int motorrx_alloc_canfd_skb_get(void *data, u64 *val) {
    *val = (u64)atomic_read(&motorrx_alloc_canfd_skb);
    return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(motorrx_alloc_canfd_skb_fops,
                         motorrx_alloc_canfd_skb_get, NULL, "%llu\n");

static int motorrx_alloc_fail_get(void *data, u64 *val) {
    *val = (u64)atomic_read(&motorrx_alloc_fail);
    return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(motorrx_alloc_fail_fops,
                         motorrx_alloc_fail_get, NULL, "%llu\n");

static int motorrx_netif_recv_get(void *data, u64 *val) {
    *val = (u64)atomic_read(&motorrx_netif_recv);
    return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(motorrx_netif_recv_fops,
                         motorrx_netif_recv_get, NULL, "%llu\n");

static int motorrx_dispatch_skip_get(void *data, u64 *val) {
    *val = (u64)atomic_read(&motorrx_dispatch_skip);
    return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(motorrx_dispatch_skip_fops,
                         motorrx_dispatch_skip_get, NULL, "%llu\n");

/* Phase 7.4.37 A2: 6th binding for motorrx_fd_promotion_branch_taken (S6) */
static int motorrx_fd_promotion_branch_taken_get(void *data, u64 *val) {
    *val = (u64)atomic_read(&motorrx_fd_promotion_branch_taken);
    return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(motorrx_fd_promotion_branch_taken_fops,
                         motorrx_fd_promotion_branch_taken_get, NULL, "%llu\n");

static int x8h7_can_diag_init(void) {
    x8h7_can_diag_dir = debugfs_create_dir("x8h7_can", NULL);
    if (IS_ERR(x8h7_can_diag_dir)) return PTR_ERR(x8h7_can_diag_dir);
    debugfs_create_file_unsafe("motorrx_alloc_can_skb",    0444,
                               x8h7_can_diag_dir, NULL, &motorrx_alloc_can_skb_fops);
    debugfs_create_file_unsafe("motorrx_alloc_canfd_skb",  0444,
                               x8h7_can_diag_dir, NULL, &motorrx_alloc_canfd_skb_fops);
    debugfs_create_file_unsafe("motorrx_alloc_fail",       0444,
                               x8h7_can_diag_dir, NULL, &motorrx_alloc_fail_fops);
    debugfs_create_file_unsafe("motorrx_netif_recv",       0444,
                               x8h7_can_diag_dir, NULL, &motorrx_netif_recv_fops);
    debugfs_create_file_unsafe("motorrx_dispatch_skip",    0444,
                               x8h7_can_diag_dir, NULL, &motorrx_dispatch_skip_fops);
    debugfs_create_file_unsafe("motorrx_fd_promotion_branch_taken", 0444,
                               x8h7_can_diag_dir, NULL, &motorrx_fd_promotion_branch_taken_fops);
    return 0;
}

static void x8h7_can_diag_exit(void) {
    debugfs_remove_recursive(x8h7_can_diag_dir);
    x8h7_can_diag_dir = NULL;
}
#endif

/*
 * device (auto-)restart mechanism runs in a timer context =>
 * MUST handle restart with asynchronous spi transfers (if any)
 */
static int x8h7_can_restart(struct net_device *net)
{
  struct x8h7_can_priv *priv = netdev_priv(net);
  int err = 0;

  DBG_PRINT("\n");

  /* @TODO: notify external hw? */
  DBG_PRINT("@TODO: notify external hw?\n");

  /* finally MUST update can state */
	priv->can.state = CAN_STATE_ERROR_ACTIVE;

  /* netdev queue can be awaken now */
	netif_wake_queue(net);

  return err;
}

/**
 */
static int x8h7_can_hw_setup(struct x8h7_can_priv *priv)
{
  struct can_bittiming *bt  = &priv->can.bittiming;
  struct can_bittiming *dbt = &priv->can.data_bittiming;
  union x8h7_can_init_message x8h7_msg;

  DBG_PRINT("bitrate: %d, sample_point: %d, tq: %d, prop_seg: %d, phase_seg1: %d, phase_seg2: %d, sjw: %d, brp: %d, freq: %d ctrlmode: %08X\n",
            bt->bitrate,
            bt->sample_point,
            bt->tq,
            bt->prop_seg,
            bt->phase_seg1,
            bt->phase_seg2,
            bt->sjw,
            bt->brp,
            priv->can.clock.freq,
            priv->can.ctrlmode);

  memset(&x8h7_msg, 0, sizeof(x8h7_msg));

  x8h7_msg.field.baud_rate_prescaler = bt->brp;
  x8h7_msg.field.time_segment_1      = bt->prop_seg + bt->phase_seg1;
  x8h7_msg.field.time_segment_2      = bt->phase_seg2;
  x8h7_msg.field.sync_jump_width     = bt->sjw;

  /* Pack data-phase bittiming (zero if FD mode not active -- H7 firmware
   * ignores these fields when FrameFormat = FDCAN_FRAME_CLASSIC). */
  if (priv->can.ctrlmode & CAN_CTRLMODE_FD) {
    x8h7_msg.field.data_baud_rate_prescaler = dbt->brp;
    x8h7_msg.field.data_time_segment_1      = dbt->prop_seg + dbt->phase_seg1;
    x8h7_msg.field.data_time_segment_2      = dbt->phase_seg2;
    x8h7_msg.field.data_sync_jump_width     = dbt->sjw;

    DBG_PRINT("dbitrate data: brp=%d tseg1=%d tseg2=%d sjw=%d\n",
              dbt->brp,
              dbt->prop_seg + dbt->phase_seg1,
              dbt->phase_seg2,
              dbt->sjw);
  }

  DBG_PRINT("baud_rate_prescaler: %d, time_segment_1: %d, time_segment_2: %d, sync_jump_width: %d\n",
            x8h7_msg.field.baud_rate_prescaler,
            x8h7_msg.field.time_segment_1,
            x8h7_msg.field.time_segment_2,
            x8h7_msg.field.sync_jump_width);

  x8h7_pkt_send_sync(priv->periph, X8H7_CAN_OC_INIT, sizeof(x8h7_msg.buf), x8h7_msg.buf);

  return 0;
}

/**
 * Data-phase bittiming callback. The kernel calls this after userspace sets
 * `dbitrate N` via ip-link. The actual values are packed and shipped together
 * with the arbitration-phase params in x8h7_can_hw_setup() on the next
 * link-up -- this callback satisfies the kernel's per-callback contract.
 */
static int x8h7_can_hw_do_set_data_bittiming(struct net_device *net)
{
  /* No-op: data-phase bittiming is applied in x8h7_can_hw_setup() as
   * part of the X8H7_CAN_OC_INIT message. */
  return 0;
}

/**
 */
static int x8h7_can_hw_stop(struct x8h7_can_priv *priv)
{
  x8h7_pkt_send_sync(priv->periph, X8H7_CAN_OC_DEINIT, 0, NULL);

  return 0;
}

/**
 */
static int x8h7_can_set_normal_mode(struct x8h7_can_priv *priv)
{
  DBG_PRINT("\n");

  /* Enable interrupts */
  x8h7_hook_set(priv->periph, x8h7_can_hook, priv);

  /* Transition out of STOPPED so the kernel CAN core stops dropping TX skbs */
  priv->can.state = CAN_STATE_ERROR_ACTIVE;

  return 0;
}

/**
 */
static void x8h7_can_error_skb(struct net_device *net, int can_id, int data1)
{
  struct sk_buff *skb;
  struct can_frame *frame;

  DBG_PRINT("\n");
  skb = alloc_can_err_skb(net, &frame);
  if (skb) {
    frame->can_id |= can_id;
    frame->data[1] = data1;
    netif_rx(skb);
  } else {
    netdev_err(net, "cannot allocate error skb\n");
  }
}

/**
 */
/* Phase 7.4.42 Patch 0089 — hrtimer callback for partial-batch flush.
 * Forward declared so x8h7_can_open below can register it. The callback
 * runs in softirq context; safe to call queue_work (which is irq-safe).
 * Definition placed AFTER x8h7_can_tx_work_handler so we can reference
 * priv->work and the workqueue directly. */
static enum hrtimer_restart x8h7_can_batch_timer_cb(struct hrtimer *t);

static int x8h7_can_open(struct net_device *net)
{
  struct x8h7_can_priv *priv = netdev_priv(net);
  int                   ret;

  DBG_PRINT("\n");

  ret = open_candev(net);
  if (ret) {
    DBG_ERROR("unable to set initial baudrate!\n");
    return ret;
  }

  priv->tx_len  = 0;

  /* Phase 7.4.42 Patch 0089 — initialize TX batching state.
   * spinlock used (not mutex) because start_xmit may run in softirq context
   * and the lock-holders are short (only update batch_outstanding + tx_ring;
   * NEVER hold while calling x8h7_pkt_send_defer/now which take the spidev
   * mutex and may sleep). */
  spin_lock_init(&priv->ring_lock);
  atomic_set(&priv->batch_outstanding, 0);
  hrtimer_init(&priv->batch_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
  priv->batch_timer.function = x8h7_can_batch_timer_cb;

  priv->wq = alloc_workqueue("x8h7_can_wq", WQ_FREEZABLE | WQ_MEM_RECLAIM | WQ_HIGHPRI, 0);
  if (!priv->wq) {
    ret = -ENOMEM;
    goto out_clean;
  }
  INIT_WORK(&priv->work, x8h7_can_tx_work_handler);

  mutex_init(&priv->lock);

  ret = x8h7_can_hw_stop(priv);
  if (ret) {
    goto out_free_wq;
  }
  ret = x8h7_can_hw_setup(priv);
  if (ret) {
    goto out_free_wq;
  }
  ret = x8h7_can_set_normal_mode(priv);
  if (ret) {
    goto out_free_wq;
  }

  netif_start_queue(net);

  return 0;

out_free_wq:
  destroy_workqueue(priv->wq);
out_clean:
  x8h7_hook_set(priv->periph, NULL, NULL);
  close_candev(net);
  return ret;
}

/*
 * Called by netdev to close the corresponding CAN interface.
 */
static int x8h7_can_stop(struct net_device *net)
{
  struct x8h7_can_priv *priv = netdev_priv(net);

  DBG_PRINT("\n");

  /* Notify upper level */
  netif_stop_queue(net);
  close_candev(net);

  /* Notify ext. hw to stop can peripheral */
  x8h7_can_hw_stop(priv);

  /* Phase 7.4.42 Patch 0089 — cancel batch flush timer BEFORE destroying
   * workqueue. hrtimer_cancel() is safe to call even if timer never fired. */
  hrtimer_cancel(&priv->batch_timer);

  /* Free priv. resources */
  mutex_lock(&priv->lock);
  x8h7_hook_set(priv->periph, NULL, NULL);
  destroy_workqueue(priv->wq);
  priv->wq = NULL;

  priv->can.state = CAN_STATE_STOPPED;
  mutex_unlock(&priv->lock);

  return 0;
}

/**
 */
static netdev_tx_t x8h7_can_start_xmit(struct sk_buff *skb,
                                       struct net_device *net)
{
  struct x8h7_can_priv        *priv = netdev_priv(net);
  const struct device         *dev = priv->dev;
  struct canfd_frame          *cfd_dbg;
  struct canfd_frame          *cfd;
  bool                         is_fd;
  int                          slot;
  unsigned long                flags;
  bool                         queue_was_full;

  DBG_PRINT("\n");

  cfd_dbg = (struct canfd_frame *)skb->data;
#if DEBUG_X8H7_TX_TRACE
  pr_info("[x8h7_can_TRACE-1] start_xmit: dev=%s skb_len=%u id=0x%08X len=%u state=%d qstop=%d outstanding=%d\n",
          net->name, skb->len, cfd_dbg->can_id, cfd_dbg->len,
          priv->can.state, netif_queue_stopped(net),
          atomic_read(&priv->batch_outstanding));
#endif

  if (can_dropped_invalid_skb(net, skb)) {
#if DEBUG_X8H7_TX_TRACE
    pr_info("[x8h7_can_TRACE-1a] start_xmit: skb DROPPED by can_dropped_invalid_skb\n");
#endif
    return NETDEV_TX_OK;
  }

  is_fd = can_is_canfd_skb(skb);
  cfd   = (struct canfd_frame *)skb->data;

  /* Phase 7.4.42 Patch 0089 — TX batching ring populate.
   * Acquire ring_lock; reserve a slot in tx_ring; populate the frame; queue
   * the echo skb at the reserved idx; release lock. We do NOT call
   * x8h7_pkt_send_defer here — that happens in the work handler with all
   * batched frames at once. */
  spin_lock_irqsave(&priv->ring_lock, flags);

  slot = atomic_read(&priv->batch_outstanding);
  if (slot >= X8H7_CAN_BATCH_MAX) {
    /* Ring full — kernel should never reach here because we stop the queue
     * on the previous frame when we hit MAX, but defensive guard remains. */
    spin_unlock_irqrestore(&priv->ring_lock, flags);
    netif_stop_queue(net);
#if DEBUG_X8H7_TX_TRACE
    pr_info("[x8h7_can_TRACE-1b] start_xmit: ring FULL (slot=%d) — returning NETDEV_TX_BUSY\n", slot);
#endif
    return NETDEV_TX_BUSY;
  }

  x8h7_can_frame_to_tx_obj(cfd, &priv->tx_ring[slot], is_fd);
  atomic_inc(&priv->batch_outstanding);
  queue_was_full = (slot + 1) >= X8H7_CAN_BATCH_MAX;

  spin_unlock_irqrestore(&priv->ring_lock, flags);

#if DEBUG_X8H7_TX_TRACE
  pr_info("[x8h7_can_TRACE-2] start_xmit: pre can_put_echo_skb (idx=%d, is_fd=%d, len=%u)\n",
          slot, is_fd, cfd->len);
#endif
  /* Echo skb uses the SAME slot index — alloc_candev was bumped to
   * X8H7_CAN_BATCH_MAX so idx 0..MAX-1 are valid. The IRQ-side TX_COMPLETE
   * handler frees all outstanding echoes (simpler than per-H7-buf-idx
   * demux; tradeoff is wasted accounting but no leak). */
  can_put_echo_skb(skb, net, slot, 0);

  if (queue_was_full) {
    /* Stop the netif queue BEFORE scheduling work — prevents another
     * start_xmit from racing with the work handler's flush. Wake happens
     * in the TX_COMPLETE handler after batch drains. */
    netif_stop_queue(net);
#if DEBUG_X8H7_TX_TRACE
    pr_info("[x8h7_can_TRACE-3] start_xmit: batch FULL -> stop_queue + queue_work (wq=%p)\n",
            priv->wq);
#endif
    /* Cancel any pending hrtimer; the immediate queue_work supersedes it. */
    hrtimer_try_to_cancel(&priv->batch_timer);
    queue_work(priv->wq, &priv->work);
  } else if (!hrtimer_active(&priv->batch_timer)) {
    /* First frame of a new batch — arm the deadline timer to ensure
     * partial batches flush even at low TX rate. */
    hrtimer_start(&priv->batch_timer,
                  ns_to_ktime(X8H7_CAN_BATCH_DEADLINE_NS),
                  HRTIMER_MODE_REL);
  }

  return NETDEV_TX_OK;
}

/**
 * Phase 7.4.29 Plan 02 Fix-B (A3 mutex split).
 *
 * Decouple the TX work handler's enqueue phase from the SPI-exchange phase so
 * the singleton `spidev->lock` (also taken by `x8h7_threaded_isr` at
 * `x8h7_drv.c:451`) is held for the SHORT enqueue window (~microseconds), then
 * released; the subsequent `x8h7_pkt_send_now()` re-takes it for the ~40 ms
 * SPI exchange.
 *
 * Why this matters: previously, `x8h7_pkt_send_sync` held the mutex for the
 * ENTIRE 40 ms TX exchange. A concurrent threaded ISR (carrying a motor 0xA0
 * response) had to wait the full ~40 ms before its own RX SPI exchange could
 * begin, serializing TX+RX round-trip at ~80 ms ≈ 12.5 Hz (matches Phase
 * 7.4.29 Plan 01 observed outer-loop rate 12.764 Hz). After this fix, the ISR
 * can interleave its mutex acquisition AT the enqueue boundary instead of
 * after the full TX exchange — narrowing the contention window.
 *
 * Scope: CAN TX work-handler hot path ONLY. Other peripherals (UART/ADC/GPIO/
 * PWM/RTC/UI/H7) still use `x8h7_pkt_send_sync` unchanged; the shared mutex
 * ABI for non-CAN clients is preserved.
 *
 * Existing primitive used: `x8h7_pkt_send_defer` (x8h7_drv.c:234-247) and
 * `x8h7_pkt_send_now` (x8h7_drv.c:251-264) — both already `EXPORT_SYMBOL_GPL`
 * exported in the production kernel module; no new exports required.
 *
 * Known limitation (carry-forward): the explicit `x8h7_pkt_send_now()` still
 * acquires the full mutex for the 40 ms SPI exchange. The TX vs RX SPI
 * exchanges remain structurally serialized; full collapse to one full-duplex
 * exchange would require relying on the next IRQ-driven exchange to flush
 * pending TX (deadlock risk if no IRQ arrives — out of scope for Plan 02).
 */
static void x8h7_can_tx_work_handler(struct work_struct *ws)
{
  struct x8h7_can_priv *priv = container_of(ws, struct x8h7_can_priv, work);
  int spi_ret = 0;
  int count;
  int i;
  unsigned long flags;
  /* Snapshot the ring under the lock; release before calling x8h7_pkt_send_*
   * which take the spidev mutex and may sleep (cannot hold spinlock while
   * sleeping in atomic context).
   *
   * Phase 7.4.42 Patch 0089: extends the Phase 7.4.29 Fix-B mutex split to
   * batch N frames per SPI exchange. The defer/now primitives from x8h7_drv
   * accumulate sub-packets in spidev->x8h7_txb; calling defer N times before
   * send_now coalesces N CAN frames into one ~40ms SPI exchange. Theoretical
   * gain at N=6: ~6x reduction in SPI exchange count vs the single-frame
   * Fix-B baseline. */
  union x8h7_can_frame_message snap_ring[X8H7_CAN_BATCH_MAX];

  spin_lock_irqsave(&priv->ring_lock, flags);
  count = atomic_read(&priv->batch_outstanding);
  if (count > X8H7_CAN_BATCH_MAX)
    count = X8H7_CAN_BATCH_MAX;  /* defensive cap */
  for (i = 0; i < count; i++) {
    snap_ring[i] = priv->tx_ring[i];
  }
  /* Note: we do NOT reset batch_outstanding here — TX_COMPLETE handler does
   * that after freeing echo skbs, so the TX queue remains stopped until the
   * H7 ACKs the batch. */
  spin_unlock_irqrestore(&priv->ring_lock, flags);

  if (count == 0) {
    /* Spurious wakeup (e.g., timer fired after start_xmit already queued
     * work that drained the ring) — no-op. */
#if DEBUG_X8H7_TX_TRACE
    pr_info("[x8h7_can_TRACE-4b] tx_work_handler: empty ring (spurious wakeup)\n");
#endif
    return;
  }

#if DEBUG_X8H7_TX_TRACE
  pr_info("[x8h7_can_TRACE-4] tx_work_handler: ENTRY periph=%u batch_count=%d\n",
          priv->periph, count);
#endif

  /* Track total bytes for stats (sum across all frames in batch) — populated
   * here before SPI exchange, reflects all frames being sent. TX_COMPLETE
   * handler bumps tx_packets per IRQ; tx_bytes is set once per batch. */
  priv->tx_len = 0;
  for (i = 0; i < count; i++) {
    priv->tx_len += snap_ring[i].field.len;
  }

#if DEBUG_X8H7_TX_TRACE
  pr_info("[x8h7_can_TRACE-5] tx_work_handler: pre x8h7_pkt_send_defer (batch=%d, total_bytes=%d)\n",
          count, priv->tx_len);
#endif

  /* Phase 7.4.42 Patch 0089 — defer all batched frames into the shared
   * spidev->x8h7_txb. Each defer takes/releases the spidev mutex, so other
   * peripherals or the threaded ISR can interleave between frames. If
   * x8h7_pkt_enq returns -ENOMEM (buffer full), we break early and trigger
   * a partial flush; remaining frames stay in the ring (kernel will issue
   * NETDEV_TX_BUSY on the next start_xmit attempt and reschedule). */
  for (i = 0; i < count; i++) {
    int defer_ret = x8h7_pkt_send_defer(priv->periph,
                       X8H7_CAN_OC_SEND,
                       X8H7_CAN_HEADER_SIZE + snap_ring[i].field.len,
                       snap_ring[i].buf);
    if (defer_ret < 0) {
      /* Buffer full in x8h7_drv. Flush what we've enqueued so far; the
       * remaining frames in priv->tx_ring will be flushed by a subsequent
       * work handler invocation triggered by either a new start_xmit hitting
       * MAX, or the deadline timer. */
#if DEBUG_X8H7_TX_TRACE
      pr_info("[x8h7_can_TRACE-5b] tx_work_handler: defer returned %d at frame %d/%d — partial flush\n",
              defer_ret, i, count);
#endif
      break;
    }
  }

  /* Phase 2 — single SPI exchange flushes the entire batch. */
  spi_ret = x8h7_pkt_send_now();

#if DEBUG_X8H7_TX_TRACE
  pr_info("[x8h7_can_TRACE-6] tx_work_handler: post x8h7_pkt_send_now ret=%d (sent %d frames)\n",
          spi_ret, i);
#endif
}

/* Phase 7.4.42 Patch 0089 — hrtimer callback for partial-batch flush.
 * Runs in softirq context; safe to call queue_work (which is irq-safe).
 * The work handler will read priv->batch_outstanding under ring_lock and
 * drain the ring. If batch_outstanding==0 (race: start_xmit already filled
 * the batch and queued work that ran before us), the handler treats it as
 * a spurious wakeup and returns. */
static enum hrtimer_restart x8h7_can_batch_timer_cb(struct hrtimer *t)
{
  struct x8h7_can_priv *priv = container_of(t, struct x8h7_can_priv, batch_timer);

  /* Only queue work if the workqueue is still alive (not destroyed by
   * x8h7_can_stop). priv->wq is set to NULL in stop; check before use. */
  if (priv->wq) {
    queue_work(priv->wq, &priv->work);
  }
  return HRTIMER_NORESTART;
}

/**
 */
static int x8h7_can_hw_do_set_bittiming(struct net_device *net)
{
  struct x8h7_can_priv *priv = netdev_priv(net);
  struct can_bittiming *bt = &priv->can.bittiming;
  union x8h7_can_bittiming_message x8h7_msg;

  DBG_PRINT("\n");

  x8h7_msg.field.baud_rate_prescaler = bt->brp;
  x8h7_msg.field.time_segment_1      = bt->prop_seg + bt->phase_seg1;
  x8h7_msg.field.time_segment_2      = bt->phase_seg2;
  x8h7_msg.field.sync_jump_width     = bt->sjw;

  DBG_PRINT("baud_rate_prescaler: %d, time_segment_1: %d, time_segment_2: %d, sync_jump_width: %d\n",
            x8h7_msg.field.baud_rate_prescaler,
            x8h7_msg.field.time_segment_1,
            x8h7_msg.field.time_segment_2,
            x8h7_msg.field.sync_jump_width);

  x8h7_pkt_send_sync(priv->periph, X8H7_CAN_OC_BITTIM, sizeof(x8h7_msg.buf), x8h7_msg.buf);

  return 0;
}

/*
 * candev callback used to change CAN mode.
 * Warning: this is called from a timer context!
 */
static int x8h7_can_do_set_mode(struct net_device *net, enum can_mode mode)
{
  int err = 0;

  DBG_PRINT("\n");

  switch (mode) {
  case CAN_MODE_START:
    err = x8h7_can_restart(net);
    if (err)
      netdev_err(net, "couldn't start device (err %d)\n",
				   err);
    break;
  default:
    return -EOPNOTSUPP;
  }

  return err;
}

/**
 */
static int x8h7_can_do_get_berr_counter(const struct net_device *net,
                                        struct can_berr_counter *bec)
{
  //@TODO: to be read from device
  bec->txerr = 0;
  bec->rxerr = 0;

  return 0;
}

/**
 */
static const struct net_device_ops x8h7_can_netdev_ops = {
  .ndo_open       = x8h7_can_open,
  .ndo_stop       = x8h7_can_stop,
  .ndo_start_xmit = x8h7_can_start_xmit,
  //.ndo_change_mtu = can_change_mtu,
};

/**
 */
static int x8h7_can_hw_config_filter(struct x8h7_can_priv *priv,
                                     uint32_t const idx,
                                     uint32_t const id,
                                     uint32_t const mask)
{
  union x8h7_can_filter_message x8h7_msg;

  x8h7_msg.field.idx  = idx;
  x8h7_msg.field.id   = id;
  x8h7_msg.field.mask = mask;

  DBG_PRINT("SEND idx %X, id %X, mask %X\n", idx, id, mask);

  x8h7_pkt_send_sync(priv->periph, X8H7_CAN_OC_FLT, sizeof(x8h7_msg.buf), x8h7_msg.buf);

  return 0;
}

/**
 * Standard id filter show
 */
static ssize_t x8h7_can_sf_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
  struct x8h7_can_priv *priv = netdev_priv(to_net_dev(dev));
  int                   len;
  int                   i;

  len = 0;
  for (i = 0; i < X8H7_STD_FLT_MAX; i++)
  {
    if (priv->std_flt[i].can_mask) {
      len += snprintf(buf + len, PAGE_SIZE - len,
                      "%02X %08X %08X\n",
                      i, priv->std_flt[i].can_id, priv->std_flt[i].can_mask);
    }
  }
  return len;
}

/**
 * Standard id filter set
 */
static ssize_t x8h7_can_sf_store(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf, size_t count)
{
  struct x8h7_can_priv *priv = netdev_priv(to_net_dev(dev));
  uint32_t              idx;
  uint32_t              id;
  uint32_t              mask;
  int                   ret;

  ret = sscanf(buf, "%x %x %x", &idx, &id, &mask);

  if (ret != 3) {
    DBG_ERROR("invalid num of params\n");
    return -EINVAL;
  }

  if ((idx >= X8H7_STD_FLT_MAX) ||
      (id & ~0x7FF) || (mask & ~0x7FF)) {
    DBG_ERROR("invalid params\n");
    return -EINVAL;
  }

  ret = x8h7_can_hw_config_filter(priv, idx, id, mask);
  if (ret) {
    DBG_ERROR("set filter\n");
    return -EIO;
  }

  priv->std_flt[idx].can_id   = id;
  priv->std_flt[idx].can_mask = mask;

  return count;
}

/**
 * Extended id filter show
 */
static ssize_t x8h7_can_ef_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
  struct x8h7_can_priv *priv = netdev_priv(to_net_dev(dev));
  int                   len;
  int                   i;

  len = 0;
  for (i = 0; i < X8H7_EXT_FLT_MAX; i++)
  {
    if (priv->ext_flt[i].can_mask) {
      len += snprintf(buf + len, PAGE_SIZE - len,
                      "%02X %08X %08X\n",
                      i, priv->ext_flt[i].can_id, priv->ext_flt[i].can_mask);
    }
  }
  return len;
}

/**
 * Extended id filter set
 */
static ssize_t x8h7_can_ef_store(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf, size_t count)
{
  struct x8h7_can_priv *priv = netdev_priv(to_net_dev(dev));
  int                   ret;
  uint32_t              idx;
  uint32_t              id;
  uint32_t              mask;

  ret = sscanf(buf, "%x %x %x", &idx, &id, &mask);

  if (ret != 3) {
    DBG_ERROR("invalid num of params\n");
    return -EINVAL;
  }

  if ((idx >= X8H7_EXT_FLT_MAX) ||
      (id & ~0x1FFFFFFF) || (mask & ~0x1FFFFFFF)) {
    DBG_ERROR("invalid params\n");
    return -EINVAL;
  }

  ret = x8h7_can_hw_config_filter(priv, idx, (CAN_EFF_FLAG | id), mask);
  if (ret) {
    DBG_ERROR("set filter\n");
    return -EIO;
  }

  priv->ext_flt[idx].can_id   = id;
  priv->ext_flt[idx].can_mask = mask;

  return count;
}

/**
 * Status show
 */
static ssize_t x8h7_can_sts_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
  struct x8h7_can_priv *priv = netdev_priv(to_net_dev(dev));
  int                   len;

  len = snprintf(buf, PAGE_SIZE,
                  "status         %d %s\n"
                  "error warning  %d\n"
                  "error passive  %d\n"
                  "bus off        %d\n"
                  "tx packets     %ld\n"
                  "tx bytes       %ld\n"
                  "rx packets     %ld\n"
                  "rx bytes       %ld\n"
                  "rx over_errors %ld\n"
                  "rx errors      %ld\n"
                  "rx dropped     %ld\n",
                  priv->can.state, can_sts(priv->can.state),
                  priv->can.can_stats.error_warning,
                  priv->can.can_stats.error_passive,
                  priv->can.can_stats.bus_off,
                  priv->net->stats.tx_packets,
                  priv->net->stats.tx_bytes,
                  priv->net->stats.rx_packets,
                  priv->net->stats.rx_bytes,
                  priv->net->stats.rx_over_errors,
                  priv->net->stats.rx_errors,
                  priv->net->stats.rx_dropped);
  return len;
}

static DEVICE_ATTR(std_flt, 0644, x8h7_can_sf_show, x8h7_can_sf_store);
static DEVICE_ATTR(ext_flt, 0644, x8h7_can_ef_show, x8h7_can_ef_store);
static DEVICE_ATTR(status , 0644, x8h7_can_sts_show, NULL);

static struct attribute *x8h7_can_sysfs_attrs[] = {
  &dev_attr_std_flt.attr,
  &dev_attr_ext_flt.attr,
  &dev_attr_status.attr,
  //....
  NULL,
};

static const struct attribute_group x8h7_can_sysfs_attr_group = {
  .name = "x8h7can",
  .attrs = (struct attribute **)x8h7_can_sysfs_attrs,
};

/**
 */
static int x8h7_can_probe(struct platform_device *pdev)
{
  struct net_device    *net;
  struct x8h7_can_priv *priv;
  int                   err;

  u32 clock_freq = 100000000;

  DBG_PRINT("\n");

  if (pdev->dev.of_node) {
    of_property_read_u32(pdev->dev.of_node, "clock-frequency", &clock_freq);
    DBG_PRINT("fdcan_clk = %d", clock_freq);
  }

  /* Phase 7.4.42 Patch 0089 — TX batching requires N outstanding echo skbs
   * (one per batched frame); bump max_tx_echo_skbs from 1 → X8H7_CAN_BATCH_MAX
   * so can_put_echo_skb(net, idx, 0) succeeds for idx 0..BATCH_MAX-1 */
  net = alloc_candev(sizeof(struct x8h7_can_priv), X8H7_CAN_BATCH_MAX);
  if (!net) {
    return -ENOMEM;
  }

  net->netdev_ops = &x8h7_can_netdev_ops;
  net->flags |= IFF_ECHO;
  net->sysfs_groups[0] = &x8h7_can_sysfs_attr_group;

  priv = netdev_priv(net);

  priv->can.clock.freq             = clock_freq;
  priv->can.bittiming_const        = &x8h7_can_bittiming_const;
  priv->can.data_bittiming_const   = &x8h7_can_data_bittiming_const;
  priv->can.do_set_bittiming       = x8h7_can_hw_do_set_bittiming;
  priv->can.do_set_data_bittiming  = x8h7_can_hw_do_set_data_bittiming;
  priv->can.do_set_mode            = x8h7_can_do_set_mode;
  priv->can.do_get_berr_counter    = x8h7_can_do_get_berr_counter;
  priv->can.ctrlmode_supported     = CAN_CTRLMODE_LOOPBACK      |
                                     CAN_CTRLMODE_LISTENONLY    |
                                     CAN_CTRLMODE_FD            |
                                     CAN_CTRLMODE_3_SAMPLES     ;
  priv->net = net;

  platform_set_drvdata(pdev, priv);

  SET_NETDEV_DEV(net, &pdev->dev);

  err = register_candev(net);
  if (err) {
    dev_err(&pdev->dev, "registering netdev failed\n");
    goto failed_register;
  }
  DBG_PRINT("net device registered %s, "
            "ifindex: %d, if_port %d, dev_id: %d, dev_port %d\n",
            net->name, net->ifindex, net->if_port, net->dev_id, net ->dev_port);

  if (net->name[3] == '0') {
    priv->periph = X8H7_CAN1_PERIPH;
  } else {
    priv->periph = X8H7_CAN2_PERIPH;
  }
  priv->dev = &pdev->dev;
  DBG_PRINT("periph: %d DONE\n", priv->periph);

  netdev_info(net, "X8H7 CAN successfully initialized.\n");

#ifdef CONFIG_X8H7_CAN_DIAG_COUNTERS
  x8h7_can_diag_init();
#endif
  return 0;

failed_register:
  DBG_ERROR("\n");
  free_candev(net);
  return err;
}

/**
 */
static int x8h7_can_remove(struct platform_device *pdev)
{
  struct x8h7_can_priv *priv = platform_get_drvdata(pdev);
  struct net_device    *net = priv->net;

#ifdef CONFIG_X8H7_CAN_DIAG_COUNTERS
  x8h7_can_diag_exit();
#endif
  unregister_candev(net);
  free_candev(net);

  return 0;
}

/**
 */
static const struct of_device_id x8h7_can_of_match[] = {
  { .compatible = "portenta,x8h7_can", },
  { /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, x8h7_can_of_match);

/**
 */
static const struct platform_device_id x8h7_can_id_table[] = {
  { .name = "x8h7_can", },
  { /* sentinel */ },
};
MODULE_DEVICE_TABLE(platform, x8h7_can_id_table);

/**
 */
static struct platform_driver x8h7_can_driver = {
  .driver = {
    .name           = DRIVER_NAME,
    .of_match_table = x8h7_can_of_match,
  },
  .probe    = x8h7_can_probe,
  .remove   = x8h7_can_remove,
  .id_table = x8h7_can_id_table,
};

module_platform_driver(x8h7_can_driver);

MODULE_AUTHOR("Massimiliano Agneni <massimiliano@iptronix.com");
MODULE_DESCRIPTION("Arduino Portenta X8 CAN driver");
MODULE_LICENSE("GPL v2");
