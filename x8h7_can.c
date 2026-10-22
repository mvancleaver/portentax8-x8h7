/**
 * X8H7 CAN driver
 */

#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/sched.h>
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

/* Phase 7.1.6.1 (Option c) / Patch 0082: every TX/RX TRACE pr_info is gated behind this (default 0 = off; ~36 ms/call on the LmP console). */
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

static void x8h7_can_tx_work_handler(struct kthread_work *ws);
static int x8h7_can_hw_setup(struct x8h7_can_priv *priv);
static int x8h7_can_hw_stop(struct x8h7_can_priv *priv);
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

/* Phase 07.4.42.2 P0-4 — retire @count oldest in-flight frames.
 * ok=true: deliver the echo (tx_packets/tx_bytes); ok=false: drop the echo
 * (tx_dropped). Clamped to the in-flight count, so an over-reporting H7 can
 * never retire a frame that was not sent. Called from the x8h7 threaded IRQ
 * (spidev->lock held) — never touches SPI. can_get_echo_skb() -> netif_rx()
 * runs with BH disabled, which is valid (NET_RX runs at spin_unlock_bh). */
static void x8h7_can_tx_retire(struct x8h7_can_priv *priv, u32 count, bool ok)
{
  struct net_device *net = priv->net;
  u32 inflight, i;

  if (!count)
    return;

  spin_lock_bh(&priv->ring_lock);
  inflight = priv->tx_head - priv->tx_inflight;
  if (count > inflight)
    count = inflight;
  for (i = 0; i < count; i++) {
    unsigned int idx = priv->tx_inflight & X8H7_CAN_TX_RING_MASK;

    if (ok) {
      net->stats.tx_bytes += can_get_echo_skb(net, idx, NULL);
      net->stats.tx_packets++;
    } else {
      can_free_echo_skb(net, idx, NULL);
      net->stats.tx_dropped++;
    }
    priv->tx_inflight++;
  }
  if (count && netif_queue_stopped(net) &&
      (priv->tx_tail - priv->tx_inflight) < X8H7_CAN_TX_RING_SIZE)
    netif_wake_queue(net);
  spin_unlock_bh(&priv->ring_lock);
}

/* Phase 07.4.42.2 P0-4 — drop every queued/in-flight frame and resynchronise
 * the ring (ndo_tx_timeout, CAN_MODE_START, ndo_stop). Bumping tx_gen tells a
 * concurrently running work handler not to un-claim or re-queue the frames it
 * snapshotted before the reset. All echo slots are freed (self-healing even if
 * an echo leaked outside [tx_inflight, tx_tail)); tx_dropped counts only slots
 * that still held an echo. REVIEW CR-01: this is the only echo flush that is
 * serialised with x8h7_can_tx_retire() (ring_lock). ndo_stop and the BUS_OFF
 * entry run it BEFORE the core's lock-free can_flush_echo_skb()
 * (close_candev() / can_restart()), so that flush always finds every slot
 * empty. */
static void x8h7_can_tx_ring_reset(struct x8h7_can_priv *priv, bool wake)
{
  struct net_device *net = priv->net;
  u32 dropped = 0;
  unsigned int idx;

  spin_lock_bh(&priv->ring_lock);
  for (idx = 0; idx < X8H7_CAN_TX_RING_SIZE; idx++) {
    if (priv->can.echo_skb[idx]) {
      can_free_echo_skb(net, idx, NULL);
      dropped++;
    }
  }
  net->stats.tx_dropped += dropped;
  priv->tx_head     = priv->tx_tail;
  priv->tx_inflight = priv->tx_tail;
  priv->tx_gen++;
  spin_unlock_bh(&priv->ring_lock);
  hrtimer_try_to_cancel(&priv->batch_timer);
  if (wake)
    netif_wake_queue(net);
}

/* Phase 07.4.42.2 P1-4 — error state from the H7 4-byte report
 * {INT_ERR, flags, TEC, REC} (new H7 fdcan_report_error_state(): PSR BusOff ->
 * TX_BO, ErrorPassive -> TX_EP|RX_EP, Warning -> EWARN, none -> 0x00).
 * Replaces Patch 0090's mapping of a per-frame TX submit failure (TX_EP) to
 * ERROR_PASSIVE and its shared can_id/data1 accumulator (IN-09). Leaving
 * BUS_OFF is reserved to the restart path (can_restart() ->
 * CAN_MODE_START -> x8h7_can_restart_work_fn), so a late non-bus-off report
 * cannot leave the kernel at ERROR_ACTIVE with the carrier still off.
 * Threaded IRQ context: no SPI, no printk. */
static void x8h7_can_error_state(struct x8h7_can_priv *priv, u8 flags, u8 tec, u8 rec)
{
  struct net_device *net = priv->net;
  enum can_state     new_state, tx_state, rx_state;
  struct can_frame  *cf = NULL;
  struct sk_buff    *skb;

  priv->last_tec = tec;
  priv->last_rec = rec;

  if (flags & X8H7_CAN_STS_FLG_TX_BO)
    new_state = CAN_STATE_BUS_OFF;
  else if (flags & (X8H7_CAN_STS_FLG_TX_EP | X8H7_CAN_STS_FLG_RX_EP))
    new_state = CAN_STATE_ERROR_PASSIVE;
  else if (flags & X8H7_CAN_STS_FLG_EWARN)
    new_state = CAN_STATE_ERROR_WARNING;
  else
    new_state = CAN_STATE_ERROR_ACTIVE;

  if (new_state == priv->can.state)
    return;   /* can_change_state() would warn "state did not change" */
  if (priv->can.state == CAN_STATE_BUS_OFF)
    return;   /* only the restart path leaves bus-off */

  tx_state = (tec >= rec) ? new_state : CAN_STATE_ERROR_ACTIVE;
  rx_state = (rec >= tec) ? new_state : CAN_STATE_ERROR_ACTIVE;

  skb = alloc_can_err_skb(net, &cf);
  can_change_state(net, skb ? cf : NULL, tx_state, rx_state);
  if (skb) {
    cf->can_id |= CAN_ERR_CNT;
    cf->data[6] = tec;
    cf->data[7] = rec;
    netif_rx(skb);
  }
  if (new_state == CAN_STATE_BUS_OFF) {
    /* 07.4.42.2 REVIEW CR-01 — the core can_restart() (restart_ms work) runs
     * can_flush_echo_skb() with no driver lock. Empty the ring here, under
     * ring_lock and in the same thread as x8h7_can_tx_retire(), so no echo is
     * in flight when that flush runs. priv->can.state is already BUS_OFF
     * (can_change_state() above), so start_xmit drops every frame from now on
     * instead of storing an echo.
     *
     * The queue is deliberately left AWAKE (ring reset with wake=true, even if
     * a full ring had stopped it): a stopped queue parks up to txqueuelen
     * (128) frames in the qdisc for the whole restart_ms window, and
     * carrier-off is a non-urgent linkwatch event (net/core/link_watch.c,
     * rate-limited to 1/s) so dev_deactivate() may or may not flush them
     * before the restart wakes the queue -- a replay of ~43 ms of stale
     * MD80 SET_TARGETS into freshly restarted motors, timing-dependent.
     * With the queue awake every frame reaches start_xmit and is dropped
     * (tx_dropped) deterministically while BUS_OFF; the ring stays empty.
     * x8h7_can_restart() stops the queue only for the H7 re-init itself and
     * x8h7_can_restart_work_fn() publishes ERROR_ACTIVE + wakes it. */
    x8h7_can_tx_ring_reset(priv, true);
    can_bus_off(net);   /* carrier off + restart_ms timer; no SPI */
  }
}

/* Phase 07.4.42.2 — CAN_STATUS (opcode 0x40, H7 -> kernel) per the shared
 * WIRE CONTRACT (07.4.42.2-04-PLAN <interfaces>; the same table is in the H7
 * Plans 05/06). Accepts both the new H7 firmware and the legacy one (md5
 * ebbd8d8e):
 *   {TX_COMPLETE, n_finished, n_aborted}  3 B new    retire n_finished:
 *                                                    (n - aborted) ok, aborted dropped
 *   {TX_COMPLETE, mask}                   2 B legacy retire hweight8(mask) ok
 *   {TX_ABORT_COMPLETE, mask}             2 B legacy retire hweight8(mask) dropped
 *   {INT_ERR, TX_OVR[, 0]}              2|3 B        tx_fifo_errors + error skb;
 *                                                    retire 1 dropped only if 2 B
 *   {INT_ERR, TX_EP[, 0]}               2|3 B        tx_errors (submit failure, not a
 *                                                    state change); retire 1 dropped only if 2 B
 *   {INT_ERR, RX_OVR, n_lost}             3 B new    rx_over_errors += max(n_lost, 1)
 *   {INT_ERR, flags, TEC, REC}            4 B new    error-state report
 *   {TX_FIFO_EMPTY, free}                 2 B        ignored
 * Inside INT_ERR, size >= 4 is an error-state report, < 4 per-event flags.
 * Runs in the x8h7 threaded IRQ with spidev->lock held: must never call
 * x8h7_pkt_send_*. No printk here (Pitfall 6, ~36 ms/call on the console). */
static void x8h7_can_status(struct x8h7_can_priv *priv, x8h7_pkt_t *pkt)
{
  struct net_device *net   = priv->net;
  u8                 intf  = pkt->data[0];
  u8                 eflag = pkt->data[1];

  if (intf == 0x20 && pkt->size >= 8) {
    /* 2026-10-22 diag: H7 heartbeat {0x20, seq, loop8, txsub8, txrep8, rxfr8, repulse8, spierr8} every 100 ms */
    memcpy(priv->hb_last, pkt->data, 8);
    priv->hb_jiffies = jiffies;
    priv->hb_count++;
    return;
  }
  if (intf & X8H7_CAN_STS_INT_ERR) {
    if (pkt->size >= 4) {
      x8h7_can_error_state(priv, eflag, pkt->data[2], pkt->data[3]);
    } else {
      if (eflag & X8H7_CAN_STS_FLG_RX_OVR) {
        u32 lost = (pkt->size >= 3 && pkt->data[2]) ? pkt->data[2] : 1;

        net->stats.rx_over_errors += lost;
        net->stats.rx_errors      += lost;
        x8h7_can_error_skb(net, CAN_ERR_CRTL, CAN_ERR_CRTL_RX_OVERFLOW);
      }
      if (eflag & X8H7_CAN_STS_FLG_TX_OVR) {
        /* H7 Tx FIFO full, frame rejected. The new H7 also counts the
         * reject in its next TX_COMPLETE (3 B form) — retire only for 2 B. */
        net->stats.tx_fifo_errors++;
        net->stats.tx_errors++;
        x8h7_can_error_skb(net, CAN_ERR_CRTL, CAN_ERR_CRTL_TX_OVERFLOW);
        if (pkt->size < 3)
          x8h7_can_tx_retire(priv, 1, false);
      }
      if (eflag & X8H7_CAN_STS_FLG_TX_EP) {
        /* per-frame submit failure, NOT error-passive — RESEARCH P1-4 */
        net->stats.tx_errors++;
        if (pkt->size < 3)
          x8h7_can_tx_retire(priv, 1, false);
      }
    }
  }

  if (intf & X8H7_CAN_STS_INT_TX_COMPLETE) {
    if (pkt->size >= 3) {
      u32 done    = pkt->data[1];
      u32 aborted = min_t(u32, pkt->data[2], done);

      x8h7_can_tx_retire(priv, done - aborted, true);
      x8h7_can_tx_retire(priv, aborted, false);
    } else {
      /* legacy sticky TXBTO mask; over-reports are clamped in retire */
      x8h7_can_tx_retire(priv, hweight8(pkt->data[1]), true);
    }
  }

  if (intf & X8H7_CAN_STS_INT_TX_ABORT_COMPLETE) {
    DBG_PRINT("TX ABORT COMPLETE");
    x8h7_can_tx_retire(priv, hweight8(pkt->data[1]), false);
  }

  if (intf & X8H7_CAN_STS_INT_TX_FIFO_EMPTY) {
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

      /* Phase 07.4.42.2 P1-9 — field.len comes from the wire: drop (and
       * count) a subpacket that is shorter than its header claims or longer
       * than a CAN-FD frame, and an FD-length frame on a classic-mode
       * interface (it would overrun the 8-byte can_frame below). */
      if (x8h7_can_msg.field.len > X8H7_CAN_FRAME_MAX_DATA_LEN ||
          pkt->size < X8H7_CAN_HEADER_SIZE + x8h7_can_msg.field.len ||
          (x8h7_can_msg.field.len > CAN_MAX_DLEN &&
           !(priv->can.ctrlmode & CAN_CTRLMODE_FD))) {
        priv->net->stats.rx_length_errors++;
        priv->net->stats.rx_errors++;
        return;
      }
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
    /* Phase 7.4.42.1 Patch 0090 (T4) -- pass full pkt so x8h7_can_status
     * can access size + data[2]/[3] for TEC/REC (length-guarded by size >= 4). */
    if (pkt->size >= 2)
      x8h7_can_status(priv, pkt);
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
 * Bus-off restart. Kernel 6.1 drivers/net/can/dev/dev.c can_restart() runs from
 * the can_priv restart delayed work (restart_ms) or the IFLA_CAN_RESTART netlink
 * request — process context in both cases — and calls do_set_mode(CAN_MODE_START)
 * after can_flush_echo_skb() and the CAN_ERR_RESTARTED error frame. The H7
 * re-init itself still runs on priv->wq (Patch 0090 structure).
 */

/* Phase 7.4.42.1 Patch 0090 (T3) / 07.4.42.2 P1-4 — restart_work handler.
 * Runs in priv->wq workqueue context (WQ_HIGHPRI). x8h7_can_hw_stop/_setup
 * call x8h7_pkt_send_sync(), which takes the spidev mutex and may sleep —
 * safe here. The TX queue was stopped by x8h7_can_restart() and is woken only
 * once the H7 has been re-initialised. */
static void x8h7_can_restart_work_fn(struct kthread_work *ws)
{
  struct x8h7_can_priv *priv = container_of(ws, struct x8h7_can_priv, restart_work);
  struct net_device    *net  = priv->net;
  int                   ret;

  DBG_PRINT("bus-off restart: re-init FDCAN via x8h7_can_hw_stop + x8h7_can_hw_setup\n");
  x8h7_can_hw_stop(priv);
  ret = x8h7_can_hw_setup(priv);
  if (ret) {
    netdev_err(net, "bus-off restart: H7 re-init failed (%d); TX queue left stopped\n", ret);
    return;
  }

  /* No CAN_ERR_RESTARTED skb here: drivers/net/can/dev/dev.c can_restart()
   * already emitted it before calling do_set_mode(CAN_MODE_START). Patch 0090's
   * second emission (REVIEW WR-02) was a duplicate — RESEARCH P1-4. */
  /* Reset cached TEC/REC so `ip -d link show can1` berr-counter does not report
   * stale near-bus-off counters after a healthy restart (REVIEW IN-03). */
  priv->last_tec = 0;
  priv->last_rec = 0;

  /* Defensive: priv->can.state is still BUS_OFF here (the 6.1 core
   * can_restart() does not touch it), so start_xmit has dropped everything
   * since bus-off entry and the ring should already be empty. Reset it anyway
   * so the ring is exactly empty when ERROR_ACTIVE is published and the queue
   * is woken below. */
  x8h7_can_tx_ring_reset(priv, false);

  /* Transition back to ERROR_ACTIVE and wake the TX queue. The state is
   * published under ring_lock (REVIEW CR-01): start_xmit tests it under the
   * same lock, so it stops dropping only after the ring reset above. */
  spin_lock_bh(&priv->ring_lock);
  WRITE_ONCE(priv->can.state, CAN_STATE_ERROR_ACTIVE);
  spin_unlock_bh(&priv->ring_lock);
  netif_wake_queue(net);
}

static int x8h7_can_restart(struct net_device *net)
{
  struct x8h7_can_priv *priv  = netdev_priv(net);
  int err = 0;

  DBG_PRINT("\n");

  /* Phase 7.4.42.1 Patch 0090 (T3) — defer the H7 re-init (x8h7_pkt_send_sync
   * takes the spidev mutex) to priv->wq. Guard: if wq is NULL (x8h7_can_stop
   * already ran) skip to avoid use-after-free — the bus is being torn down.
   * Phase 07.4.42.2: keep start_xmit out until restart_work has re-initialised
   * the H7 (the core turns the carrier back on as soon as we return), so no
   * frame races the DEINIT/INIT pair. This is the ONLY place the queue is
   * stopped for bus-off (entry keeps it awake so start_xmit drops; see
   * x8h7_can_error_state()); the window is bounded by the H7 re-init and
   * restart_work_fn resets the ring before waking, so nothing parked here can
   * replay. netif_trans_update() stops dev_watchdog from treating the stopped
   * queue as a TX stall in the meantime. */
  if (priv->wq) {
    netif_trans_update(net);
    netif_stop_queue(net);
    kthread_queue_work(priv->wq, &priv->restart_work);
  } else {
    netdev_warn(net, "x8h7_can_restart: wq=NULL (stop in progress), skipping H7 re-init\n");
    /* Still update state so the kernel CAN core is consistent. */
    priv->can.state = CAN_STATE_ERROR_ACTIVE;
    netif_wake_queue(net);
  }

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

  /* 07.4.42.2 IN-08: propagate x8h7_pkt_send_sync() failure */
  return x8h7_pkt_send_sync(priv->periph, X8H7_CAN_OC_INIT, sizeof(x8h7_msg.buf), x8h7_msg.buf) < 0 ? -EIO : 0;
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
  /* 07.4.42.2 IN-08: propagate x8h7_pkt_send_sync() failure */
  return x8h7_pkt_send_sync(priv->periph, X8H7_CAN_OC_DEINIT, 0, NULL) < 0 ? -EIO : 0;
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
 * runs in hard-IRQ context; it only calls queue_work (which is irq-safe).
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

  /* Phase 7.4.42 Patch 0089 / 07.4.42.2 P0-4 — initialize TX ring state.
   * spinlock (not mutex): start_xmit runs with BH disabled and lock holders
   * are short (ring indices + tx_ring copies; NEVER held while calling
   * x8h7_pkt_send_defer/now which take the spidev mutex and may sleep).
   * Echo slots are empty here (x8h7_can_stop() emptied the ring at the last
   * stop, REVIEW CR-01), so the counters restart at 0. */
  spin_lock_init(&priv->ring_lock);
  priv->tx_tail = priv->tx_head = priv->tx_inflight = 0;
  priv->tx_gen = 0;
  hrtimer_init(&priv->batch_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
  priv->batch_timer.function = x8h7_can_batch_timer_cb;

  /* 07.4.42.2 bench 2026-10-22: dedicated SCHED_FIFO kthread (see x8h7_can.h). comm is 15 chars:
   * "x8h7cantx-can1". x8-rt-promote.sh raises it to io-thread+1 (91 with --prio 95). */
  priv->wq = kthread_create_worker(0, "x8h7cantx-%s", net->name);
  if (IS_ERR_OR_NULL(priv->wq)) {
    priv->wq = NULL;
    ret = -ENOMEM;
    goto out_clean;
  }
  sched_set_fifo(priv->wq->task);   /* FIFO 50 by default; above every CFS task, below irq/<n>-x8h7 */
  kthread_init_work(&priv->work, x8h7_can_tx_work_handler);
  kthread_init_work(&priv->restart_work, x8h7_can_restart_work_fn);

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
  kthread_destroy_worker(priv->wq);
  priv->wq = NULL;   /* 07.4.42.2 REVIEW IN-03: x8h7_can_restart() and the batch timer test priv->wq */
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

  /* 07.4.42.2 REVIEW CR-01 — close_candev() runs the core
   * can_flush_echo_skb() with no driver lock, while the x8h7 threaded IRQ
   * retires echo slots under ring_lock. Detach the hook first (no new
   * TX-status retire), then empty the ring under ring_lock: every echo is
   * freed and tx_inflight = tx_tail, so a retire that is already running
   * clamps to 0. __dev_close_many() quiesced start_xmit before ndo_stop, so
   * no new echo can be stored after this reset, and the core flush below
   * finds every slot empty. The reset also drops anything still queued, so a
   * later open starts clean (P0-4). */
  x8h7_hook_set(priv->periph, NULL, NULL);
  x8h7_can_tx_ring_reset(priv, false);

  close_candev(net);   /* cancels the core restart work; its echo flush is now a no-op */

  /* 07.4.42.2: a bus-off restart_work still queued must not re-init the H7
   * (and wake the queue) after the DEINIT below. */
  kthread_cancel_work_sync(&priv->restart_work);

  /* Notify ext. hw to stop can peripheral */
  x8h7_can_hw_stop(priv);

  /* Phase 7.4.42 Patch 0089 — cancel batch flush timer BEFORE destroying
   * workqueue. hrtimer_cancel() is safe to call even if timer never fired. */
  hrtimer_cancel(&priv->batch_timer);

  /* Free priv. resources */
  mutex_lock(&priv->lock);
  kthread_destroy_worker(priv->wq);
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
  struct canfd_frame          *cfd;
  bool                         is_fd;
  unsigned int                 slot;
  u32                          pending;

  DBG_PRINT("\n");

#if DEBUG_X8H7_TX_TRACE
  pr_info("[x8h7_can_TRACE-1] start_xmit: dev=%s skb_len=%u state=%d qstop=%d tail=%u head=%u inflight=%u\n",
          net->name, skb->len, priv->can.state, netif_queue_stopped(net),
          priv->tx_tail, priv->tx_head, priv->tx_inflight);
#endif

  if (can_dropped_invalid_skb(net, skb))
    return NETDEV_TX_OK;

  is_fd = can_is_canfd_skb(skb);
  cfd   = (struct canfd_frame *)skb->data;

  /* Phase 07.4.42.2 P0-4 — append at tx_tail. The echo skb is put under
   * ring_lock BEFORE the slot is published (tx_tail++), so a TX_COMPLETE can
   * never retire a slot whose echo is not there yet, and a slot is reused only
   * after it was retired (no "echo_skb is occupied"). The work handler sends
   * [tx_head, tx_tail); the status handler retires from tx_inflight. */
  spin_lock_bh(&priv->ring_lock);
  if (READ_ONCE(priv->can.state) == CAN_STATE_BUS_OFF) {
    /* 07.4.42.2 REVIEW CR-01 — no new echo while bus-off: the ring was
     * emptied at bus-off entry and the core can_restart() flushes the echo
     * slots with no driver lock. The queue is kept awake in BUS_OFF on
     * purpose so every frame lands here and is dropped NOW rather than
     * parked in the qdisc and replayed after the restart (stale motor
     * targets). x8h7_can_restart_work_fn() publishes ERROR_ACTIVE under
     * ring_lock once the H7 has been re-initialised. */
    net->stats.tx_dropped++;
    spin_unlock_bh(&priv->ring_lock);
    kfree_skb(skb);
    return NETDEV_TX_OK;
  }
  if (priv->tx_tail - priv->tx_inflight >= X8H7_CAN_TX_RING_SIZE) {
    /* Ring full — normally unreachable (we stop the queue when the last slot
     * is taken below). */
    netif_stop_queue(net);
    spin_unlock_bh(&priv->ring_lock);
    return NETDEV_TX_BUSY;
  }
  slot = priv->tx_tail & X8H7_CAN_TX_RING_MASK;
  x8h7_can_frame_to_tx_obj(cfd, &priv->tx_ring[slot], is_fd);
  if (can_put_echo_skb(skb, net, slot, 0)) {
    /* -ENOMEM / -EBUSY: the skb was freed by can_put_echo_skb() */
    net->stats.tx_dropped++;
    spin_unlock_bh(&priv->ring_lock);
    return NETDEV_TX_OK;   /* skb consumed in all cases */
  }
  priv->tx_tail++;
  pending = priv->tx_tail - priv->tx_head;
  if (priv->tx_tail - priv->tx_inflight >= X8H7_CAN_TX_RING_SIZE)
    netif_stop_queue(net);   /* woken by x8h7_can_tx_retire / ring reset */
  spin_unlock_bh(&priv->ring_lock);

  if (pending >= X8H7_CAN_BATCH_MAX) {
    /* Full batch — flush now; the immediate queue_work supersedes the timer. */
    hrtimer_try_to_cancel(&priv->batch_timer);
    kthread_queue_work(priv->wq, &priv->work);
  } else if (!hrtimer_is_queued(&priv->batch_timer)) {
    /* Partial batch — arm the deadline timer so it flushes even at low TX
     * rate. hrtimer_is_queued (not hrtimer_active): re-arming while the
     * callback is running is legal and closes the window in which the
     * callback's queue_work already ran before this frame was published. */
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
 * released; the subsequent `x8h7_pkt_send_now()` re-takes it for the SPI
 * exchange (~0.3 ms measured p50 per SPI exchange (Phase 7.4.41)).
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
 * acquires the full mutex for the whole SPI exchange. The TX vs RX SPI
 * exchanges remain structurally serialized; full collapse to one full-duplex
 * exchange would require relying on the next IRQ-driven exchange to flush
 * pending TX (deadlock risk if no IRQ arrives — out of scope for Plan 02).
 */
static void x8h7_can_tx_work_handler(struct kthread_work *ws)
{
  struct x8h7_can_priv *priv = container_of(ws, struct x8h7_can_priv, work);
  /* Phase 7.4.42 Patch 0089 / 07.4.42.2 P0-4: up to X8H7_CAN_BATCH_MAX frames
   * are deferred into spidev->x8h7_txb and flushed by ONE x8h7_pkt_send_now().
   * The snapshot is taken under ring_lock, which is released before any
   * x8h7_pkt_send_* call (spidev mutex; may sleep). */
  union x8h7_can_frame_message snap[X8H7_CAN_BATCH_MAX];
  u32  start, n, sent, gen, bytes;
  bool more;

  spin_lock_bh(&priv->ring_lock);
  start = priv->tx_head;
  for (n = 0, bytes = 0; n < X8H7_CAN_BATCH_MAX && start + n != priv->tx_tail; n++) {
    union x8h7_can_frame_message const *f = &priv->tx_ring[(start + n) & X8H7_CAN_TX_RING_MASK];
    u32 sz = X8H7_CAN_SUBPKT_HDR_SIZE + X8H7_CAN_HEADER_SIZE + f->field.len;

    if (n && bytes + sz > X8H7_CAN_SPI_BUDGET)
      break;   /* rest goes in the next exchange (re-queued below) */
    bytes += sz;
    snap[n] = *f;
  }
  priv->tx_head = start + n;   /* claim BEFORE deferring: the threaded IRQ may flush spidev->x8h7_txb and
                                * an early TX_COMPLETE must be able to retire these frames */
  gen = priv->tx_gen;
  spin_unlock_bh(&priv->ring_lock);

  if (n == 0) {
    /* Spurious wakeup (timer fired after a full-batch flush drained the ring,
     * or the ring was reset) — no-op. */
    return;
  }

#if DEBUG_X8H7_TX_TRACE
  pr_info("[x8h7_can_TRACE-4] tx_work_handler: periph=%u start=%u n=%u bytes=%u\n",
          priv->periph, start, n, bytes);
#endif

  for (sent = 0; sent < n; sent++)
    if (x8h7_pkt_send_defer(priv->periph, X8H7_CAN_OC_SEND,
                            X8H7_CAN_HEADER_SIZE + snap[sent].field.len, snap[sent].buf) < 0)
      break;   /* spidev txb full: un-claim the rest below, they go in the next exchange */

  spin_lock_bh(&priv->ring_lock);
  /* Un-claim the frames that were not deferred — unless the ring was reset
   * meanwhile (tx_gen) or a status report already retired past them. */
  if (sent < n && gen == priv->tx_gen && priv->tx_head == start + n &&
      (s32)(priv->tx_inflight - (start + sent)) <= 0)
    priv->tx_head = start + sent;
  more = (gen == priv->tx_gen) && (priv->tx_tail != priv->tx_head);
  spin_unlock_bh(&priv->ring_lock);

  {
    /* 2026-10-22 bench: the 600 s run dies on a transient (>= 300 ms) silence of the whole link. Time the
     * exchange (includes waiting for the spidev mutex held by the IRQ thread's exchange): a long value here
     * means the SPI transfer or its lock hung, a short value with no replies means the H7 did not transmit. */
    ktime_t const t0 = ktime_get();
    s64 us;
    x8h7_pkt_send_now();   /* flushes our frames and anything else queued, frees txb for the remainder */
    us = ktime_us_delta(ktime_get(), t0);
    if (us > priv->tx_xchg_max_us) priv->tx_xchg_max_us = us;
    if (us > 2000) {
      priv->tx_xchg_slow++;
      if (net_ratelimit())
        netdev_warn(priv->net, "SPI exchange (pkt_send_now) took %lld us (slow #%u, max %lld us, inflight %u)\n",
                    (long long)us, priv->tx_xchg_slow, (long long)priv->tx_xchg_max_us,
                    priv->tx_head - priv->tx_inflight);
    }
  }

#if DEBUG_X8H7_TX_TRACE
  pr_info("[x8h7_can_TRACE-6] tx_work_handler: flushed sent=%u/%u more=%d\n", sent, n, more);
#endif

  if (more && priv->wq)
    kthread_queue_work(priv->wq, &priv->work);
}

/* Phase 7.4.42 Patch 0089 — hrtimer callback for partial-batch flush.
 * HRTIMER_MODE_REL timers expire in hard-IRQ context on this (non-RT) kernel:
 * only queue_work() here (irq-safe), never ring_lock. The work handler reads
 * [tx_head, tx_tail) under ring_lock; an empty range (a full-batch flush or a
 * ring reset got there first) is a spurious wakeup and returns. */
static enum hrtimer_restart x8h7_can_batch_timer_cb(struct hrtimer *t)
{
  struct x8h7_can_priv *priv = container_of(t, struct x8h7_can_priv, batch_timer);

  /* Only queue work if the workqueue is still alive (not destroyed by
   * x8h7_can_stop). priv->wq is set to NULL in stop; check before use. */
  if (priv->wq) {
    kthread_queue_work(priv->wq, &priv->work);
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
 * Kernel 6.1: called by can_restart() from the can_priv restart delayed work
 * (restart_ms) or from the IFLA_CAN_RESTART netlink request — process context,
 * not a timer.
 */
static int x8h7_can_do_set_mode(struct net_device *net, enum can_mode mode)
{
  int err = 0;

  DBG_PRINT("\n");

  switch (mode) {
  case CAN_MODE_START:
    /* 07.4.42.2 P0-4 scenario 5 / REVIEW CR-01 — the ring was already
     * emptied at bus-off entry (x8h7_can_error_state) and start_xmit drops
     * while bus-off, so the core can_restart()'s lock-free
     * can_flush_echo_skb() found no echo. Resynchronise the ring indices and
     * drop every pre-bus-off command so nothing stale is replayed after the
     * restart. */
    x8h7_can_tx_ring_reset(netdev_priv(net), false);
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
  struct x8h7_can_priv *priv = netdev_priv((struct net_device *)net);
  /* Phase 7.4.42.1 Patch 0090 (T4) -- return real TEC/REC cached by
   * x8h7_can_status() from the 4-byte H7 status payload (Patch 0091 T4).
   * Updated on every error-state-change (low-rate; no per-frame cost). */
  bec->txerr = priv->last_tec;
  bec->rxerr = priv->last_rec;

  return 0;
}

/* Phase 07.4.42.2 P0-4 — bounded recovery from a lost H7 status packet
 * (arduino/portentax8-x8h7#10, portentax8-stm32h7-fw#74). dev_watchdog calls
 * this from the timer softirq when the queue has been stopped for longer than
 * the X8H7_CAN_TX_WATCHDOG_MS deadline. The ring is reset (all echoes
 * dropped) and the queue woken — except in BUS_OFF, where only a successful
 * restart may wake it (the H7 has not been re-initialised). Note: dev_watchdog prints a one-time
 * "NETDEV WATCHDOG ... transmit queue 0 timed out" WARN before calling us. */
static void x8h7_can_tx_timeout(struct net_device *net, unsigned int txqueue)
{
  struct x8h7_can_priv *priv = netdev_priv(net);

  x8h7_can_tx_ring_reset(priv, priv->can.state != CAN_STATE_BUS_OFF);
  net->stats.tx_errors++;
  if (net_ratelimit())
    netdev_warn(net, "TX stalled (lost H7 status?) -- ring reset, %s; H7 heartbeat #%u age %u ms "
                "{seq %u loop %u txsub %u txrep %u rxfr %u repulse %u spierr %u}; slow xchg %u max %lld us\n",
                "07.4.42.2 P0-4", priv->hb_count, jiffies_to_msecs(jiffies - priv->hb_jiffies),
                priv->hb_last[1], priv->hb_last[2], priv->hb_last[3], priv->hb_last[4], priv->hb_last[5],
                priv->hb_last[6], priv->hb_last[7], priv->tx_xchg_slow, (long long)priv->tx_xchg_max_us);
}

/**
 */
static const struct net_device_ops x8h7_can_netdev_ops = {
  .ndo_open       = x8h7_can_open,
  .ndo_stop       = x8h7_can_stop,
  .ndo_start_xmit = x8h7_can_start_xmit,
  .ndo_tx_timeout = x8h7_can_tx_timeout,
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

  /* Phase 07.4.42.2 P0-4 — one echo skb per TX ring slot (slot index ==
   * echo index), so can_put_echo_skb(net, idx, 0) is valid for
   * idx 0..X8H7_CAN_TX_RING_SIZE-1. */
  net = alloc_candev(sizeof(struct x8h7_can_priv), X8H7_CAN_TX_RING_SIZE);
  if (!net) {
    return -ENOMEM;
  }

  net->netdev_ops = &x8h7_can_netdev_ops;
  /* ndo_tx_timeout deadline. dev_watchdog re-arms itself with round_jiffies(),
   * so the effective detection latency is 50 ms .. ~1 s. */
  net->watchdog_timeo = msecs_to_jiffies(X8H7_CAN_TX_WATCHDOG_MS);
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
