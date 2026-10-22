/**
 * X8H7 CAN driver
 */

#ifndef __X8H7_CAN_H
#define __X8H7_CAN_H

/**
 * INCLUDES
 */

#include <linux/timer.h>
#include <linux/hrtimer.h>
#include <linux/can/dev.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/atomic.h>
#include <linux/spinlock.h>

/**
 * DEFINES
 */
#define X8H7_CAN1_PERIPH    0x03
#define X8H7_CAN2_PERIPH    0x04

#define X8H7_CAN_OC_INIT    0x10
#define X8H7_CAN_OC_DEINIT  0x11
#define X8H7_CAN_OC_BITTIM  0x12
#define X8H7_CAN_OC_SEND    0x01
#define X8H7_CAN_OC_RECV    0x01
#define X8H7_CAN_OC_STS     0x40
#define X8H7_CAN_OC_FLT     0x50

#define X8H7_CAN_STS_INT_TX_COMPLETE       0x01
#define X8H7_CAN_STS_INT_RX                0x02
#define X8H7_CAN_STS_INT_ERR               0x04
#define X8H7_CAN_STS_INT_TX_ABORT_COMPLETE 0x08
#define X8H7_CAN_STS_INT_TX_FIFO_EMPTY     0x10

#define X8H7_CAN_STS_FLG_RX_OVR  0x01  // Receive Buffer Overflow
#define X8H7_CAN_STS_FLG_TX_BO   0x02  // Bus-Off
#define X8H7_CAN_STS_FLG_TX_EP   0x04  // Transmit Error-Passive
#define X8H7_CAN_STS_FLG_RX_EP   0x08  // Receive Error-Passive
#define X8H7_CAN_STS_FLG_TX_WAR  0x10  // Transmit Error Warning
#define X8H7_CAN_STS_FLG_RX_WAR  0x20  // Receive Error Warning
#define X8H7_CAN_STS_FLG_EWARN   0x40  // Error Warning
#define X8H7_CAN_STS_FLG_TX_OVR  0x80  // Transmit Buffer Overflow

#define X8H7_CAN_HEADER_SIZE        6
#define X8H7_CAN_FRAME_MAX_DATA_LEN 64

/* CAN-FD flags byte (byte 5 of x8h7_can_frame_message wire layout).
 * Must be kept in lockstep with X8H7_CAN_FLAG_* in can_handler.c
 * on the stm32h7-fw side (Plan 03 patch 0003). */
#define X8H7_CAN_FLAG_FD            0x01  /* FDFormat = FDCAN_FD_CAN */
#define X8H7_CAN_FLAG_BRS           0x02  /* BitRateSwitch = FDCAN_BRS_ON */
#define X8H7_CAN_FLAG_ESI           0x04  /* ErrorStateIndicator passive */

#define X8H7_STD_FLT_MAX  128
#define X8H7_EXT_FLT_MAX   64

#define X8H7_TX_FIFO_SIZE  32

/* Phase 7.4.42 Patch 0089 / Phase 07.4.42.2 P0-4 — TX batching.
 * start_xmit() appends to priv->tx_ring; the work handler drains up to
 * X8H7_CAN_BATCH_MAX frames per SPI exchange via x8h7_pkt_send_defer
 * (per-frame) + one final x8h7_pkt_send_now.
 *
 * X8H7_CAN_BATCH_MAX = max frames per SPI flush. Production SET_TARGETS
 * frames are 6 + 16 B + 4 B subpacket header = 26 B, so 8 frames = 208 B
 * <= the 252 B SPI payload (X8H7_BUF_SIZE 256 - 4 B packet header). Larger
 * frames are limited by X8H7_CAN_SPI_BUDGET below and/or hit x8h7_pkt_enq
 * -ENOMEM; the remainder is flushed in a following exchange. 8 equals the
 * FDCAN2 Tx FIFO half-depth after the 07.4.42.2 H7 repartition (Tx 16).
 *
 * BATCH_DEADLINE_NS bounds TX latency for partial batches at low rate.
 * 500 us = 10% of 5 ms (200 Hz wheel period); acceptable per control budget.
 */
#define X8H7_CAN_BATCH_MAX         6       /* 2026-10-22: = fleet size, so a full write pass flushes at once (was 8) */
#define X8H7_CAN_BATCH_DEADLINE_NS 100000  /* 100 us (was 500): on a cool board this took the production stale fraction from 0.17 to 0.0057 (gate 0.01) */

/* Phase 07.4.42.2 P0-4 — TX ring (head/tail/inflight, free-running u32
 * counters masked into the ring). RING_SIZE is also the echo-skb count
 * passed to alloc_candev(). */
#define X8H7_CAN_TX_RING_SIZE      16
#define X8H7_CAN_TX_RING_MASK      (X8H7_CAN_TX_RING_SIZE - 1)
#define X8H7_CAN_TX_WATCHDOG_MS    50      /* net->watchdog_timeo → ndo_tx_timeout */

/* Bytes one CAN frame occupies in spidev->x8h7_txb: x8h7_subpkt_t (4 B,
 * private to x8h7_drv.c) + the 6 B frame header + payload. The work handler
 * stops claiming frames once a batch would exceed X8H7_CAN_SPI_BUDGET
 * (= X8H7_PKT_SIZE, 248 B) so a normal flush never reaches the
 * x8h7_pkt_enq -ENOMEM path (which printk()s on the console). */
#define X8H7_CAN_SUBPKT_HDR_SIZE   4
#define X8H7_CAN_SPI_BUDGET        248

_Static_assert((X8H7_CAN_TX_RING_SIZE & X8H7_CAN_TX_RING_MASK) == 0,
               "ring size must be a power of two");
_Static_assert(X8H7_CAN_BATCH_MAX <= X8H7_CAN_TX_RING_SIZE,
               "batch must fit the ring");

/**
 * TYPEDEFS
 */
union x8h7_can_init_message
{
  struct __attribute__((packed))
  {
    uint32_t baud_rate_prescaler;
    uint32_t time_segment_1;
    uint32_t time_segment_2;
    uint32_t sync_jump_width;
    uint32_t data_baud_rate_prescaler;
    uint32_t data_time_segment_1;
    uint32_t data_time_segment_2;
    uint32_t data_sync_jump_width;
  } field;
  uint8_t buf[8 * sizeof(uint32_t)];  /* 32 bytes (was 16) */
};
_Static_assert(sizeof(union x8h7_can_init_message) == 32,
               "x8h7 CAN init wire layout drift (must match stm32h7-fw side)");

union x8h7_can_bittiming_message
{
  struct __attribute__((packed))
  {
    uint32_t baud_rate_prescaler;
    uint32_t time_segment_1;
    uint32_t time_segment_2;
    uint32_t sync_jump_width;
  } field;
  uint8_t buf[sizeof(uint32_t) /* can_bitrate_Hz */ + sizeof(uint32_t) /* time_segment_1 */ + sizeof(uint32_t) /* time_segment_2 */ + sizeof(uint32_t) /* sync_jump_width */];
};

union x8h7_can_filter_message
{
  struct __attribute__((packed))
  {
    uint32_t idx;
    uint32_t id;
    uint32_t mask;
  } field;
  uint8_t buf[sizeof(uint32_t) /* idx */ + sizeof(uint32_t) /* id */ + sizeof(uint32_t) /* mask */];
};

union x8h7_can_frame_message
{
  struct __attribute__((packed))
  {
    uint32_t id;
    /* WIRE FORMAT INVARIANT (Phase 06.5 — paired with stm32h7-fw can_handler.c):
     * `len` is the BYTE COUNT (0..64). NOT an FDCAN DLC code.
     * Kernel side: copies cfd->len into field.len after WARN_ON +
     *   clamp to X8H7_CAN_FRAME_MAX_DATA_LEN (64). See x8h7_can.c:91-93.
     * H7 side: can_write() switches on len at can.c:316-335. Lengths
     *   not in {0..8, 12, 16, 20, 24, 32, 48, 64} fall through `default:`
     *   to FDCAN_DLC_BYTES_0 (frame transmits with DLC=0). Per ISO 11898-1
     *   §10.4.1 these are the only valid CAN-FD byte counts; non-standard
     *   lengths SHOULD never appear post-kernel-clamp.
     * Wire format = byte count throughout. DLC-code mapping lives ONLY
     *   inside can_write()'s switch (H7 side). */
    uint8_t  len;
    uint8_t  flags;  /* X8H7_CAN_FLAG_FD | X8H7_CAN_FLAG_BRS | X8H7_CAN_FLAG_ESI */
    uint8_t  data[X8H7_CAN_FRAME_MAX_DATA_LEN];
  } field;
  uint8_t buf[X8H7_CAN_HEADER_SIZE + X8H7_CAN_FRAME_MAX_DATA_LEN];
};
_Static_assert(sizeof(union x8h7_can_frame_message) == 70,
               "x8h7 CAN FD frame wire layout drift (must match stm32h7-fw side)");
_Static_assert(X8H7_CAN_FRAME_MAX_DATA_LEN == 64,
               "Phase 06.5 wire-format invariant: ISO 11898-1 max FD payload is 64 bytes; "
               "change requires bilateral wire-format audit (kernel x8h7_can.h + H7 can_handler.c)");

struct x8h7_can_priv {
  struct can_priv           can;
  struct net_device        *net;
  struct device            *dev;
  int                       periph;

  /* 07.4.42.2 bench 2026-10-22: the TX path runs on a DEDICATED kthread_worker (named
   * "x8h7cantx-<if>", SCHED_FIFO 50 by default, raised to io-thread+1 by x8-rt-promote.sh),
   * not a per-CPU CFS workqueue: a SCHED_FIFO userspace writer on the same CPU starved the
   * kworker and the drain fell from 3000 to ~940 requests/s (raw sender at FIFO 90: 9000 -> ~500). */
  struct kthread_worker       *wq;
  struct kthread_work          work;

  /* Phase 7.4.42.1 Patch 0090 (T3) -- bus-off restart work item.
   * Deferred from the can_dev timer context (x8h7_can_restart) onto priv->wq
   * so x8h7_can_hw_setup (which calls x8h7_pkt_send_sync; may sleep) runs
   * off the timer/softirq stack. Initialized in x8h7_can_open alongside work. */
  struct kthread_work          restart_work;

  /* Phase 7.4.42.1 Patch 0090 (T4) -- TEC/REC berr cache.
   * Written by x8h7_can_status() from 4-byte H7 status payload (Patch 0091 T4).
   * Read by do_get_berr_counter() to populate `ip -d link show can1` berr-counter.
   * Updated on error-state-change only (not per-frame -- Pitfall 6).
   * ASVS V5: only written when pkt->size >= 4 (length-guard against short frames). */
  u8 last_tec;
  u8 last_rec;

  union x8h7_can_frame_message tx_frame;  /* legacy single-slot; preserved for
                                           * non-batched paths (status frame,
                                           * etc.) — Patch 0089 adds batch ring
                                           * below for start_xmit hot path */

  /* Phase 07.4.42.2 P0-4 — TX ring (replaces Patch 0089's single
   * outstanding-frame counter, which duplicated, dropped and wedged frames).
   * Free-running u32 counters, all protected by ring_lock (spin_lock_bh —
   * there is no hard-IRQ user), compared with unsigned subtraction.
   * Invariant: tx_inflight <= tx_head <= tx_tail and
   *            tx_tail - tx_inflight <= X8H7_CAN_TX_RING_SIZE.
   * Slot i (and echo skb i) = counter & X8H7_CAN_TX_RING_MASK. ring_lock is
   * never held while calling x8h7_pkt_send_* (spidev mutex, may sleep).
   * batch_timer schedules a deferred flush for partial batches at low rate.
   */
  union x8h7_can_frame_message tx_ring[X8H7_CAN_TX_RING_SIZE];
  u32                          tx_tail;      /* next free slot (producer: start_xmit) */
  u32                          tx_head;      /* next slot to send (work handler) */
  u32                          tx_inflight;  /* oldest sent-but-unretired slot */
  u32                          tx_gen;       /* bumped on every ring reset */
  spinlock_t                   ring_lock;
  struct hrtimer               batch_timer;

  struct can_filter         std_flt[X8H7_STD_FLT_MAX];
  struct can_filter         ext_flt[X8H7_EXT_FLT_MAX];

  struct mutex              lock;
};

#endif
