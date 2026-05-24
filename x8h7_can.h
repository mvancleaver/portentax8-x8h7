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

/* Phase 7.4.42 Patch 0089 — TX batching for sustained 6-motor production load.
 * Multiple start_xmit() invocations accumulate into priv->tx_ring; the work
 * handler drains the ring via x8h7_pkt_send_defer (per-frame) + one final
 * x8h7_pkt_send_now (single SPI exchange carries N CAN frames).
 *
 * BATCH_MAX must be <= X8H7_TX_FIFO_SIZE (32) and must fit alongside other
 * peripherals' subpkts in spidev->x8h7_txb (X8H7_BUF_SIZE = 256). With
 * X8H7_CAN_HEADER_SIZE + X8H7_CAN_FRAME_MAX_DATA_LEN = 70 + ~4-byte subpkt
 * header per frame, 6 frames worst-case = 444 bytes which exceeds 256.
 * In practice motor SET_TARGETS frames are 8-12 bytes payload, so 6 fit
 * comfortably in the buffer (~120-150 bytes). For larger payloads the H7
 * x8h7_pkt_enq returns -ENOMEM and we flush early.
 *
 * BATCH_DEADLINE_NS bounds TX latency for partial batches at low rate.
 * 500 us = 10% of 5 ms (200 Hz wheel period); acceptable per control budget.
 */
#define X8H7_CAN_BATCH_MAX         6
#define X8H7_CAN_BATCH_DEADLINE_NS 500000  /* 500 us */

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

  int                       tx_len;

  struct workqueue_struct     *wq;
  struct work_struct           work;
  union x8h7_can_frame_message tx_frame;  /* legacy single-slot; preserved for
                                           * non-batched paths (status frame,
                                           * etc.) — Patch 0089 adds batch ring
                                           * below for start_xmit hot path */

  /* Phase 7.4.42 Patch 0089 — TX batching state.
   * tx_ring stores N pending TX frames; batch_outstanding counts how many
   * slots are populated. ring_lock protects both (spinlock; held briefly in
   * start_xmit + work_handler + status TX_COMPLETE handler; never held while
   * calling x8h7_pkt_send_defer/now which take the spidev mutex).
   * batch_timer schedules a deferred flush for partial batches at low rate.
   */
  union x8h7_can_frame_message tx_ring[X8H7_CAN_BATCH_MAX];
  atomic_t                     batch_outstanding;
  spinlock_t                   ring_lock;
  struct hrtimer               batch_timer;

  struct can_filter         std_flt[X8H7_STD_FLT_MAX];
  struct can_filter         ext_flt[X8H7_EXT_FLT_MAX];

  struct mutex              lock;
};

#endif
