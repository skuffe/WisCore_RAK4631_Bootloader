/**
 * TCP DFU over a WIZnet W5100S (RAK13800 module on a RAK19007 base board).
 *
 * Entered when the application writes DFU_MAGIC_TCP_RESET to GPREGRET and
 * resets. The application passes its live network configuration through a
 * noinit RAM struct (same retained-RAM trick as the double-reset detection
 * at DFU_DBL_RESET_MEM); the bootloader applies it statically — no DHCP.
 *
 * The W5100S terminates TCP in hardware, so this needs only an SPI register
 * driver, no IP stack.
 */
#ifndef DFU_TCP_H__
#define DFU_TCP_H__

#include <stdint.h>

#define DFU_MAGIC_TCP_RESET       0x7E

// Retained-RAM handoff block, written by the application immediately before
// reset. Sits just below DFU_DBL_RESET_MEM (0x20007F7C).
#define DFU_TCP_HANDOFF_ADDR      0x20007F60UL
#define DFU_TCP_HANDOFF_MAGIC     0x54435041UL  // "TCPA"

typedef struct
{
  uint32_t magic;       // DFU_TCP_HANDOFF_MAGIC
  uint8_t  ip[4];       // static IP (the app's DHCP lease)
  uint8_t  gw[4];
  uint8_t  mask[4];
  uint16_t port;        // TCP listen port
  uint16_t reserved;
} dfu_tcp_handoff_t;

/**
 * Run the TCP DFU receiver. On a successful image transfer this function
 * finalizes the app bank and resets the MCU (does not return). On timeout
 * or failure it returns, and the caller falls through to the normal boot
 * logic (valid app boots; invalid app lands in OTA DFU rescue).
 */
void dfu_tcp_run(void);

#endif
