#include <string.h>

#include "nrf.h"
#include "nrf_gpio.h"
#include "nrfx.h"

#include "boards.h"
#include "dfu_types.h"
#include "bootloader.h"
#include "flash_nrf5x.h"
#include "dfu_tcp.h"

//--------------------------------------------------------------------+
// Pins: RAK13800 (W5100S) on the WisBlock IO slot, all on P0.
//--------------------------------------------------------------------+
#define ETH_PIN_SCK     3
#define ETH_PIN_MISO    29
#define ETH_PIN_MOSI    30
#define ETH_PIN_CS      26
#define ETH_PIN_RESET   21

#define ETH_SPIM        NRF_SPIM2

// Receive protocol
#define DFU_TCP_HDR_MAGIC      0x4D43544FUL   // "MCTO" (LE on the wire: 4F 54 43 4D)
#define DFU_TCP_MAX_IMAGE      (BOOTLOADER_REGION_START - DFU_BANK_0_REGION_START)
#define DFU_TCP_CONNECT_MS     90000
#define DFU_TCP_STALL_MS       15000

// Status byte sent back to the client
enum
{
  DFU_TCP_OK            = 0x00,
  DFU_TCP_ERR_MAGIC     = 0x01,
  DFU_TCP_ERR_TOO_BIG   = 0x02,
  DFU_TCP_ERR_CRC       = 0x03,
};

//--------------------------------------------------------------------+
// SPI (SPIM2, blocking, EasyDMA buffers in static RAM)
//--------------------------------------------------------------------+
static uint8_t _spi_tx[4];
static uint8_t _spi_rx[4];

static void spi_init(void)
{
  nrf_gpio_cfg_output(ETH_PIN_SCK);
  nrf_gpio_cfg_output(ETH_PIN_MOSI);
  nrf_gpio_cfg_input(ETH_PIN_MISO, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_output(ETH_PIN_CS);
  nrf_gpio_pin_set(ETH_PIN_CS);

  ETH_SPIM->PSEL.SCK  = ETH_PIN_SCK;
  ETH_SPIM->PSEL.MOSI = ETH_PIN_MOSI;
  ETH_SPIM->PSEL.MISO = ETH_PIN_MISO;
  ETH_SPIM->FREQUENCY = SPIM_FREQUENCY_FREQUENCY_M8;
  ETH_SPIM->CONFIG    = 0;   // mode 0, MSB first
  ETH_SPIM->ENABLE    = SPIM_ENABLE_ENABLE_Enabled;
}

// One 4-byte W5100S frame: [op, addr_hi, addr_lo, data]; returns 4th RX byte
static uint8_t spi_frame(uint8_t op, uint16_t addr, uint8_t data)
{
  _spi_tx[0] = op;
  _spi_tx[1] = (uint8_t) (addr >> 8);
  _spi_tx[2] = (uint8_t) addr;
  _spi_tx[3] = data;

  nrf_gpio_pin_clear(ETH_PIN_CS);

  ETH_SPIM->TXD.PTR    = (uint32_t) _spi_tx;
  ETH_SPIM->TXD.MAXCNT = 4;
  ETH_SPIM->RXD.PTR    = (uint32_t) _spi_rx;
  ETH_SPIM->RXD.MAXCNT = 4;
  ETH_SPIM->EVENTS_END = 0;
  ETH_SPIM->TASKS_START = 1;
  while ( !ETH_SPIM->EVENTS_END ) { }

  nrf_gpio_pin_set(ETH_PIN_CS);
  return _spi_rx[3];
}

//--------------------------------------------------------------------+
// W5100S register access
//--------------------------------------------------------------------+
static void    w5_wr (uint16_t addr, uint8_t v)  { spi_frame(0xF0, addr, v); }
static uint8_t w5_rd (uint16_t addr)             { return spi_frame(0x0F, addr, 0); }

static void w5_wr16(uint16_t addr, uint16_t v)
{
  w5_wr(addr, (uint8_t) (v >> 8));
  w5_wr(addr + 1, (uint8_t) v);
}

// volatile 16-bit counters: read until two consecutive reads agree
static uint16_t w5_rd16v(uint16_t addr)
{
  uint16_t v1, v2;
  v2 = ((uint16_t) w5_rd(addr) << 8) | w5_rd(addr + 1);
  do
  {
    v1 = v2;
    v2 = ((uint16_t) w5_rd(addr) << 8) | w5_rd(addr + 1);
  } while ( v1 != v2 );
  return v2;
}

// Common registers
#define W5_MR        0x0000
#define W5_GAR       0x0001
#define W5_SUBR      0x0005
#define W5_SHAR      0x0009
#define W5_SIPR      0x000F
#define W5_RMSR      0x001A
#define W5_TMSR      0x001B

// Socket 0 registers
#define S0_MR        0x0400
#define S0_CR        0x0401
#define S0_IR        0x0402
#define S0_SR        0x0403
#define S0_PORT      0x0404
#define S0_TX_FSR    0x0420
#define S0_TX_WR     0x0424
#define S0_RX_RSR    0x0426
#define S0_RX_RD     0x0428

// Socket 0 buffers (default 2KB each)
#define S0_TX_BASE   0x4000
#define S0_RX_BASE   0x6000
#define S0_BUF_MASK  0x07FF

// Commands / status
#define CMD_OPEN     0x01
#define CMD_LISTEN   0x02
#define CMD_DISCON   0x08
#define CMD_CLOSE    0x10
#define CMD_SEND     0x20
#define CMD_RECV     0x40

#define SOCK_INIT         0x13
#define SOCK_LISTEN       0x14
#define SOCK_ESTABLISHED  0x17
#define SOCK_CLOSE_WAIT   0x1C

static void s0_cmd(uint8_t cmd)
{
  w5_wr(S0_CR, cmd);
  while ( w5_rd(S0_CR) ) { }   // command register self-clears when accepted
}

//--------------------------------------------------------------------+
// Chip bring-up
//--------------------------------------------------------------------+

// Same locally-administered MAC derivation as the application's ethernet
// console — keeps the switch ARP/DHCP lease consistent across the handoff.
static void mac_from_ficr(uint8_t mac[6])
{
  uint32_t lo = NRF_FICR->DEVICEID[0];
  uint32_t hi = NRF_FICR->DEVICEID[1];
  mac[0] = 0x02;
  mac[1] = (uint8_t) (hi >> 8);
  mac[2] = (uint8_t) hi;
  mac[3] = (uint8_t) (lo >> 16);
  mac[4] = (uint8_t) (lo >> 8);
  mac[5] = (uint8_t) lo;
}

static bool w5_init(dfu_tcp_handoff_t const *cfg)
{
  nrf_gpio_cfg_output(ETH_PIN_RESET);
  nrf_gpio_pin_clear(ETH_PIN_RESET);
  NRFX_DELAY_MS(100);
  nrf_gpio_pin_set(ETH_PIN_RESET);
  NRFX_DELAY_MS(100);

  spi_init();

  w5_wr(W5_MR, 0x80);          // soft reset
  NRFX_DELAY_MS(10);

  uint8_t mac[6];
  mac_from_ficr(mac);
  for ( int i = 0; i < 6; i++ ) w5_wr(W5_SHAR + i, mac[i]);
  for ( int i = 0; i < 4; i++ ) w5_wr(W5_SIPR + i, cfg->ip[i]);
  for ( int i = 0; i < 4; i++ ) w5_wr(W5_GAR + i, cfg->gw[i]);
  for ( int i = 0; i < 4; i++ ) w5_wr(W5_SUBR + i, cfg->mask[i]);

  w5_wr(W5_RMSR, 0x55);        // 2KB rx/tx buffers per socket (default)
  w5_wr(W5_TMSR, 0x55);

  // sanity: read the MAC back to prove the chip is talking
  for ( int i = 0; i < 6; i++ )
  {
    if ( w5_rd(W5_SHAR + i) != mac[i] ) return false;
  }
  return true;
}

//--------------------------------------------------------------------+
// Socket helpers
//--------------------------------------------------------------------+
static bool s0_listen(uint16_t port)
{
  w5_wr(S0_MR, 0x01);          // TCP
  w5_wr16(S0_PORT, port);
  s0_cmd(CMD_OPEN);
  if ( w5_rd(S0_SR) != SOCK_INIT ) return false;
  s0_cmd(CMD_LISTEN);
  return w5_rd(S0_SR) == SOCK_LISTEN;
}

// Receive exactly len bytes; returns false on stall timeout or peer close
// with an empty buffer.
static bool s0_recv(uint8_t *buf, uint32_t len)
{
  uint32_t stall_ms = 0;

  while ( len )
  {
    uint16_t avail = w5_rd16v(S0_RX_RSR);
    if ( avail == 0 )
    {
      uint8_t sr = w5_rd(S0_SR);
      if ( sr != SOCK_ESTABLISHED && sr != SOCK_CLOSE_WAIT ) return false;
      if ( sr == SOCK_CLOSE_WAIT ) return false;   // peer closed, no more data
      if ( stall_ms++ > DFU_TCP_STALL_MS ) return false;
      NRFX_DELAY_MS(1);
      continue;
    }
    stall_ms = 0;

    uint16_t chunk = (avail < len) ? avail : (uint16_t) len;
    uint16_t rd = w5_rd16v(S0_RX_RD);
    for ( uint16_t i = 0; i < chunk; i++ )
    {
      *buf++ = w5_rd(S0_RX_BASE + ((rd + i) & S0_BUF_MASK));
    }
    w5_wr16(S0_RX_RD, rd + chunk);
    s0_cmd(CMD_RECV);
    len -= chunk;
  }
  return true;
}

static void s0_send_byte(uint8_t v)
{
  uint16_t wr = w5_rd16v(S0_TX_WR);
  w5_wr(S0_TX_BASE + (wr & S0_BUF_MASK), v);
  w5_wr16(S0_TX_WR, wr + 1);
  w5_wr(S0_IR, 0x10);          // clear SEND_OK
  s0_cmd(CMD_SEND);
  uint32_t guard = 0;
  while ( !(w5_rd(S0_IR) & 0x10) && guard++ < 2000 ) { NRFX_DELAY_MS(1); }
}

//--------------------------------------------------------------------+
// CRC32 (IEEE 802.3, matches zlib.crc32) — bitwise to save flash
//--------------------------------------------------------------------+
static uint32_t crc32_update(uint32_t crc, uint8_t const *data, uint32_t len)
{
  crc = ~crc;
  while ( len-- )
  {
    crc ^= *data++;
    for ( int k = 0; k < 8; k++ )
    {
      crc = (crc >> 1) ^ (0xEDB88320UL & (-(int32_t)(crc & 1)));
    }
  }
  return ~crc;
}

//--------------------------------------------------------------------+
// Main entry
//--------------------------------------------------------------------+
static uint8_t _chunk[1024];

void dfu_tcp_run(void)
{
  dfu_tcp_handoff_t const *cfg = (dfu_tcp_handoff_t const *) DFU_TCP_HANDOFF_ADDR;
  if ( cfg->magic != DFU_TCP_HANDOFF_MAGIC ) return;

  // invalidate the handoff so a crash loop can't re-enter TCP DFU forever
  ((dfu_tcp_handoff_t *) DFU_TCP_HANDOFF_ADDR)->magic = 0;

  if ( !w5_init(cfg) ) return;
  if ( !s0_listen(cfg->port ? cfg->port : 4444) ) return;

  led_state(STATE_USB_MOUNTED);   // distinct "ready" pattern

  // wait for a client
  uint32_t waited = 0;
  while ( w5_rd(S0_SR) != SOCK_ESTABLISHED )
  {
    if ( waited++ > DFU_TCP_CONNECT_MS ) return;
    NRFX_DELAY_MS(1);
  }

  // header: magic, image_len, crc32 (all LE)
  uint8_t hdr[12];
  if ( !s0_recv(hdr, sizeof(hdr)) ) return;

  uint32_t magic, image_len, crc_expected;
  memcpy(&magic, hdr, 4);
  memcpy(&image_len, hdr + 4, 4);
  memcpy(&crc_expected, hdr + 8, 4);

  if ( magic != DFU_TCP_HDR_MAGIC ) { s0_send_byte(DFU_TCP_ERR_MAGIC); return; }
  if ( image_len == 0 || image_len > DFU_TCP_MAX_IMAGE ) { s0_send_byte(DFU_TCP_ERR_TOO_BIG); return; }

  led_state(STATE_WRITING_STARTED);

  // stream image to flash (page-cached writer, erases as it goes)
  uint32_t off = 0;
  while ( off < image_len )
  {
    uint32_t n = image_len - off;
    if ( n > sizeof(_chunk) ) n = sizeof(_chunk);
    if ( !s0_recv(_chunk, n) ) return;   // partial write leaves app invalid -> DFU rescue
    flash_nrf5x_write(DFU_BANK_0_REGION_START + off, _chunk, (int) n, true);
    off += n;
  }
  flash_nrf5x_flush(true);

  led_state(STATE_WRITING_FINISHED);

  // verify what actually landed in flash
  uint32_t crc = crc32_update(0, (uint8_t const *) DFU_BANK_0_REGION_START, image_len);
  if ( crc != crc_expected )
  {
    s0_send_byte(DFU_TCP_ERR_CRC);
    return;
  }

  // mark bank 0 valid (same finalize as the UF2 path)
  dfu_update_status_t update_status;
  memset(&update_status, 0, sizeof(update_status));
  update_status.status_code = DFU_UPDATE_APP_COMPLETE;
  bootloader_dfu_update_process(update_status);

  s0_send_byte(DFU_TCP_OK);
  NRFX_DELAY_MS(100);          // let the status byte drain
  s0_cmd(CMD_DISCON);
  NRFX_DELAY_MS(100);

  NVIC_SystemReset();
}
