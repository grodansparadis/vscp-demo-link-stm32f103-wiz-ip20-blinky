/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <string.h>
#include <blinky.h>
#include <flash_storage.h>
#include <vscp-fifo.h>
#include <vscp-firmware-helper.h>
#include <vscp-firmware-level2.h>
#include <vscp-link-protocol.h>
#include <vscp-binary-protocol.h>
#include <vscp-link-protocol-callbacks.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
  VSCP_STATE_DISCONNECTED = 0, /**< Waiting for a client to connect    */
  VSCP_STATE_CONNECTED,        /**< Client connected, processing cmds  */
} vscp_state_t;

typedef enum {
  VSCP_SUBSTATE_POLL = 0, /**< Polling state */
  VSCP_SUBSTATE_AUTO,     /**< RETR mode */
} vscp_substate_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ** Check so one and only one build type is enabled **

/* Convert each define to 0 or 1 */
#ifdef BLINKY_MODE_TCP_SERVER
#define _MODE_TCP_SERVER 1
#else
#define _MODE_TCP_SERVER 0
#endif

#ifdef BLINKY_DEMO_MODE_TCP_CLIENT
#define _OPT_MODE_TCP_CLIENT 1
#else
#define _OPT_MODE_TCP_CLIENT 0
#endif

#ifdef BLINKY_DEMO_MODE_UDP
#define _OPT_MODE_UDP 1
#else
#define _OPT_MODE_UDP 0
#endif

#ifdef BLINKY_DEMO_MODE_MULTICAST
#define _OPT_MODE_MULTICAST 1
#else
#define _OPT_MODE_MULTICAST 0
#endif

#ifdef BLINKY_DEMO_MODE_MQTT
#define _OPT_MODE_MQTT 1
#else
#define _OPT_MODE_MQTT 0
#endif

#define _OPT_COUNT (_MODE_TCP_SERVER + _OPT_MODE_TCP_CLIENT + _OPT_MODE_UDP + _OPT_MODE_MULTICAST + _OPT_MODE_MQTT)

#if _OPT_COUNT > 1
#error                                                                                                                 \
  "Only one of BLINKY_MODE_TCP_SERVER/BLINKY_DEMO_MODE_TCP_CLIENT/BLINKY_DEMO_MODE_UDP/BLINKY_DEMO_MODE_MULTICAST/BLINKY_DEMO_MODE_MQTT may be defined at a time"
#endif

#if _OPT_COUNT != 1
#error                                                                                                                 \
  "Exactly one of BLINKY_MODE_TCP_SERVER/BLINKY_DEMO_MODE_TCP_CLIENT/BLINKY_DEMO_MODE_UDP/BLINKY_DEMO_MODE_MULTICAST/BLINKY_DEMO_MODE_MQTT must be defined"
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Ring buffer for IRQ-driven UART1 RX ------------------------------------ */
#define UART1_RX_BUF_SIZE 2048u /* must be a power of 2 */
static uint8_t uart1_rx_buf[UART1_RX_BUF_SIZE];
static volatile uint16_t uart1_rx_head = 0; /* written by ISR/callback     */
static volatile uint16_t uart1_rx_tail = 0; /* read  by main loop          */
static uint8_t uart1_rx_byte;               /* single-byte DMA target      */

//----------------------------------------------------------------------------
//                         VSCP Link Protocol
//----------------------------------------------------------------------------

/*
  Connect callbacks in op table. This table is passed to the VSCP link protocol handler
  and defines the behavior for various operations.
*/
static const vscp_link_ops_t link_ops = {
  /* ── Transport ──────────────────────────────────────────────── */
  .write_client = vscp_link_callback_write_client,
  .disconnect   = vscp_link_callback_disconnect,
  .quit         = vscp_link_callback_quit,
  .welcome      = vscp_link_callback_welcome,

  /* ── Authentication ─────────────────────────────────────────── */
  .check_user          = vscp_link_callback_check_user,
  .check_password      = vscp_link_callback_check_password,
  .challenge           = vscp_link_callback_challenge,
  .check_authenticated = vscp_link_callback_check_authenticated,
  .check_privilege     = vscp_link_callback_check_privilege,

  /* ── Event I/O ──────────────────────────────────────────────── */
  .send               = vscp_link_callback_send,
  .chkdata            = vscp_link_callback_chkData,
  .clrall             = vscp_link_callback_clrall,
  .retr               = vscp_link_callback_retr,
  .rcvloop            = vscp_link_callback_rcvloop,
  .enable_rcvloop     = vscp_link_callback_enable_rcvloop,
  .get_rcvloop_status = vscp_link_callback_get_rcvloop_status,

  /* ── Node information ───────────────────────────────────────── */
  .get_guid       = vscp_link_callback_get_guid,
  .set_guid       = vscp_link_callback_set_guid,
  .get_version    = vscp_link_callback_get_version,
  .get_channel_id = vscp_link_callback_get_channel_id,
  .wcyd           = vscp_link_callback_wcyd,
  .statistics     = vscp_link_callback_statistics,
  .info           = vscp_link_callback_info,

  /* ── Filter / buffer ────────────────────────────────────────── */
  .set_filter = vscp_link_callback_setFilter,
  .set_mask   = vscp_link_callback_setMask,

  /* ── Interfaces ─────────────────────────────────────────────── */
  .get_interface_count = vscp_link_callback_get_interface_count,
  .get_interface       = vscp_link_callback_get_interface,
  .close_interface     = vscp_link_callback_close_interface,

  /* ── Misc ────────────────────────────────────────────────────── */
  .help_custom = NULL, // OR vscp_link_callback_help,
  .test        = vscp_link_callback_test,
  .shutdown    = vscp_link_callback_shutdown,
  .restart     = vscp_link_callback_restart,

  /* ── Binary mode ─────────────────────────────────────────────── */
  .binary = NULL // vscp_link_callback_binary,
};

/* Context for each possible client connection  */
static vscp_link_ctx_t ctx_link = {
  .next              = NULL,
  .ops               = &link_ops,
  .id                = 0,
  .sock              = 0,
  .guid              = { 0 },
  .user              = { 0 },
  .fifoEventsIn      = { 0 },
  .fifoEventsOut     = { 0 },
  .bValidated        = 0,
  .bRcvLoop          = 0,
  .privLevel         = 0,
  .filter            = { 0 },
  .statistics        = { 0 },
  .status            = { 0 },
  .last_rcvloop_time = 0,
  .user_data         = NULL,
};

//----------------------------------------------------------------------------
//                         VSCP Binary Protocol
//----------------------------------------------------------------------------

/* Binary context for each possible client connection */
static vscp_binary_ctx_t ctx_binary = { 0 };

//----------------------------------------------------------------------------
//                         VSCP Protocol
//----------------------------------------------------------------------------

/* Context for VSCP Level II protocol stack */
static vscp_frmw2_firmware_context_t ctx_firmware = {
  .level    = VSCP_LEVEL2,
  .state    = FRMW2_STATE_NONE,
  .substate = 0,
};

node_persistent_config_t g_persistent;

volatile uint32_t msTicks = 0U; /* incremented every 1 ms by TIM2 ISR */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void
SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void
uart1_rx_start(void);
static int
uart1_rx_getline(char *out, size_t max_len, uint16_t timeout_ms);
static int
uart1_rx_wait_for(const char *needle, uint16_t timeout_ms);
static int
uart1_rx_scan(const char *needle);
static void
uart1_rx_consume_through(const char *needle);
void
setContextDefaults(ctx_t *pctx);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <stdio.h>
#include "usart.h"

//////////////////////////////////////////////////////////////////////////
// Retargeting printf to USART2 for debug output.
// This allows us to use LOGSTR() to send data over USART2,
// which can be useful for debugging purposes. The __io_putchar
// function is called by the LOGSTR() function to output each character,
// and it uses the HAL_UART_Transmit function to send the character over
// USART2. Make sure that USART2 is properly initialized and configured in
// your project for this to work correctly.
//

/*!
 * @brief  Retargets the C library printf function to the USART.
 * @param  ch: Character to be transmitted.
 * @retval The character that was transmitted.
 */

int
__io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t *) &ch, 1, HAL_MAX_DELAY);
  return ch;
}

/*!
 * @brief  Retargets the C library printf function to the USART.
 * @param  file: File descriptor (not used).
 * @param  ptr: Pointer to the data to be written.
 * @param  len: Length of the data to be written.
 * @retval Number of bytes written.
 */

int
_write(int file, char *ptr, int len)
{
  for (int i = 0; i < len; i++) {
    __io_putchar(*ptr++);
  }
  return len;
}

/*!
 * @brief  Get UART response with timeout (blocking, used during AT init only).
 * @param  buf: Buffer to store the received data.
 * @param  buf_size: Size of the buffer.
 * @param  timeout_ms: Timeout in milliseconds.
 * @retval Number of bytes received, or 0 on timeout.
 */

static size_t
getWizIp20Response(char *buf, size_t buf_size, uint32_t timeout_ms)
{
  memset(buf, 0, buf_size);
  uint16_t rx_len = 0;
  HAL_UARTEx_ReceiveToIdle(&huart1, (uint8_t *) buf, (uint16_t) (buf_size - 1), &rx_len, timeout_ms);
  if (rx_len > 0) {
    buf[rx_len] = '\0';
    LOGSTR("IP20 response: %s\r\n", buf);
  }
  return rx_len;
}

/*!
 * @brief  Re-arm UART1 IRQ for one byte.  Call once at startup and again
 *         from HAL_UART_RxCpltCallback.
 */

static void
uart1_rx_start(void)
{
  HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
}

/*!
 * @brief  HAL callback – fires after each byte received via UART1 IT.
 *         Stores the byte in the ring buffer and re-arms the IRQ.
 */

void
HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    uint16_t next = (uart1_rx_head + 1u) & (UART1_RX_BUF_SIZE - 1u);
    if (next != uart1_rx_tail) { /* drop byte if buffer full */
      uart1_rx_buf[uart1_rx_head] = uart1_rx_byte;
      uart1_rx_head               = next;
    }
    uart1_rx_start(); /* re-arm for next byte     */
  }
}

/*!
 * @brief  Fetch a \r\n-terminated string from the UART1 ring buffer.
 *
 * The buffer is only consumed when a complete \r\n line is available,
 * so partial data is never discarded on timeout.
 *
 * @param  out        Destination buffer; null-terminated on success.
 * @param  max_len    Size of @p out in bytes (including the null terminator).
 * @param  timeout_ms Wait behaviour:
 *                      0x0000 – return immediately if no complete line yet
 *                      0x0001…0xFFFE – wait up to N milliseconds
 *                      0xFFFF – wait forever
 * @retval  0   A complete \r\n-terminated line was copied into @p out.
 * @retval -1   No complete line available within the specified timeout.
 */

static int
uart1_rx_getline(char *out, size_t max_len, uint16_t timeout_ms)
{
  /* Scan the ring buffer for \r\n without consuming any bytes yet. */
  for (;;) {
    /* --- peek: is there a \r\n somewhere in the buffer? --- */
    uint16_t head  = uart1_rx_head; /* snapshot (volatile, read once) */
    uint16_t idx   = uart1_rx_tail;
    uint8_t found  = 0;
    uint16_t eol   = 0; /* index of '\n' in ring buffer   */
    uint16_t count = 0; /* bytes up to and including '\n' */

    while (idx != head) {
      uint8_t ch = uart1_rx_buf[idx];
      count++;
      if (ch == '\n' && count >= 2 && uart1_rx_buf[(idx - 1u) & (UART1_RX_BUF_SIZE - 1u)] == '\r') {
        found = 1;
        eol   = idx;
        break;
      }
      idx = (idx + 1u) & (UART1_RX_BUF_SIZE - 1u);
    }

    if (found) {
      /* Consume bytes up to and including the '\n'. */
      size_t pos = 0;
      idx        = uart1_rx_tail;
      while (1) {
        uint8_t ch = uart1_rx_buf[idx];
        if (pos < max_len - 1u) {
          out[pos++] = (char) ch;
        }
        idx           = (idx + 1u) & (UART1_RX_BUF_SIZE - 1u);
        uart1_rx_tail = idx;
        if (idx == (uint16_t) ((eol + 1u) & (UART1_RX_BUF_SIZE - 1u)))
          break;
      }
      out[pos] = '\0';
      return 0;
    }

    /* No complete line yet – handle timeout. */
    if (timeout_ms == 0u) {
      return -1; /* no-wait: fail immediately      */
    }
    if (timeout_ms == 0xFFFFu) {
      continue; /* wait forever: keep polling     */
    }

    /* Finite timeout: spin until deadline or line arrives. */
    uint32_t deadline = HAL_GetTick() + timeout_ms;
    while (HAL_GetTick() < deadline) {
      /* re-check ring buffer */
      head  = uart1_rx_head;
      idx   = uart1_rx_tail;
      found = 0;
      count = 0;
      while (idx != head) {
        uint8_t ch = uart1_rx_buf[idx];
        count++;
        if (ch == '\n' && count >= 2 && uart1_rx_buf[(idx - 1u) & (UART1_RX_BUF_SIZE - 1u)] == '\r') {
          found = 1;
          eol   = idx;
          break;
        }
        idx = (idx + 1u) & (UART1_RX_BUF_SIZE - 1u);
      }
      if (found)
        break;
    }
    if (!found)
      return -1; /* timed out                      */
    /* line arrived during finite wait – loop back to consume it        */
    timeout_ms = 0xFFFFu; /* force consume path next iter   */
  }
}

/*!
 * @brief  Wait until @p needle appears anywhere in a received line.
 *
 * Reads complete \r\n-terminated lines from the ring buffer one at a time
 * and checks whether @p needle is a substring of each line.
 * Lines that do not match are discarded.
 *
 * @param  needle      String to search for (case-sensitive substring match).
 * @param  timeout_ms  Same semantics as uart1_rx_getline:
 *                       0x0000 – return immediately if no matching line queued
 *                       0x0001–0xFFFE – wait up to N ms
 *                       0xFFFF – wait forever
 * @retval  0   A line containing @p needle was found.
 * @retval -1   Timeout elapsed without a matching line.
 */

static int __attribute__((unused))
uart1_rx_wait_for(const char *needle, uint16_t timeout_ms)
{
  static char line[100];
  uint32_t deadline = (timeout_ms != 0u && timeout_ms != 0xFFFFu) ? HAL_GetTick() + timeout_ms : 0u;

  for (;;) {
    /* How long to wait for the next line? */
    uint16_t line_timeout;
    if (timeout_ms == 0u) {
      line_timeout = 0u; /* no-wait              */
    }
    else if (timeout_ms == 0xFFFFu) {
      line_timeout = 0xFFFFu; /* wait forever         */
    }
    else {
      uint32_t now = HAL_GetTick();
      if (now >= deadline)
        return -1; /* overall timeout      */
      uint32_t left = deadline - now;
      line_timeout  = (left > 0xFFFEu) ? 0xFFFEu : (uint16_t) left;
    }

    if (uart1_rx_getline(line, sizeof(line), line_timeout) != 0) {
      return -1; /* no line / timed out  */
    }

    if (strstr(line, needle) != NULL) {
      return 0; /* match found          */
    }
    /* line didn't match – discard and try again (unless no-wait) */
    if (timeout_ms == 0u)
      return -1;
  }
}

/*!
 * @brief  Scan the ring buffer for @p needle without consuming any bytes.
 * @retval 0  Needle found in the buffer.
 * @retval -1 Needle not present.
 */

static int
uart1_rx_scan(const char *needle)
{
  size_t nlen   = strlen(needle);
  uint16_t head = uart1_rx_head; /* snapshot */
  uint16_t idx  = uart1_rx_tail;
  size_t mpos   = 0; /* match position within needle */

  while (idx != head) {
    if ((char) uart1_rx_buf[idx] == needle[mpos]) {
      mpos++;
      if (mpos == nlen)
        return 0; /* full match */
    }
    else {
      mpos = 0;
      /* re-check current char against needle[0] */
      if ((char) uart1_rx_buf[idx] == needle[0])
        mpos = 1;
    }
    idx = (idx + 1u) & (UART1_RX_BUF_SIZE - 1u);
  }
  return -1;
}

/*!
 * @brief  Consume bytes from the ring buffer up to and including the first occurrence of @p needle.
 *
 * This function scans the ring buffer for the first occurrence of @p needle (a string), and if found, advances the
 * tail pointer to consume all bytes up to and including that occurrence. If @p needle is not found, the buffer is
 * left unchanged.
 *
 * @param  needle  String to search for (case-sensitive substring match).
 */
static void
uart1_rx_consume_through(const char *needle)
{
  size_t nlen   = strlen(needle);
  uint16_t head = uart1_rx_head; /* snapshot */
  uint16_t idx  = uart1_rx_tail;
  size_t mpos   = 0;

  while (idx != head) {
    if ((char) uart1_rx_buf[idx] == needle[mpos]) {
      mpos++;
      idx = (idx + 1u) & (UART1_RX_BUF_SIZE - 1u);
      if (mpos == nlen) {
        uart1_rx_tail = idx; /* consume up to end of needle */
        return;
      }
    }
    else {
      mpos = 0;
      if ((char) uart1_rx_buf[idx] == needle[0]) {
        mpos = 1;
      }
      idx = (idx + 1u) & (UART1_RX_BUF_SIZE - 1u);
    }
  }
}

/*!
 * @brief  Get the unique identifier and set (GUID) for the device.
 *
 * VSCP reserves a group of GUID's for MCU's with a built-in unique ID,
 * where the GUID is constructed from a fixed prefix.
 *
 * One other possibility was to use the WIZnet's MAC address and the
 * reserved GUID prefix to construct a GUID built here we use the unique
 * MCU ID instead since it's guaranteed to be unique and doesn't require
 * an extra step to read the MAC address from the WIZnet.
 *
 * More info on VSCP GUID construction for MCU-based devices here:
 * https://grodansparadis.github.io/vscp-doc-spec/#/./vscp_globally_unique_identifiers
 *
 *This GUID is constructed like this
 *
 * FD:AA:BB:YY:YY:YY:YY:YY:YY:YY:YY:YY:YY:YY:YY:YY
 *
 * AA is a manufacturer code assigned by VSCP (02 for STMicroelectronics)
 *
 * BB is a device type code assigned by VSCP (01 for STM32)
 *
 * The 13 byte YY:YY:YY:YY:YY:YY:YY:YY:YY:YY:YY:YY:YY is something we can
 * define freely as long as it's unique for each device. We can get 12 byte
 * from the unique MCU ID using the HAL_GetUIDw0(), HAL_GetUIDw1(),
 * and HAL_GetUIDw2() functions. Then we can set the last byte to zero.
 *
 * @param  pguid: Pointer to a 16-byte array where the GUID will be stored.
 */

void
setGUID(uint8_t *pguid)
{
  uint32_t uid;

  memset(pguid, 0, 16); /* Clear the GUID buffer */
  pguid[0] = 0xFD;      /* VSCP reserved prefix for MCU-based GUIDs */
  pguid[1] = 0x02;      /* Manufacturer code for STMicroelectronics */
  pguid[2] = 0x01;      /* Device type code for STM32 */

  uid      = HAL_GetUIDw2(); // Words 2 (bits 95:64)
  pguid[3] = (uid >> 24) & 0xFF;
  pguid[4] = (uid >> 16) & 0xFF;
  pguid[5] = (uid >> 8) & 0xFF;
  pguid[6] = uid & 0xFF;

  uid       = HAL_GetUIDw0(); // Words 0 (bits 31:0)
  pguid[7]  = (uid >> 24) & 0xFF;
  pguid[8]  = (uid >> 16) & 0xFF;
  pguid[9]  = (uid >> 8) & 0xFF;
  pguid[10] = uid & 0xFF;

  uid       = HAL_GetUIDw1(); // Words 1 (bits 63:32)
  pguid[11] = (uid >> 24) & 0xFF;
  pguid[12] = (uid >> 16) & 0xFF;
  pguid[13] = (uid >> 8) & 0xFF;
  pguid[14] = uid & 0xFF;

  pguid[15] = 0; // Last byte set to zero
}

/*!
 * @brief  Initialize the VSCP link context with default values.
 * @param  pctx: Pointer to the context structure to initialize.
 *
 * As we only have one channel we can get away with a single global context
 * object, but we define this function to set defaults for it and to reset
 * it between connections if needed.
 */

void
setContextDefaults(ctx_t *pctx)
{
  memset(pctx, 0, sizeof(ctx_t));
  pctx->id   = 0;
  pctx->sock = 0;
  memset(pctx->user, 0, VSCP_LINK_MAX_USER_NAME_LENGTH);
  setGUID(pctx->guid);
  vscp_fifo_init(&pctx->fifoEventsOut, VSCP_LINK_MAX_OUT_FIFO_SIZE);
  vscp_fifo_init(&pctx->fifoEventsIn, VSCP_LINK_MAX_IN_FIFO_SIZE);
  pctx->bValidated = 0; // No credentials yet
  pctx->privLevel  = 0; // No privileges before we are logged in
  pctx->bRcvLoop   = 0; // Polling mode by default, can switch to RETR after login if desired
  memset(&pctx->filter, 0, sizeof(vscpEventFilter));       // All events is received by client
  memset(&pctx->statistics, 0, sizeof(vscp_statistics_t)); // VSCP Statistics
  memset(&pctx->status, 0, sizeof(vscp_status_t));         // VSCP status
  pctx->last_rcvloop_time = 0;
}

/*!
 * @brief  Reset the VSCP link context to default values for a new connection.
 * @param  pctx: Pointer to the context structure to reset.
 *
 * This function is similar to setContextDefaults but is intended to be called
 * when resetting the context for a new client connection, without changing
 * fields that should persist across connections (like the GUID).
 *
 * Note: In this simple example we only support one client at a time, so we
 * don't actually need to preserve any fields across connections, but in a more
 * complex implementation with multiple clients you might want to keep certain
 * fields unchanged when resetting for a new connection.
 */

void
resetContextDefaults(ctx_t *pctx)
{
  pctx->bValidated = 0; // No credentials yet
  pctx->privLevel  = 0; // No privileges before we are logged in
  pctx->bRcvLoop   = 0; // Polling mode by default, can switch to RETR after login if desired
  memset(&pctx->filter, 0, sizeof(vscpEventFilter));       // All events is received by client
  memset(&pctx->statistics, 0, sizeof(vscp_statistics_t)); // VSCP Statistics
  memset(&pctx->status, 0, sizeof(vscp_status_t));         // VSCP status
  pctx->last_rcvloop_time = 0;

  // Drain the fifos
  vscp_fifo_clear(&pctx->fifoEventsOut);
  vscp_fifo_clear(&pctx->fifoEventsIn);
}

/*!
 * @brief  Enter WIZnet IP20 command mode and wait for the "CMD" response.
 *
 * The WIZnet IP20 module uses an AT command interface for configuration. To send
 * AT commands, we first need to enter command mode by sending "+++" with a guard
 * time before and after. After sending "+++", we wait for the "CMD" response to
 * confirm that we are in command mode before proceeding with further configuration.
 *
 * @retval 0 if successfully entered command mode, -1 on timeout or error.
 */
int
wiznet_ip20_command_mode(void)
{
  size_t rx_len;

  char buf[80];

  LOGSTR("WIZnet IP20 enter command mode:      \r\n");

  // Enter command mode: guard time, "+++", guard time
  HAL_Delay(500); // guard time before
  HAL_UART_Transmit(&huart1, (uint8_t *) "+++", 3, HAL_MAX_DELAY);
  HAL_Delay(500); // guard time after
  // rx_len = getWizIp20Response(buf, sizeof(buf), 1000);
  if (0 == uart1_rx_getline(buf, sizeof(buf), 1000)) {
    LOGSTR("Received response after +++: %s\r\n", buf);
    if (strstr(buf, "SEG:AT Mode") == 0) {
      LOGSTR("Entered command mode successfully %s\r\n", buf);
      return 0;
    }
  }
  
  LOGSTR("Failed to enter command mode (maybe already in command mode)\r\n");

  // If already in command mode this will give error but we get back to command mode anyway so we can ignore it
  HAL_UART_Transmit(&huart1, (uint8_t *) "\r\n", 2, HAL_MAX_DELAY);
  uart1_rx_getline(buf, sizeof(buf), 1000); // Clear any response from the "+++" command

  return 0;
}

/*!
  * @brief  Save WIZnet IP20 configuration to non-volatile memory.
  *
  * After configuring the WIZnet IP20 module with the desired settings using AT commands, we need to save the configuration
  * to non-volatile memory so that it persists across power cycles. This function sends the appropriate AT command to
  * trigger the save operation on the WIZnet IP20 module. The exact command may depend on the module's firmware version,
  * so you should refer to the WIZnet IP20 documentation for the correct command to use for saving the configuration.
  *
  * Note: This function assumes that you are already in command mode before calling it.
  *
  * @retval None
  */

void wiznet_ip20_save(void)
{
  // For some strange reason two SV is needed to really save
  HAL_UART_Transmit(&huart1, (uint8_t *) "SV\r\nSV\r\n", 8, HAL_MAX_DELAY);
}

/*!
  * @brief  Restart the WIZnet IP20 module to apply new settings.
  *
  * After saving the configuration, we need to restart the WIZnet IP20 module for the new settings to take effect. This
  * function sends the appropriate AT command to trigger a restart of the module. The exact command may depend on the
  * module's firmware version, so you should refer to the WIZnet IP20 documentation for the correct command to use for
  * restarting the module.
  *
  * Note: This function assumes that you are already in command mode before calling it.
  *
  * @retval None
*/
void wiznet_ip20_restart(void)
{
  char buf[80];
  HAL_UART_Transmit(&huart1, (uint8_t *) "RT\r\n", 8, HAL_MAX_DELAY);
  if (0 == uart1_rx_getline(buf, sizeof(buf), 1000)) {
     LOGSTR("Response: %s\r\n", buf);
  }
}

/*!
  * @brief  Send an AT command to the WIZnet IP20 module and wait for a response.
  *
  * This function sends a specified AT command string to the WIZnet IP20 module over UART1 and waits for a response. The
  * response is expected to be a complete line terminated by \r\n. The function uses the uart1_rx_getline() helper function
  * to read the response from the ring buffer with a specified timeout. If a response is received within the timeout, it
  * is stored in the provided response buffer and the function returns success. If no response is received within the
  * timeout, the function returns failure.
  *
  * Note: This function assumes that you are already in command mode before calling it.
  *
  * @param cmd                The AT command string to send (should include \r\n if required by the command).
  * @param response_buf       Buffer to store the received response (should be large enough to hold expected responses).
  * @param response_buf_size  Size of the response buffer in bytes.
  * @param timeout_ms         Timeout in milliseconds to wait for a response.
  * @retval int               Returns 1 on success (response received), or 0 on failure (timeout or error).
*/
int
wiznet_ip20_send_command(const char *cmd, char *response_buf, size_t response_buf_size, uint16_t timeout_ms)
{
  LOGSTR("WIZnet IP20 send command: %s\r\n", cmd);
  HAL_UART_Transmit(&huart1, (uint8_t *) cmd, strlen(cmd), HAL_MAX_DELAY);
  if (-1 == uart1_rx_getline(response_buf, response_buf_size, timeout_ms)) {
    LOGSTR("Failed to send command: %s\r\n", cmd);
    return 0;
  }
  return 1;
}

///////////////////////////////////////////////////////////////////////////////
// validate_user
//
int
validate_user(const char *user, const char *password)
{
  // For demonstration purposes, we accept a single hardcoded username/password.
  // In a real implementation, you should check against securely stored credentials.
  const char *valid_user     = "vscp";
  const char *valid_password = "secret";

  return (strcmp(user, valid_user) == 0) && (strcmp(password, valid_password) == 0);
}

/*!
 * @brief  Initialize the WIZnet IP20 module with the desired network settings and link protocol configuration.
 *
 * This function sends a series of AT commands to the WIZnet IP20 module over UART1 to configure its network settings
 * (IP address, netmask, gateway, DNS) and link protocol parameters (port, operation mode, connect/disconnect strings,
 * etc.) according to the defined constants. It uses the sendCommand() helper function to send each command and wait for
 * a response. The exact commands sent depend on the specific configuration you want for your application, and you
 * should refer to the WIZnet IP20 documentation for details on the available AT commands and their syntax.
 *
 * Note: This function assumes that you have already entered command mode using goCommandMode() before calling it.
 *
 * @retval VSCP_ERROR_SUCCESS if initialization was successful, VSCP error code on error.
 */
int
init_wiznet_ip20(void)
{
  char buf[100];
  size_t rx_len;

  // Set IP address
  HAL_UART_Transmit(&huart1, (uint8_t *) WIZ_IP20_IP, strlen(WIZ_IP20_IP), HAL_MAX_DELAY);

  // Set netmask
  HAL_UART_Transmit(&huart1, (uint8_t *) WIZ_IP20_NETMASK, strlen(WIZ_IP20_NETMASK), HAL_MAX_DELAY);

  // Set gateway
  HAL_UART_Transmit(&huart1, (uint8_t *) WIZ_IP20_GATEWAY, strlen(WIZ_IP20_GATEWAY), HAL_MAX_DELAY);

  // Set DNS server
  HAL_UART_Transmit(&huart1, (uint8_t *) WIZ_IP20_DNS, strlen(WIZ_IP20_DNS), HAL_MAX_DELAY);

  // Set IP allocation mode
  HAL_UART_Transmit(&huart1,
                    (uint8_t *) WIZ_IP20_IP_ALLOCATION_METHOD,
                    strlen(WIZ_IP20_IP_ALLOCATION_METHOD),
                    HAL_MAX_DELAY);

  // Set port
  HAL_UART_Transmit(&huart1, (uint8_t *) WIZ_IP20_PORT, strlen(WIZ_IP20_PORT), HAL_MAX_DELAY);

  // Set operation mode
  HAL_UART_Transmit(&huart1, (uint8_t *) WIZ_IP20_OPERATION_MODE, strlen(WIZ_IP20_OPERATION_MODE), HAL_MAX_DELAY);

  // Set connect string
  HAL_UART_Transmit(&huart1, (uint8_t *) WIZ_IP20_LINK_CONNECT_STR, strlen(WIZ_IP20_LINK_CONNECT_STR), HAL_MAX_DELAY);

  // Set disconnect string
  HAL_UART_Transmit(&huart1,
                    (uint8_t *) WIZ_IP20_LINK_DISCONNECT_STR,
                    strlen(WIZ_IP20_LINK_DISCONNECT_STR),
                    HAL_MAX_DELAY);

  // Set Ethernet connect string
  HAL_UART_Transmit(&huart1,
                    (uint8_t *) WIZ_IP20_LINK_ETH_CONNECT_STR,
                    strlen(WIZ_IP20_LINK_ETH_CONNECT_STR),
                    HAL_MAX_DELAY);

#if defined(BLINKY_MODE_TCP_CLIENT) || defined(BLINKY_MODE_MQTT)
  // Remote host
  HAL_UART_Transmit(&huart1, (uint8_t *) WIZ_IP20_REMOTE_HOST_IP, strlen(WIZ_IP20_REMOTE_HOST_IP), HAL_MAX_DELAY);

  // Remote port
  HAL_UART_Transmit(&huart1, (uint8_t *) WIZ_IP20_REMOTE_HOST_PORT, strlen(WIZ_IP20_REMOTE_HOST_PORT), HAL_MAX_DELAY);

  // TCP client reconnect interval
  HAL_UART_Transmit(&huart1,
                    (uint8_t *) WIZ_IP20_LINK_CLIENT_RECONNECT,
                    strlen(WIZ_IP20_LINK_CLIENT_RECONNECT),
                    HAL_MAX_DELAY);
#endif

  // Data packing time in milliseconds
  HAL_UART_Transmit(&huart1, (uint8_t *) WIZ_IP20_LINK_PACKING_TIME, strlen(WIZ_IP20_LINK_PACKING_TIME), HAL_MAX_DELAY);

  // Data packing size in bytes (0 not used)
  HAL_UART_Transmit(&huart1, (uint8_t *) WIZ_IP20_LINK_PACKING_SIZE, strlen(WIZ_IP20_LINK_PACKING_SIZE), HAL_MAX_DELAY);

  // Data packing char (0 not used, 0x0a = newline) The designated character is not included in data.
  HAL_UART_Transmit(&huart1, (uint8_t *) WIZ_IP20_LINK_PACKING_CHAR, strlen(WIZ_IP20_LINK_PACKING_CHAR), HAL_MAX_DELAY);

  // Inactivity time in seconds (0 not used)
  HAL_UART_Transmit(&huart1, (uint8_t *) WIZ_IP20_LINK_INACTIVITY, strlen(WIZ_IP20_LINK_INACTIVITY), HAL_MAX_DELAY);

  // Retry count (0 not used)
  // HAL_UART_Transmit(&huart1, (uint8_t *) WIZ_IP20_LINK_RETRY_CNT, strlen(WIZ_IP20_LINK_RETRY_CNT), HAL_MAX_DELAY);

  // Keep alive (0 not used, 1 = enabled)
  HAL_UART_Transmit(&huart1, (uint8_t *) WIZ_IP20_LINK_KEEP_ALIVE, strlen(WIZ_IP20_LINK_KEEP_ALIVE), HAL_MAX_DELAY);

  // Keep alive interval in milliseconds
  HAL_UART_Transmit(&huart1,
                    (uint8_t *) WIZ_IP20_LINK_KEEP_ALIVE_INTERVAL,
                    strlen(WIZ_IP20_LINK_KEEP_ALIVE_INTERVAL),
                    HAL_MAX_DELAY);

  // Keep alive retry interval in milliseconds
  HAL_UART_Transmit(&huart1,
                    (uint8_t *) WIZ_IP20_LINK_KEEP_ALIVE_RETRY_INTERVAL,
                    strlen(WIZ_IP20_LINK_KEEP_ALIVE_RETRY_INTERVAL),
                    HAL_MAX_DELAY);

  // SSL close (0 not used, 1 = enabled)
  HAL_UART_Transmit(&huart1, (uint8_t *) WIZ_IP20_LINK_SSL_CLOSE, strlen(WIZ_IP20_LINK_SSL_CLOSE), HAL_MAX_DELAY);

  // Password for TCP server (0 not used)
  HAL_UART_Transmit(&huart1,
                    (uint8_t *) WIZ_IP20_TCPSRV_ENABLE_PASSWORD,
                    strlen(WIZ_IP20_TCPSRV_ENABLE_PASSWORD),
                    HAL_MAX_DELAY);

  // #Password for TCP server (max 8 characters)
  // HAL_UART_Transmit(&huart1,
  //                   (uint8_t *) WIZ_IP20_TCPSRV_SET_PASSWORD,
  //                   strlen(WIZ_IP20_TCPSRV_SET_PASSWORD),
  //                   HAL_MAX_DELAY);

  // Search id code
  // HAL_UART_Transmit(&huart1, (uint8_t *) WIZ_IP20_SEARCH_ID_CODE, strlen(WIZ_IP20_SEARCH_ID_CODE), HAL_MAX_DELAY);

  // Enable debug messages (0 not used, 1 = enabled)
  HAL_UART_Transmit(&huart1, (uint8_t *) WIZ_IP20_DEBUG_ENABLE, strlen(WIZ_IP20_DEBUG_ENABLE), HAL_MAX_DELAY);

  // MQTT settings (not used here)
#ifdef BLINKY_DEMO_MODE_MQTT
  // QUmqtt_user\r\n MQTT username (max 128 characters)
  HAL_UART_Transmit(&huart1, (uint8_t *) WIZ_IP20_MQTT_USER, strlen(WIZ_IP20_MQTT_USER), HAL_MAX_DELAY);

  // QPmqtt_password\r\n MQTT password (max 128 characters)
  HAL_UART_Transmit(&huart1, (uint8_t *) WIZ_IP20_MQTT_PASSWORD, strlen(WIZ_IP20_MQTT_PASSWORD), HAL_MAX_DELAY);

  // QCmqtt_client_id\r\n MQTT client ID (max 128 characters)
  HAL_UART_Transmit(&huart1, (uint8_t *) WIZ_IP20_MQTT_CLIENT_ID, strlen(WIZ_IP20_MQTT_CLIENT_ID), HAL_MAX_DELAY);

  // QK0\r\n MQTT keep alive (0 not used, 1 = enabled)
  HAL_UART_Transmit(&huart1, (uint8_t *) WIZ_IP20_MQTT_KEEP_ALIVE, strlen(WIZ_IP20_MQTT_KEEP_ALIVE), HAL_MAX_DELAY);

  // QQ0\r\n MQTT QoS (0, 1, or 2)
  HAL_UART_Transmit(&huart1, (uint8_t *) WIZ_IP20_MQTT_QOS, strlen(WIZ_IP20_MQTT_QOS), HAL_MAX_DELAY);

  // Publish topic\r\n MQTT publish topic (max 128 characters)
  HAL_UART_Transmit(&huart1, (uint8_t *) WIZ_IP20_MQTT_PUB_TOPIC, strlen(WIZ_IP20_MQTT_PUB_TOPIC), HAL_MAX_DELAY);

  // MQTT subscribe topic 1 (max 128 characters)
  HAL_UART_Transmit(&huart1, (uint8_t *) WIZ_IP20_MQTT_SUB_TOPIC0, strlen(WIZ_IP20_MQTT_SUB_TOPIC0), HAL_MAX_DELAY);

  // MQTT subscribe topic 2 (max 128 characters)
  HAL_UART_Transmit(&huart1, (uint8_t *) WIZ_IP20_MQTT_SUB_TOPIC1, strlen(WIZ_IP20_MQTT_SUB_TOPIC1), HAL_MAX_DELAY);

  // MQTT subscribe topic 3 (max 128 characters)
  HAL_UART_Transmit(&huart1, (uint8_t *) WIZ_IP20_MQTT_SUB_TOPIC2, strlen(WIZ_IP20_MQTT_SUB_TOPIC2), HAL_MAX_DELAY);

#endif
  

  return VSCP_ERROR_SUCCESS;
}

/*!
 * @brief  Show the current WIZnet IP20 settings by querying the module and printing the results.
 *
 * This function sends a series of AT commands to the WIZnet IP20 module to query its current configuration settings
 * (such as IP address, netmask, gateway, DNS, operation mode, etc.) and prints the responses to the console. This can
 * be useful for debugging and verifying that the module is configured correctly after initialization.
 *
 * Note: This function assumes that you have already entered command mode using goCommandMode() before calling it.
 *
 * @retval VSCP_ERROR_SUCCESS if settings were successfully queried and displayed, VSCP error code on error.
 */

int
show_wiznet_ip20_settings(void)
{
  char buf[100];
  size_t rx_len;

  LOGSTR("WIZnet IP20 current settings:\r\n");

  // Query product name
  HAL_UART_Transmit(&huart1, (uint8_t *) "MN\r\n", 4, HAL_MAX_DELAY);
  if (0 == uart1_rx_getline(buf, sizeof(buf), 5000)) {
    LOGSTR("Response: %s\r\n", buf);
  }

  // Query firmware version
  HAL_UART_Transmit(&huart1, (uint8_t *) "VR\r\n", 4, HAL_MAX_DELAY);
  if (0 == uart1_rx_getline(buf, sizeof(buf), 5000)) {
    LOGSTR("Response: %s\r\n", buf);
  }

  // Query MAC address
  HAL_UART_Transmit(&huart1, (uint8_t *) "MC\r\n", 4, HAL_MAX_DELAY);
  if (0 == uart1_rx_getline(buf, sizeof(buf), 5000)) {
    LOGSTR("Response: %s\r\n", buf);
  }

  // Query operation mode
  HAL_UART_Transmit(&huart1, (uint8_t *) "OP\r\n", 4, HAL_MAX_DELAY);
  if (0 == uart1_rx_getline(buf, sizeof(buf), 5000)) {
    LOGSTR("Response: %s\r\n", buf);
  }

  // Get connect string
  HAL_UART_Transmit(&huart1, (uint8_t *) "SD\r\n", 4, HAL_MAX_DELAY);
  if (0 == uart1_rx_getline(buf, sizeof(buf), 5000)) {
    LOGSTR("Response: %s\r\n", buf);
  }

  // Get connect string
  HAL_UART_Transmit(&huart1, (uint8_t *) "DD\r\n", 4, HAL_MAX_DELAY);
  if (0 == uart1_rx_getline(buf, sizeof(buf), 5000)) {
    LOGSTR("Response: %s\r\n", buf);
  }

  // Get UART interface
  HAL_UART_Transmit(&huart1, (uint8_t *) "UN\r\n", 4, HAL_MAX_DELAY);
  if (0 == uart1_rx_getline(buf, sizeof(buf), 5000)) {
    LOGSTR("Response: %s\r\n", buf);
  }

  // Get local IP address
  HAL_UART_Transmit(&huart1, (uint8_t *) "LI\r\n", 4, HAL_MAX_DELAY);
  if (0 == uart1_rx_getline(buf, sizeof(buf), 5000)) {
    LOGSTR("Response: %s\r\n", buf);
  }

  // Get local subnet mask
  HAL_UART_Transmit(&huart1, (uint8_t *) "SM\r\n", 4, HAL_MAX_DELAY);
  if (0 == uart1_rx_getline(buf, sizeof(buf), 5000)) {
    LOGSTR("Response: %s\r\n", buf);
  }

  // Get local gateway IP address
  HAL_UART_Transmit(&huart1, (uint8_t *) "GW\r\n", 4, HAL_MAX_DELAY);
  if (0 == uart1_rx_getline(buf, sizeof(buf), 5000)) {
    LOGSTR("Response: %s\r\n", buf);
  }
  // Get local DNS IP address
  HAL_UART_Transmit(&huart1, (uint8_t *) "DS\r\n", 4, HAL_MAX_DELAY);
  if (0 == uart1_rx_getline(buf, sizeof(buf), 5000)) {
    LOGSTR("Response: %s\r\n", buf);
  }

  // Get local port number
  HAL_UART_Transmit(&huart1, (uint8_t *) "LP\r\n", 4, HAL_MAX_DELAY);
  if (0 == uart1_rx_getline(buf, sizeof(buf), 5000)) {
    LOGSTR("Response: %s\r\n", buf);
  }

  HAL_UART_Transmit(&huart1, (uint8_t *) "\r\n", 4, HAL_MAX_DELAY);
  if (0 == uart1_rx_getline(buf, sizeof(buf), 5000)) {
    LOGSTR("Response: %s\r\n", buf);
  }

  return VSCP_ERROR_SUCCESS;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int
main(void)
{
  /* USER CODE BEGIN 1 */
  char buf[100]; // Buffer for AT command responses and incoming data
  size_t rx_len;
  (void) rx_len;                  // only used to capture AT response lengths during init
  uint32_t led_blink_until   = 0; // LED blink timestamp
  vscp_state_t vscp_state    = VSCP_STATE_DISCONNECTED;
  vscp_state_t vscp_substate = VSCP_SUBSTATE_POLL;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* Start TIM2 in interrupt mode — fires every 1 ms */
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
    Error_Handler();
  }

  /* Start TIM3 — free-running 1 µs counter, overflow interrupt for 32-bit extension */
  if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK) {
    Error_Handler();
  }

  // Switch to IRQ-driven receive for data mode
  uart1_rx_start();

  // Initialize context for client connections
  // setContextDefaults(&ctx[0]);

  /*
    * WIZ-IP20 initialization sequence (AT command mode, blocking)
      We use blocking calls here for simplicity since this only runs once at startup.

      Commands are here: https://docs.wiznet.io/Product/Modules/Serial-to-Ethernet-Module/W232N/command-manual-en
  */

  // rx_len = getWizIp20Response(buf, sizeof(buf), 10);

  wiznet_ip20_command_mode();
  init_wiznet_ip20();
  show_wiznet_ip20_settings();
  wiznet_ip20_save();
  wiznet_ip20_restart();


  // Debug print to indicate that the program has started
  LOGSTR("STM32F103 Wiznet IP20 VSCP blink demo starting\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* ------------------------------------------------------------------
     * State machine – scan ring buffer for control tokens (no \r\n needed)
     * ----------------------------------------------------------------*/
    switch (vscp_state) {

      case VSCP_STATE_DISCONNECTED:
        if (uart1_rx_scan("<CONNECT>") == 0) {
          uart1_rx_consume_through("<CONNECT>");
          LOGSTR("Received <CONNECT>\r\n");
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
          led_blink_until      = HAL_GetTick() + 50u;
          const char welcome[] = "Welcome to the VSCP daemon on ST32 + Wiznet IP20.\r\n"
                                 "Version: 0.0.1\r\n"
                                 "Copyright (c) 2000-2026 Ake Hedman, Grodans \r\n"
                                 "Paradis AB, https://www.grodansparadis.com\r\n"
                                 "+OK - Success.\r\n";
          HAL_UART_Transmit(&huart1, (uint8_t *) welcome, sizeof(welcome) - 1, HAL_MAX_DELAY);
          vscp_state = VSCP_STATE_CONNECTED;
          LOGSTR("State: DISCONNECTED -> CONNECTED\r\n");
        }
        else {
          /* Discard any \r\n-terminated lines that aren't <CONNECT> */
          uart1_rx_getline(buf, sizeof(buf), 0);
        }
        break;

      case VSCP_STATE_CONNECTED:
        if (uart1_rx_scan("<DISCONNECT>") == 0) {
          uart1_rx_consume_through("<DISCONNECT>");
          LOGSTR("Received <DISCONNECT>\n");
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
          led_blink_until = HAL_GetTick() + 50u;
          HAL_UART_Transmit(&huart1, (uint8_t *) "+OK - Disconnect.\r\n", 19, HAL_MAX_DELAY);
          vscp_state = VSCP_STATE_DISCONNECTED;
          LOGSTR("State: CONNECTED -> DISCONNECTED\r\n");
        }
        else if (uart1_rx_getline(buf, sizeof(buf), 0) == 0) {
          /* Consume and log incoming VSCP commands */
          LOGSTR("VSCP cmd: %s", buf);
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
          led_blink_until = HAL_GetTick() + 50u;
          vscp_link_parser(&ctx_link, buf);
        }
        break;

      default:
        vscp_state = VSCP_STATE_DISCONNECTED;
        break;
    }

    /* Turn LED off after 50 ms blink */
    if (led_blink_until != 0u && HAL_GetTick() >= led_blink_until) {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); /* LED off */
      led_blink_until = 0u;
    }

    /* Toggle the state of pin 13 on GPIO port C */
    // if ((HAL_GetTick() - now) >= 500) {
    //   now = HAL_GetTick();
    //   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    //   LOGSTR("Super\r\n"); // Print "Super" to the ITM console
    //   const char msg[] = "hello\r\n";
    //   HAL_UART_Transmit(&huart2, (uint8_t *) msg, sizeof(msg) - 1, HAL_MAX_DELAY);
    // }

    /* Wait for 500 milliseconds */
    // HAL_Delay(500);
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void
SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
  RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void
Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void
assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: LOGSTR("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
