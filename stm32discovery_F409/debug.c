#include "debug.h"
#include "helpers.h"
#include "product.h"
#include "stm32f4xx_usart.h"
#include <stdarg.h>

#define BUF_SIZE 4096
#define BUF_MAX (BUF_SIZE - 1)
#define PUT_CHAR(c)                                                            \
  debug_ring_buffer.buf[debug_ring_buffer.end++] = c;                                      \
  debug_ring_buffer.end %= BUF_MAX;                                                  \
  if (debug_ring_buffer.end == debug_ring_buffer.start) {                                  \
    debug_ring_buffer.start++;                                                       \
    debug_ring_buffer.start %= BUF_MAX;                                              \
  }

static volatile int logging = 1;

cb_usart_getc_t debug_console_getc = NULL;
cb_usart_putc_t debug_console_putc = NULL;

static struct {
  uint32_t start;
  uint32_t end;
  char buf[BUF_SIZE];
} debug_ring_buffer;

void init_debug_ring_buffer(void) {
  int i = 0;
  debug_ring_buffer.end = 0;
  debug_ring_buffer.start = debug_ring_buffer.end;

  for (i = 0; i < BUF_MAX; i++) {
    debug_ring_buffer.buf[i] = '\0';
  }
}

static uint32_t current_dbg_usart = 0;
void cb_debug_data_irq(void) {
  if (get_reg(r_CORTEX_M_USART_SR(current_dbg_usart), USART_SR_RXNE)) {
    /* Data received */
    char c;
    if (debug_console_getc == NULL) {
      panic("Error: console_getc not initialized!");
    }

    c = debug_console_getc();

    if (c != ' ' && c != 'p') {
      dbg_flush();
      return;
    }

    logging = !logging;
    if (logging)
      dbg_log("Logging enabled\n");
    else
      dbg_log("Logging disabled\n");
    dbg_flush();
  }
}


void debug_console_init(usart_config_t *config) {
  init_debug_ring_buffer();
  /* Configure the USART in UART mode */
  if (config != NULL) {
    config->set_mask = USART_SET_ALL;
    config->usart = DEBUG_CONSOLE_USART;
    config->baudrate = 115200;
    config->word_length = USART_CR1_M_8;
    config->stop_bits = USART_CR2_STOP_1BIT;
    config->parity = USART_CR1_PCE_DIS;
    config->hw_flow_control = USART_CR3_CTSE_CTS_DIS | USART_CR3_RTSE_RTS_DIS;
    /* We disable the TX interrupt since we will handle it with polling */
    config->options_cr1 = USART_CR1_TE_EN | USART_CR1_RE_EN | USART_CR1_UE_EN |
                          USART_CR1_RXNEIE_EN | USART_CR1_TCIE_DIS;
    if (config->callback_irq_handler == NULL) {
      config->callback_irq_handler = cb_debug_data_irq;
    }
    if (config->callback_usart_getc_ptr == NULL) {
      config->callback_usart_getc_ptr = &debug_console_getc;
    }
    if (config->callback_usart_putc_ptr == NULL) {
      config->callback_usart_putc_ptr = &debug_console_putc;
    }
    current_dbg_usart = PROD_CONSOLE_USART;
  } else {
    return;
  }
  /* Initialize the USART related to the console */
  usart_init(config);
  dbg_log("[USART%d initialized for debug output, baudrate=%d]\n",
          config->usart, config->baudrate);
  dbg_flush();
}

static void write_digit(uint8_t digit) {
  if (digit < 0xa)
    digit += '0';
  else
    digit += 'a' - 0xa;
  PUT_CHAR(digit);
}

static void itoa(unsigned long long value, uint8_t base) {
  if (value / base == 0) {
    write_digit(value % base);
  } else {
    itoa(value / base, base);
    write_digit(value % base);
  }
}

static void copy_string(char *str, uint32_t len) {
  uint32_t size =
      len < (BUF_MAX - debug_ring_buffer.end) ? len : BUF_MAX - debug_ring_buffer.end;
  strncpy(debug_ring_buffer.buf + debug_ring_buffer.end, str, size);
  uint32_t dist = debug_ring_buffer.start - debug_ring_buffer.end;
  if (debug_ring_buffer.end < debug_ring_buffer.start && dist < size) {
    debug_ring_buffer.start += size - dist + 1;
    debug_ring_buffer.start %= BUF_MAX;
  }
  debug_ring_buffer.end += size;
  debug_ring_buffer.end %= BUF_MAX;
  if (len - size)
    copy_string(str + size, len - size);
}

void dbg_flush(void) {
  if (debug_console_putc == NULL) {
    panic("Error: console_putc not initialized");
  }
  while (debug_ring_buffer.start != debug_ring_buffer.end) {
    debug_console_putc(debug_ring_buffer.buf[debug_ring_buffer.start++]);
    debug_ring_buffer.start %= BUF_MAX;
  }
}

static void print(char *fmt, va_list args) {
  uint32_t i = 0;
  char *string;

  for (i = 0; fmt[i]; i++) {
    if (fmt[i] == '%') {
      i++;
      switch (fmt[i]) {
      case 'd':
        itoa(va_arg(args, uint32_t), 10);
        break;
      case 'x':
        // PUT_CHAR('0');
        // PUT_CHAR('x');
        itoa(va_arg(args, uint32_t), 16);
        break;
      case '%':
        PUT_CHAR('%');
        break;
      case 's':
        string = va_arg(args, char *);
        copy_string(string, strlen(string));
        break;
      case 'l':
        if (fmt[i + 1] == 'l' && fmt[i + 2] == 'd') {
          itoa(va_arg(args, unsigned long long), 10);
          i += 2;
        } else if (fmt[i + 1] == 'd') {
          itoa(va_arg(args, unsigned long), 10);
          i++;
        }
        break;
      case 'c':
        PUT_CHAR((unsigned char)va_arg(args, int));
        break;
      default:
        PUT_CHAR('?');
      }
    } else if (fmt[i] == '\n' && fmt[i + 1] != '\r') {
      copy_string("\n\r", 2);
    } else {
      PUT_CHAR(fmt[i]);
    }
  }
}

void dbg_log(char *fmt, ...) {
  va_list args;

  va_start(args, fmt);
  print(fmt, args);
  va_end(args);
}

void panic(char *fmt, ...) {
  va_list args;

  va_start(args, fmt);
  print(fmt, args);
  va_end(args);
  dbg_flush();
  __asm__ volatile("bkpt\n");
  while (1)
    continue;
}
