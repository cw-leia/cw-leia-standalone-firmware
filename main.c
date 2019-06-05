#include "debug.h"
#include "helpers.h"
#include "init.h"
#include "leds.h"
#include "stm32f4xx_usart.h"
#include "uart.h"

#include "smartcard.h"
#include "smartcard_iso7816_platform.h"
#include "protocol.h"
#include "ring_buffer.h"
#include "triggers.h"

volatile usart_config_t debug_console_config = {0};
volatile usart_config_t console_config = {0};

volatile struct s_ring console_rx_ring_buffer;
volatile struct s_ring debug_rx_ring_buffer;

static void console_usart_data_received(void){
  char c = usart_getc(console_config.usart);
  ring_buffer_write_char(&console_rx_ring_buffer, c);
}

static void debug_usart_data_received(void) {
  char c = usart_getc(debug_console_config.usart);
  usart_putc(debug_console_config.usart, c);
  ring_buffer_write_char(&debug_rx_ring_buffer, c);
}

void init_leia(int argc, char *args[]) {
  disable_irq();

  char *base_address = 0;
  if (argc == 1) {
    base_address = (char *)args[0];
    system_init((uint32_t)base_address - VTORS_SIZE);
  } else {
    panic("Unable to find base address\n");
  }

  // Initialize LEDS
  leds_init();

  // Initialize sys tick
  systick_init();
  
  // Initialize UARTs (log + console)
  debug_console_config.callback_irq_handler = debug_usart_data_received;
  console_config.callback_irq_handler = console_usart_data_received;
  debug_console_init((usart_config_t *)&debug_console_config);
  console_init((usart_config_t *)&console_config);

  // Initialize UARTs RX Ring Buffers
  init_ring_buffer(&console_rx_ring_buffer);
  init_ring_buffer(&debug_rx_ring_buffer);


  enable_irq();

  dbg_log("======== LEIA FIRMWARE =========\n");
  dbg_log("\tBuilt date\t: %s at %s\n", __DATE__, __TIME__);
  dbg_log("\tBoard\t\t: STM32F407 Discovery\n");
  dbg_log("================================\n");
  dbg_flush();

  // Initialize Smartcard reader contact pin
  platform_init_smartcard_contact();

  // Initialize trigger IO
  trigger_init_IO();
}

int main(int argc, char *args[]) {

  SC_Card card;
  init_leia(argc, args);

  protocol_parse_cmd(&console_rx_ring_buffer, &card);

  return 0;
}
