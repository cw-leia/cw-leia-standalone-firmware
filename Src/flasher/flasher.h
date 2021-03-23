#ifndef __FLASHER_H__
#define __FLASHER_H__

#include "flasher_platform.h"

#define LOW_BYTE        0
#define HIGH_BYTE       1

int send_flasher_spi_generic(uint8_t b_in, uint8_t *b_out);

int send_flasher_command(uint8_t command[4], uint8_t response[4]);

int flasher_programming_enable(uint32_t *frequency);

int flasher_release_reset(void);

int flasher_leave_programming_mode(void);

int flasher_chip_erase(void);

int flasher_read_program_memory_byte(uint16_t address, uint8_t low_high, uint8_t *byte);

int flasher_load_program_memory_page(uint8_t byte_offset, uint8_t low_high, uint8_t byte);

int flasher_write_program_memory_page(uint8_t page_address);

int flasher_read_eeprom_memory(uint16_t address, uint8_t *byte);

int flasher_write_eeprom_memory(uint16_t address, uint8_t byte);

int flasher_read_lock_bits(uint8_t *byte);

int flasher_write_lock_bits(uint8_t byte);

int flasher_read_signature_byte(uint8_t address, uint8_t *byte);

int flasher_write_fuse_bits(uint8_t byte);

int flasher_write_fuse_high_bits(uint8_t byte);

int flasher_read_fuse_bits(uint8_t *byte);

int flasher_read_fuse_high_bits(uint8_t *byte);

int flasher_read_device_codes(uint8_t *vendor, uint8_t *family, uint8_t *part);

#endif /* __FLASHER_H__ */
