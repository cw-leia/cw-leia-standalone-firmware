#include "flasher.h"
#include "flasher_print.h"

/*
 * This is a straightforward implementation of the ISP commands
 * for the Atmega8515 MCU available here (table on page 196):
 * https://ww1.microchip.com/downloads/en/DeviceDoc/doc2512.pdf
 *
 * NOTE: although most of the following commands are common
 * to most of/all AVRs, some differences might arise and it
 * is not safe to assume that this is compatible with all the
 * AVR family (minor modifications and additions might be
 * necessary).
 */


/* Generic SPI send / receive byte */
int send_flasher_spi_generic(uint8_t b_in, uint8_t *b_out)
{
    if(b_out == NULL){
        goto err;
    }
    platform_flasher_xfer_byte(b_in, b_out, MSB);

    return 0;
err:
    return -1;
}


/* Generic ISP SPI send/receive command (4 byte sent / 4 bytes
 * received)
 */
int send_flasher_command(uint8_t command[4], uint8_t response[4])
{
    if((command == NULL) || (response == NULL)){
        goto err;
    }
    platform_flasher_xfer_byte(command[0], &response[0], MSB);
    platform_flasher_xfer_byte(command[1], &response[1], MSB);
    platform_flasher_xfer_byte(command[2], &response[2], MSB);
    platform_flasher_xfer_byte(command[3], &response[3], MSB);

    /* Sanity check */
    if(response[2] != command[1]){
        goto err;
    }

    return 0;
err:
    return -1;
}

#define FLASHER_DEFAULT_FREQUENCY 450000

int flasher_programming_enable(uint32_t *frequency)
{
    platform_flasher_init();
    if(frequency == NULL){
        uint32_t curr_freq;
        /* Conservative value for the frequency
         * NOTE: the SPI ISP frequency must be:
         *   - Less than 1/4 of the AVR clock frequency if it is less than 12 MHz
         *   - Less than 1/6 of the AVR clock frequency if it is greater or equal to 12 MHz
         */
        curr_freq = FLASHER_DEFAULT_FREQUENCY;
        if(platform_flasher_set_freq(&curr_freq)){
            goto err;
        }    
    }
    else{
        if(platform_flasher_set_freq(frequency)){
            goto err;
        }
    }
    platform_flasher_set_rst(1);
    platform_flasher_set_clk(0);
    platform_flasher_delay_milliseconds(50);

    platform_flasher_set_rst(0);
    platform_flasher_delay_milliseconds(50);
    platform_flasher_set_mosi(0);

    uint8_t command[4]  = { 0xac, 0x53, 0x00, 0x00 };
    uint8_t response[4] = { 0 };
    if(send_flasher_command(command, response)){
        goto err;
    }

    return 0;
err:
    return -1;
}

int flasher_release_reset(void)
{
    platform_flasher_set_rst(1);

    return 0;
}

int flasher_leave_programming_mode(void)
{
    /* Leaving the programming mode consists of releasing reset */
    flasher_release_reset();

    /* And deinit our hardware (stop our clocks, etc.) */
    platform_flasher_deinit();

    return 0;
}

int flasher_chip_erase(void)
{
    uint8_t command[4]  = { 0xac, 0x80, 0x00, 0x00 };
    uint8_t response[4] = { 0 };
    if(send_flasher_command(command, response)){
        goto err;
    }

    return 0;
err:
    return -1;
}

int flasher_read_program_memory_byte(uint16_t address, uint8_t low_high, uint8_t *byte)
{
    if(byte == NULL){
        goto err;
    }

    uint8_t command[4]  = { 0x20, 0x00, 0x00, 0x00 };
    uint8_t response[4] = { 0 };
    
    if(low_high == HIGH_BYTE){
        command[0] |= 0x08;
    }
    else if(low_high != LOW_BYTE){
        goto err;
    }
    /* Address overflow */
    if(address >> 12){
        goto err;
    }
    command[1] = (address >> 8) & 0x0f;
    command[2] = address & 0xff;
    if(send_flasher_command(command, response)){
        goto err;
    }
    *byte = response[3];

    return 0;
err:
    return -1;
}


int flasher_load_program_memory_page(uint8_t byte_offset, uint8_t low_high, uint8_t byte)
{
    uint8_t command[4]  = { 0x40, 0x00, 0x00, 0x00 };
    uint8_t response[4] = { 0 };
    
    if(low_high == HIGH_BYTE){
        command[0] |= 0x08;
    }
    else if(low_high != LOW_BYTE){
        goto err;
    }
    /* Byte offset overflow (only on 5 bits) */
    if(byte_offset >> 5){
        goto err;
    }
    command[2] = byte_offset & 0x1f;
    command[3] = byte;

    if(send_flasher_command(command, response)){
        goto err;
    }

    return 0;
err:
    return -1;
}

int flasher_write_program_memory_page(uint8_t page_address)
{
    uint8_t command[4]  = { 0x4c, 0x00, 0x00, 0x00 };
    uint8_t response[4] = { 0 };

    /* Page address overflow (only on 7 bits) */
    if(page_address >> 7){
        goto err;
    }
    command[1] = (page_address >> 3) & 0x0f;
    command[2] = (page_address & 0x7) << 5;

    if(send_flasher_command(command, response)){
        goto err;
    }

    return 0;
err:
    return -1;
}

int flasher_read_eeprom_memory(uint16_t address, uint8_t *byte)
{
    if(byte == NULL){
        goto err;
    }
    uint8_t command[4]  = { 0xa0, 0x00, 0x00, 0x00 };
    uint8_t response[4] = { 0 };

    /* EEPROM address overflow (only on 9 bits) */
    if(address >> 9){
        goto err;
    }
    command[1] = (address >> 8) & 0x1;
    command[2] = address & 0xff;

    if(send_flasher_command(command, response)){
        goto err;
    }
    *byte = response[3];

    return 0;
err:
    return -1;
}

int flasher_write_eeprom_memory(uint16_t address, uint8_t byte)
{
    uint8_t command[4]  = { 0xc0, 0x00, 0x00, 0x00 };
    uint8_t response[4] = { 0 };

    /* EEPROM address overflow (only on 9 bits) */
    if(address >> 9){
        goto err;
    }
    command[1] = (address >> 8) & 0x1;
    command[2] = address & 0xff;
    command[3] = byte;

    if(send_flasher_command(command, response)){
        goto err;
    }

    return 0;
err:
    return -1;
}

int flasher_read_lock_bits(uint8_t *byte)
{
    if(byte == NULL){
        goto err;
    }
    uint8_t command[4]  = { 0x58, 0x00, 0x00, 0x00 };
    uint8_t response[4] = { 0 };

    if(send_flasher_command(command, response)){
        goto err;
    }
    *byte = response[3] & 0x3f;

    return 0;
err:
    return -1;
}

int flasher_write_lock_bits(uint8_t byte)
{
    if(byte >> 6){
        goto err;
    }
    uint8_t command[4]  = { 0xac, 0x00, 0x00, 0x00 };
    uint8_t response[4] = { 0 };

    command[3] = 0xc0 | (byte & 0x3f);
    if(send_flasher_command(command, response)){
        goto err;
    }

    return 0;
err:
    return -1;
}

int flasher_read_signature_byte(uint8_t address, uint8_t *byte)
{
    if(byte == NULL){
        goto err;
    }
    /* Signature bytes address is on 2 bits */
    if(address >> 2){
        goto err;
    }
    uint8_t command[4]  = { 0x30, 0x00, 0x00, 0x00 };
    uint8_t response[4] = { 0 };    
    command[2] = address & 0x0f;
    if(send_flasher_command(command, response)){
        goto err;
    }
    *byte = response[3];

    return 0;
err:
    return -1;
}

int flasher_write_fuse_bits(uint8_t byte)
{
    uint8_t command[4]  = { 0xac, 0xa0, 0x00, 0x00 };
    uint8_t response[4] = { 0 };    
    command[3] = byte;
    if(send_flasher_command(command, response)){
        goto err;
    }

    return 0;
err:
    return -1;
}

int flasher_write_fuse_high_bits(uint8_t byte)
{
    uint8_t command[4]  = { 0xa8, 0xa0, 0x00, 0x00 };
    uint8_t response[4] = { 0 };
    command[3] = byte;
    if(send_flasher_command(command, response)){
        goto err;
    }

    return 0;
err:
    return -1;
}

int flasher_read_fuse_bits(uint8_t *byte)
{
    if(byte == NULL){
        goto err;
    }
    uint8_t command[4]  = { 0x50, 0x00, 0x00, 0x00 };
    uint8_t response[4] = { 0 };
    if(send_flasher_command(command, response)){
        goto err;
    }
    *byte = response[3];

    return 0;
err:
    return -1;
}

int flasher_read_fuse_high_bits(uint8_t *byte)
{
    if(byte == NULL){
        goto err;
    }
    uint8_t command[4]  = { 0x58, 0x00, 0x00, 0x00 };
    uint8_t response[4] = { 0 };
    if(send_flasher_command(command, response)){
        goto err;
    }
    *byte = response[3];

    return 0;
err:
    return -1;
}

int flasher_read_device_codes(uint8_t *vendor, uint8_t *family, uint8_t *part)
{
    if(vendor != NULL){
        if(flasher_read_signature_byte(0x00, vendor)){
            goto err;
        }
    }
    if(family != NULL){
        if(flasher_read_signature_byte(0x01, family)){
            goto err;
        }
    }
    if(part != NULL){
        if(flasher_read_signature_byte(0x02, part)){
            goto err;
        }
    }

    return 0;
err:
    return -1;
}

int flasher_read_calibration_byte(uint8_t address, uint8_t *byte)
{
    if(byte == NULL){
        goto err;
    }
    /* Signature bytes address is on 2 bits */
    if(address >> 2){
        goto err;
    }
    uint8_t command[4]  = { 0x38, 0x00, 0x00, 0x00 };
    uint8_t response[4] = { 0 };    
    command[2] = address & 0x0f;
    if(send_flasher_command(command, response)){
        goto err;
    }
    *byte = response[3];

    return 0;
err:
    return -1;
}


