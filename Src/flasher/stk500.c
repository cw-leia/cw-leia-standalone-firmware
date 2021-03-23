#include "flasher.h"
#include "flasher_print.h"
#include "helpers.h"
#include "console.h"
#include "stk500.h"
#include "leds.h"

/* NOTE: since some elements are left as future work implementation,
 * many variables extracted from the STK500 commands are left unused,
 * hence the following pragma usage to disable the warning for all this
 * module.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"

#define ntoh_u16(c)                                                         	\
  ((uint16_t)(((uint16_t)(((uint8_t *)(c))[0]) << 8) |                         	\
              (uint16_t)(((uint8_t *)(c))[1])))

#define hton_u16(buffer, i)                                                 	\
  do {                                                                         	\
    ((uint8_t *)(buffer))[0] = ((uint16_t)(i) >> 8) & 0xFF;                    	\
    ((uint8_t *)(buffer))[1] = ((uint16_t)(i)) & 0xFF;                         	\
  } while (0);

#define ntoh_u32(c)                                                            \
  ((uint32_t)(((uint32_t)(((uint8_t *)(c))[0]) << 24) |                        \
              ((uint32_t)(((uint8_t *)(c))[1]) << 16) |                        \
              ((uint32_t)(((uint8_t *)(c))[2]) << 8) |                         \
              (uint32_t)(((uint8_t *)(c))[3])))

#define hton_u32(buffer, i)                                                    \
  do {                                                                         \
    ((uint8_t *)(buffer))[0] = ((uint32_t)(i) >> 24) & 0xFF;                   \
    ((uint8_t *)(buffer))[1] = ((uint32_t)(i) >> 16) & 0xFF;                   \
    ((uint8_t *)(buffer))[2] = ((uint32_t)(i) >> 8) & 0xFF;                    \
    ((uint8_t *)(buffer))[3] = ((uint32_t)(i)) & 0xFF;                         \
  } while (0);


extern void prod_console_write(uint8_t *Buf, uint32_t Len);
int stk500_console_putc(uint8_t c)
{
    console_putc(c);
    return 0;
}

extern void prod_console_read(uint8_t *Buf, uint32_t Len);
int stk500_console_getc(uint8_t *c)
{
    if(c == NULL){
        return -1;
    }
    uint8_t chr = console_getc();
    *c = chr;
    return 0;
}

/* Tnis is a simple and straightforward implementation of 
 * the Atmel STK500 console interface.
 * See http://ww1.microchip.com/downloads/en/Appnotes/doc2591.pdf for
 * more details.
 */

/* Maximum size of 300 bytes */
#define MAX_STK500_SIZE 300
/* STK500 message format */
typedef struct __attribute__((packed)) {
    uint8_t message_start;
    uint8_t sequence_number;
    uint16_t message_size;
    uint8_t token;
    uint8_t message_body[MAX_STK500_SIZE];
    uint8_t checksum;
} stk500_message;


/* Atmel defined constants, mainly stolen from:
 * https://github.com/arduino/Arduino-stk500v2-bootloader/blob/master/command.h
 */
/*****************[ STK message constants ]***************************/
#define MESSAGE_START                       0x1B        //= ESC = 27 decimal
#define TOKEN                               0x0E

/*****************[ STK general command constants ]**************************/

#define CMD_SIGN_ON                         0x01
#define CMD_SET_PARAMETER                   0x02
#define CMD_GET_PARAMETER                   0x03
#define CMD_SET_DEVICE_PARAMETERS           0x04
#define CMD_OSCCAL                          0x05
#define CMD_LOAD_ADDRESS                    0x06
#define CMD_FIRMWARE_UPGRADE                0x07


/*****************[ STK ISP command constants ]******************************/

#define CMD_ENTER_PROGMODE_ISP              0x10
#define CMD_LEAVE_PROGMODE_ISP              0x11
#define CMD_CHIP_ERASE_ISP                  0x12
#define CMD_PROGRAM_FLASH_ISP               0x13
#define CMD_READ_FLASH_ISP                  0x14
#define CMD_PROGRAM_EEPROM_ISP              0x15
#define CMD_READ_EEPROM_ISP                 0x16
#define CMD_PROGRAM_FUSE_ISP                0x17
#define CMD_READ_FUSE_ISP                   0x18
#define CMD_PROGRAM_LOCK_ISP                0x19
#define CMD_READ_LOCK_ISP                   0x1A
#define CMD_READ_SIGNATURE_ISP              0x1B
#define CMD_READ_OSCCAL_ISP                 0x1C
#define CMD_SPI_MULTI                       0x1D

/*****************[ STK PP command constants ]*******************************/

#define CMD_ENTER_PROGMODE_PP               0x20
#define CMD_LEAVE_PROGMODE_PP               0x21
#define CMD_CHIP_ERASE_PP                   0x22
#define CMD_PROGRAM_FLASH_PP                0x23
#define CMD_READ_FLASH_PP                   0x24
#define CMD_PROGRAM_EEPROM_PP               0x25
#define CMD_READ_EEPROM_PP                  0x26
#define CMD_PROGRAM_FUSE_PP                 0x27
#define CMD_READ_FUSE_PP                    0x28
#define CMD_PROGRAM_LOCK_PP                 0x29
#define CMD_READ_LOCK_PP                    0x2A
#define CMD_READ_SIGNATURE_PP               0x2B
#define CMD_READ_OSCCAL_PP                  0x2C    

#define CMD_SET_CONTROL_STACK               0x2D

/*****************[ STK HVSP command constants ]*****************************/

#define CMD_ENTER_PROGMODE_HVSP             0x30
#define CMD_LEAVE_PROGMODE_HVSP             0x31
#define CMD_CHIP_ERASE_HVSP                 0x32
#define CMD_PROGRAM_FLASH_HVSP	            0x33
#define CMD_READ_FLASH_HVSP                 0x34
#define CMD_PROGRAM_EEPROM_HVSP             0x35
#define CMD_READ_EEPROM_HVSP                0x36
#define CMD_PROGRAM_FUSE_HVSP               0x37
#define CMD_READ_FUSE_HVSP                  0x38
#define CMD_PROGRAM_LOCK_HVSP               0x39
#define CMD_READ_LOCK_HVSP                  0x3A
#define CMD_READ_SIGNATURE_HVSP             0x3B
#define CMD_READ_OSCCAL_HVSP                0x3C

/*****************[ STK status constants ]***************************/

/* Success */
#define STATUS_CMD_OK                       0x00

/* Warnings */
#define STATUS_CMD_TOUT                     0x80
#define STATUS_RDY_BSY_TOUT                 0x81
#define STATUS_SET_PARAM_MISSING            0x82

/* Errors */
#define STATUS_CMD_FAILED                   0xC0
#define STATUS_CKSUM_ERROR                  0xC1
#define STATUS_CMD_UNKNOWN                  0xC9

/*****************[ STK parameter constants ]***************************/
#define PARAM_BUILD_NUMBER_LOW              0x80
#define PARAM_BUILD_NUMBER_HIGH             0x81
#define PARAM_HW_VER                        0x90
#define PARAM_SW_MAJOR                      0x91
#define PARAM_SW_MINOR                      0x92
#define PARAM_VTARGET                       0x94
#define PARAM_VADJUST                       0x95
#define PARAM_OSC_PSCALE                    0x96
#define PARAM_OSC_CMATCH                    0x97
#define PARAM_SCK_DURATION                  0x98
#define PARAM_TOPCARD_DETECT                0x9A
#define PARAM_STATUS                        0x9C
#define PARAM_DATA                          0x9D
#define PARAM_RESET_POLARITY                0x9E
#define PARAM_CONTROLLER_INIT               0x9F

/*****************[ STK answer constants ]***************************/

#define ANSWER_CKSUM_ERROR                  0xB0


static int stk500_cmd_sign_on(stk500_message *cmd, stk500_message *resp)
{
    if((cmd ==  NULL) || (resp == NULL)){
        goto err;
    }
    if((cmd->message_size != 1) || (cmd->message_body[0] != CMD_SIGN_ON)){
        goto err;
    }

    /* Response */
    resp->message_size = 11;
    resp->message_body[0] = cmd->message_body[0];
    resp->message_body[1] = STATUS_CMD_OK;
    resp->message_body[2] = 8;
    local_memcpy(&resp->message_body[3], "AVRISP_2", 8);

    return 0;
err:
    return -1;
}

static int stk500_cmd_set_parameter(stk500_message *cmd, stk500_message *resp)
{
    if((cmd ==  NULL) || (resp == NULL)){
        goto err;
    }
    if((cmd->message_size != 3) || (cmd->message_body[0] != CMD_SET_PARAMETER)){
        goto err;
    }
    /* Check the parameter to set */
     
    /* TODO: properly handle setting parameters */

    /* Response */
    resp->message_size = 2;
    resp->message_body[0] = cmd->message_body[0];
    resp->message_body[1] = STATUS_CMD_OK;

    return 0;
err:
    return -1;
}

static int stk500_cmd_get_parameter(stk500_message *cmd, stk500_message *resp)
{
    if((cmd ==  NULL) || (resp == NULL)){
        goto err;
    }
    if((cmd->message_size != 2) || (cmd->message_body[0] != CMD_GET_PARAMETER)){
        goto err;
    }
    /* Check the parameter to set */
     
    /* TODO: properly handle getting parameters */

    /* Response */
    resp->message_size = 3;
    resp->message_body[0] = cmd->message_body[0];
    resp->message_body[1] = STATUS_CMD_OK;
    resp->message_body[2] = 0; /* FIXME */

    return 0;
err:
    return -1;
}

static int stk500_cmd_osccal(stk500_message *cmd, stk500_message *resp)
{
    if((cmd ==  NULL) || (resp == NULL)){
        goto err;
    }
    if((cmd->message_size != 1) || (cmd->message_body[0] != CMD_OSCCAL)){
        goto err;
    }

    /* Response */
    resp->message_size = 2;
    resp->message_body[0] = cmd->message_body[0];
    resp->message_body[1] = STATUS_CMD_OK;

    return 0;
err:
    return -1;
}

static volatile uint32_t current_loaded_address = 0xDEADBEEF;
static int stk500_cmd_load_address(stk500_message *cmd, stk500_message *resp)
{
    if((cmd ==  NULL) || (resp == NULL)){
        goto err;
    }
    if((cmd->message_size != 5) || (cmd->message_body[0] != CMD_LOAD_ADDRESS)){
        goto err;
    }
    uint32_t address = ntoh_u32(&cmd->message_body[1]);
  
    /* Response */
    resp->message_size = 2;
    resp->message_body[0] = cmd->message_body[0];
    /* Address overflow compared to our target? */
    if(address > 0xffff){
        resp->message_body[1] = STATUS_CMD_FAILED;
    }
    else{
        resp->message_body[1] = STATUS_CMD_OK;
        current_loaded_address = address;
    }

    return 0;
err:
    return -1;
}

static int stk500_cmd_firmware_upgrade(stk500_message *cmd, stk500_message *resp)
{
    if((cmd ==  NULL) || (resp == NULL)){
        goto err;
    }
    if((cmd->message_size != 11) || (cmd->message_body[0] != CMD_FIRMWARE_UPGRADE)){
        goto err;
    }
    
    /* We do not support firmware upgrade! (through STK500) */    
  
    /* Response */
    resp->message_size = 2;
    resp->message_body[0] = cmd->message_body[0];
    resp->message_body[1] = STATUS_CMD_FAILED;

    return 0;
err:
    return -1;
}

/* ISP programming commands */
static int stk500_cmd_enter_progmode_isp(stk500_message *cmd, stk500_message *resp)
{
    /* Reset currently loaded address */
    current_loaded_address = 0xDEADBEEF;

    if((cmd ==  NULL) || (resp == NULL)){
        goto err;
    }
    if((cmd->message_size != 12) || (cmd->message_body[0] != CMD_ENTER_PROGMODE_ISP)){
        goto err;
    }
    /* Get the parameters */
    uint8_t timeout = cmd->message_body[1];
    uint8_t stabDelay = cmd->message_body[2];
    uint8_t cmdexeDelay = cmd->message_body[3];
    uint8_t synchLoops = cmd->message_body[4];
    uint8_t byteDelay = cmd->message_body[5];
    uint8_t pollValue = cmd->message_body[6];
    uint8_t pollIndex = cmd->message_body[7];
    uint8_t cmd1 = cmd->message_body[8];
    uint8_t cmd2 = cmd->message_body[9];
    uint8_t cmd3 = cmd->message_body[10];
    uint8_t cmd4 = cmd->message_body[11];

    /* FIXME: handle timeouts in ms */
    resp->message_body[1] = STATUS_CMD_OK;
    /* We only support AVR */
    if(pollValue != 0x53){
        resp->message_body[1] = STATUS_CMD_FAILED;
        goto resp;
    }
    /* check our command */
    if((cmd1 != 0xac) || (cmd2 != 0x53)){
        resp->message_body[1] = STATUS_CMD_FAILED;
        goto resp;
    }
    if(flasher_programming_enable(NULL)){
        resp->message_body[1] = STATUS_CMD_FAILED;
    }

resp:
    /* Response */
    resp->message_size = 2;
    resp->message_body[0] = cmd->message_body[0];

    return 0;
err:
    return -1;
}

static int stk500_cmd_leave_progmode_isp(stk500_message *cmd, stk500_message *resp)
{
    /* Reset currently loaded address */
    current_loaded_address = 0xDEADBEEF;

    if((cmd ==  NULL) || (resp == NULL)){
        goto err;
    }
    if((cmd->message_size != 3) || (cmd->message_body[0] != CMD_LEAVE_PROGMODE_ISP)){
        goto err;
    }

    flasher_leave_programming_mode();

    /* Response */
    resp->message_size = 2;
    resp->message_body[0] = cmd->message_body[0];
    resp->message_body[1] = STATUS_CMD_OK;

    return 0;
err:
    return -1;
}

static int stk500_cmd_chip_erase_isp(stk500_message *cmd, stk500_message *resp)
{
    if((cmd ==  NULL) || (resp == NULL)){
        goto err;
    }
    if((cmd->message_size != 7) || (cmd->message_body[0] != CMD_CHIP_ERASE_ISP)){
        goto err;
    }

    /* Get the parameters */
    uint8_t eraseDelay = cmd->message_body[1];
    uint8_t pollMethod = cmd->message_body[2];
    uint8_t cmd1 = cmd->message_body[3];
    uint8_t cmd2 = cmd->message_body[4];
    uint8_t cmd3 = cmd->message_body[5];
    uint8_t cmd4 = cmd->message_body[6];

    resp->message_body[1] = STATUS_CMD_OK;
    /* Check our command */
    if((cmd1 != 0xac) || ((cmd2 & 0xe0) != 0x80)){
        goto err;
    }
    /* NOTE: we do not support the RDY/BSY command, ignore it */
    /* Execute our chip erase */
    if(flasher_chip_erase()){
        goto err;
    }
    /* Wait for the asked delay */
    platform_flasher_delay_milliseconds(eraseDelay);

    /* Response */
    resp->message_size = 2;
    resp->message_body[0] = cmd->message_body[0];

    return 0;
err:
    return -1;
}

static int stk500_cmd_program_flash_isp(stk500_message *cmd, stk500_message *resp)
{
    if((cmd ==  NULL) || (resp == NULL)){
        goto err;
    }
    if((cmd->message_size < 10) || ((cmd->message_body[0] != CMD_PROGRAM_FLASH_ISP) && (cmd->message_body[0] != CMD_PROGRAM_EEPROM_ISP))){
        goto err;
    }

    /* Get the parameters */
    uint16_t NumBytes = ntoh_u16(&cmd->message_body[1]);
    /* Check for overflow */
    if(NumBytes > (MAX_STK500_SIZE - 10)){
        goto err;
    }
    uint8_t mode  = cmd->message_body[3];
    uint8_t delay = cmd->message_body[4];
    uint8_t cmd1  = cmd->message_body[5];
    uint8_t cmd2  = cmd->message_body[6];
    uint8_t cmd3  = cmd->message_body[7];
    uint8_t poll1 = cmd->message_body[8];
    uint8_t poll2 = cmd->message_body[9];
    uint8_t *data = &cmd->message_body[10];
    
    if(delay == 0){
        /* Override delay if it is zero
         *   - 10 ms is a conservative value for FLASH
         *   - 20 ms is a conservative value for EEPROM
         */
        delay = (cmd->message_body[0] == CMD_PROGRAM_FLASH_ISP) ? 10 : 20;
    }
    /* Response is always two bytes long */
    resp->message_size = 2;
    if(cmd->message_body[0] == CMD_PROGRAM_FLASH_ISP){
        /* Check what we have to do: we only support page mode for flash */
        if(mode & 0x01){
            /* Page mode:  */
            /* cmd1 should be a Load Page and cmd2 a Write Program Memory Page */
            if(cmd1 != 0x40){
                goto err;
            }
            uint16_t write_bytes = 0;
            uint16_t base_address = (uint16_t)(current_loaded_address & 0x1f);
            uint16_t base_address_word = (uint16_t)(current_loaded_address & 0x1f);
            while(write_bytes < NumBytes){
                uint8_t b_type = ((base_address % 2) == 0) ? LOW_BYTE : HIGH_BYTE;
                if(flasher_load_program_memory_page(base_address_word, b_type, data[write_bytes])){
                    goto err;
                }
                base_address++;
                if((base_address % 2) == 0){
                    base_address_word++;
                }
                write_bytes++;
            }
            /* Do we have a write page bit set? */
            if(mode & 0x80){
                if(cmd2 != 0x4c){
                    goto err;
                }
                if(flasher_write_program_memory_page((uint16_t)(current_loaded_address >> 5))){
                    goto err;
                }
            }
            /* What do we do now? */
            if(mode & (0x1 << 4)){
                /* Timed delay selected: wait the delay for termination */
                platform_flasher_delay_milliseconds(delay);                 
                resp->message_body[1] = STATUS_CMD_OK;
                goto resp;
            }
            else if(mode & (0x1 << 6)){
                /* We do not support RDY/BSY polling */
                resp->message_body[1] = STATUS_RDY_BSY_TOUT;
                goto resp;
            }
            else if(mode & (0x1 << 5)){
                /* TODO: handle the value polling mode.
                 * For now, wait for the asked delay.
                 */
                platform_flasher_delay_milliseconds(delay);                 
                resp->message_body[1] = STATUS_CMD_OK;
                goto resp;
            }
        }
        else{
            goto err;
        }
    }
    else{
        /* EEPROM programming mode: we only support word mode */
        if(mode & 0x01){
            goto err;
        }
        else{
            /* cmd1 should be a Write EEPROM memory */
            if(cmd1 != 0xc0){
                goto err;
            }
            uint16_t write_bytes = 0;
            uint16_t base_address = (uint16_t)(current_loaded_address & 0x1ff);
            while(write_bytes < NumBytes){
                if(flasher_write_eeprom_memory(base_address, data[write_bytes])){
                    goto err;
                }
                base_address++;
                write_bytes++;
            }
            /* What do we do now? */
            if(mode & (0x1 << 1)){
                /* Timed delay selected: wait the delay for termination */
                platform_flasher_delay_milliseconds(delay);
                resp->message_body[1] = STATUS_CMD_OK;
                goto resp;
            }
            else if(mode & (0x1 << 3)){
                /* We do not support RDY/BSY polling */
                resp->message_body[1] = STATUS_RDY_BSY_TOUT;
                goto resp;
            }
            else if(mode & (0x1 << 2)){
                /* TODO: handle the value polling mode.
                 * For now, wait for the asked delay.
                 */
                platform_flasher_delay_milliseconds(delay);                 
                resp->message_body[1] = STATUS_CMD_OK;
                goto resp;
            }
        }
    }

resp:
    /* Response */
    resp->message_size = 2;
    resp->message_body[0] = cmd->message_body[0];

    return 0;
err:
    return -1;
}

static int stk500_cmd_read_flash_isp(stk500_message *cmd, stk500_message *resp)
{
    if((cmd ==  NULL) || (resp == NULL)){
        goto err;
    }
    if((cmd->message_size != 4) || ((cmd->message_body[0] != CMD_READ_FLASH_ISP) && (cmd->message_body[0] != CMD_READ_EEPROM_ISP))){
        goto err;
    }

    /* Get the parameters */
    uint16_t NumBytes = ntoh_u16(&cmd->message_body[1]);
    /* Check for overflow */
    if(NumBytes > (MAX_STK500_SIZE - 3)){
        resp->message_body[1] = STATUS_CMD_FAILED;
        resp->message_size = 2;
        goto resp;      
    }
    uint8_t cmd1  = cmd->message_body[3];
    /* This should be a Read Memory command */
    if((cmd->message_body[0] == CMD_READ_FLASH_ISP) && (cmd1 != 0x20)){
        resp->message_body[1] = STATUS_CMD_FAILED;
        resp->message_size = 2;
        goto resp;
    }
    if((cmd->message_body[0] == CMD_READ_EEPROM_ISP) && (cmd1 != 0xa0)){
        resp->message_body[1] = STATUS_CMD_FAILED;
        resp->message_size = 2;
        goto resp;
    }
    
    /* Check that we have already loaded an address */
    if(current_loaded_address == 0xDEADBEEF){
        resp->message_body[1] = STATUS_CMD_FAILED;
        resp->message_size = 2;
        goto resp;
    }
    /* Now go and read NumBytes in flash */
    uint16_t read_bytes = 0;
    uint16_t base_address = (uint16_t)current_loaded_address;
    uint16_t base_address_word = (uint16_t)current_loaded_address;
    /* Status1 */
    resp->message_body[1] = STATUS_CMD_OK;
    while(read_bytes < NumBytes){
        uint8_t b;
        uint8_t b_type = ((base_address % 2) == 0) ? LOW_BYTE : HIGH_BYTE;
        int ret = (cmd->message_body[0] == CMD_READ_FLASH_ISP) ? flasher_read_program_memory_byte(base_address_word, b_type, &b) : flasher_read_eeprom_memory(base_address, &b);
        if(ret){
            resp->message_body[1] = STATUS_CMD_FAILED;
            resp->message_size = 2;
            goto resp;
        }
        resp->message_body[2 + read_bytes] = b;
        base_address++;
        if((base_address % 2) == 0){
            base_address_word++;
        }
        read_bytes++;
    }
    /* Data size */
    resp->message_size = 3 + NumBytes;
    /* Status2 */
    resp->message_body[2 + NumBytes] = STATUS_CMD_OK;

resp:
    /* Response */
    resp->message_body[0] = cmd->message_body[0];

    return 0;
err:
    return -1;
}


static int stk500_cmd_read_fuse_isp(stk500_message *cmd, stk500_message *resp)
{
    if((cmd ==  NULL) || (resp == NULL)){
        goto err;
    }
    if((cmd->message_size != 6) || ((cmd->message_body[0] != CMD_READ_FUSE_ISP) && (cmd->message_body[0] != CMD_READ_LOCK_ISP) && (cmd->message_body[0] != CMD_READ_SIGNATURE_ISP) && (cmd->message_body[0] != CMD_READ_OSCCAL_ISP))){
        goto err;
    }

    /* Get the parameters */
    uint8_t RetAddr = cmd->message_body[1];
    uint8_t command[4], response[4];   
    command[0]  = cmd->message_body[2];
    command[1]  = cmd->message_body[3];
    command[2]  = cmd->message_body[4];
    command[3]  = cmd->message_body[5];
    if(send_flasher_command(command, response)){
        goto err;       
    }
    /* Response */
    resp->message_size = 4;
    resp->message_body[0] = cmd->message_body[0];
    resp->message_body[1] = STATUS_CMD_OK;
    resp->message_body[2] = response[3];
    resp->message_body[3] = STATUS_CMD_OK;

    return 0;
err:
    return -1;
}

static int stk500_cmd_program_fuse_isp(stk500_message *cmd, stk500_message *resp)
{
    if((cmd ==  NULL) || (resp == NULL)){
        goto err;
    }
    if((cmd->message_size != 5) || ((cmd->message_body[0] != CMD_PROGRAM_FUSE_ISP) && (cmd->message_body[0] != CMD_PROGRAM_LOCK_ISP))){
        goto err;
    }

    /* Get the parameters */
    uint8_t command[4], response[4];   
    command[0]  = cmd->message_body[1];
    command[1]  = cmd->message_body[2];
    command[2]  = cmd->message_body[3];
    command[3]  = cmd->message_body[4];
    if(send_flasher_command(command, response)){
        goto err;       
    }
    /* Response */
    resp->message_size = 3;
    resp->message_body[0] = cmd->message_body[0];
    resp->message_body[1] = STATUS_CMD_OK;
    resp->message_body[2] = STATUS_CMD_OK;

    return 0;
err:
    return -1;
}

static int stk500_cmd_spi_multi(stk500_message *cmd, stk500_message *resp)
{
    if((cmd ==  NULL) || (resp == NULL)){
        goto err;
    }
    if((cmd->message_size < 3) || (cmd->message_body[0] != CMD_SPI_MULTI)){
        goto err;
    }

    /* Get the parameters */
    uint8_t command[4], response[4];
    uint8_t NumTx = cmd->message_body[1];
    uint8_t NumRx = cmd->message_body[2];
    uint8_t RxStartAddr = cmd->message_body[3];
    /* Sanity checks */
    if(NumTx > (MAX_STK500_SIZE - 4)){
        goto err;
    }
    if(NumRx > (MAX_STK500_SIZE - 3)){
        goto err;
    }
    unsigned int i;
    uint8_t received = 0;
    for(i = 0; i < NumTx; i++){
        uint8_t *b;
        uint8_t dummy;
        if((i >= RxStartAddr) && (received <= NumRx)){
            b = &resp->message_body[2 + received];
            received++;
        }
        else{
            b = &dummy;
        }
        if(send_flasher_spi_generic(cmd->message_body[4 + i], b)){
            goto err;
        }
    }
    /* Pad the Rx buffer with zeroes if necessary */
    for(i = received; i < NumRx; i++){
        resp->message_body[2 + i] = 0;
    }
    /* Response */
    resp->message_size = 3 + NumRx;
    resp->message_body[0] = cmd->message_body[0];
    resp->message_body[1] = STATUS_CMD_OK;
    resp->message_body[2 + NumRx] = STATUS_CMD_OK;

    return 0;
err:
    return -1;
}



/*** Main loop for STK500 commands ***/
static int stk500_receive_cmd(stk500_message *cmd)
{
    uint8_t checksum = 0;
    /* Get start byte */
    if(stk500_console_getc(&cmd->message_start)){
        goto err;
    }
    checksum ^= cmd->message_start;
    if(cmd->message_start != MESSAGE_START){
        goto err;
    }
    /* Get sequence number */
    if(stk500_console_getc(&cmd->sequence_number)){
        goto err;
    }
    checksum ^= cmd->sequence_number;
    /* Get size */
    uint8_t size[2];
    if(stk500_console_getc(&size[0])){
        goto err;
    }
    if(stk500_console_getc(&size[1])){
        goto err;
    }
    cmd->message_size = ntoh_u16(size);
    if(cmd->message_size > MAX_STK500_SIZE){
        /* Overflow */
        goto err;
    }
    checksum ^= (size[0] ^ size[1]);
    /* Get token */
    if((stk500_console_getc(&cmd->token)) || (cmd->token != TOKEN)){
        goto err;
    }
    checksum ^= cmd->token;
    /* Get message body */
    unsigned int i;
    for(i = 0; i < cmd->message_size; i++){
        if(stk500_console_getc(&cmd->message_body[i])){
            goto err;
        }
        checksum ^= cmd->message_body[i];
    }
    /* Get checksum */
    if(stk500_console_getc(&cmd->checksum)){
        goto err;
    }
    /* Check checksum */
    if(cmd->checksum != checksum){
        goto err;
    }

    return 0;
err:
    return -1;
}


static int stk500_send_resp(stk500_message *resp)
{
    uint8_t checksum = 0;
    /* Send start byte */
    resp->message_start = MESSAGE_START;
    if(stk500_console_putc(resp->message_start)){
        goto err;
    }
    checksum ^= resp->message_start;
    /* Send sequence number */
    if(stk500_console_putc(resp->sequence_number)){
        goto err;
    }
    checksum ^= resp->sequence_number;
    /* Send size */
    if(resp->message_size > MAX_STK500_SIZE){
        /* Overflow */
        goto err;
    }
    uint8_t size[2];
    hton_u16(size, resp->message_size);    
    if(stk500_console_putc(size[0])){
        goto err;
    }
    if(stk500_console_putc(size[1])){
        goto err;
    }
    checksum ^= (size[0] ^ size[1]);
    /* Send token */
    resp->token = TOKEN;
    if(stk500_console_putc(resp->token)){
        goto err;
    }
    checksum ^= resp->token;
    /* Send data */
    unsigned int i;
    for(i = 0; i < resp->message_size; i++){
        if(stk500_console_putc(resp->message_body[i])){
            goto err;
        }
        checksum ^= resp->message_body[i];
    }
    /* Send checksum */
    resp->checksum = checksum;
    if(stk500_console_putc(resp->checksum)){
        goto err;
    }
   
    return 0;
err:
    return -1;

}

int stk500_parse_cmd(void)
{
    stk500_message cmd, resp;

    while(1){
        /* Wait for an incoming command */
        if(stk500_receive_cmd(&cmd)){
            goto err;
        }

        /* Blink our activity LED */
        led_error_on();
        HAL_Delay(10);
        led_error_off();
        //log_printf("=> Received STK500 command %x, size %d, %x %x %x\r\n", cmd.message_body[0], cmd.message_size, cmd.message_body[1], cmd.message_body[2], cmd.message_body[3]);

        if(cmd.message_size < 1){
            /* Bad command ... */
            continue;
        }
        switch(cmd.message_body[0]){
            case CMD_SIGN_ON:{
                if(stk500_cmd_sign_on(&cmd, &resp)){
                    continue;
                }
                break;
            }
            case CMD_SET_PARAMETER:{
                if(stk500_cmd_set_parameter(&cmd, &resp)){
                    continue;
                }
                break;
            }
            case CMD_GET_PARAMETER:{
                if(stk500_cmd_get_parameter(&cmd, &resp)){
                    continue;
                }
                break;
            }
            case CMD_OSCCAL:{
                if(stk500_cmd_osccal(&cmd, &resp)){
                    continue;
                }
                break;
            }
            case CMD_LOAD_ADDRESS:{
                if(stk500_cmd_load_address(&cmd, &resp)){
                    continue;
                }
                break;
            }
            case CMD_FIRMWARE_UPGRADE:{
                if(stk500_cmd_firmware_upgrade(&cmd, &resp)){
                    continue;
                }
                break;
            }
            case CMD_ENTER_PROGMODE_ISP:{
                if(stk500_cmd_enter_progmode_isp(&cmd, &resp)){
                    continue;
                }
                break;
            }
            case CMD_LEAVE_PROGMODE_ISP:{
                if(stk500_cmd_leave_progmode_isp(&cmd, &resp)){
                    continue;
                }
                break;
            }
            case CMD_CHIP_ERASE_ISP:{
                if(stk500_cmd_chip_erase_isp(&cmd, &resp)){
                    continue;
                }
                break;
            }
            case CMD_PROGRAM_FLASH_ISP:
            case CMD_PROGRAM_EEPROM_ISP:{
                if(stk500_cmd_program_flash_isp(&cmd, &resp)){
                    continue;
                }
                break;
            }
            case CMD_READ_FLASH_ISP:
            case CMD_READ_EEPROM_ISP:{
                if(stk500_cmd_read_flash_isp(&cmd, &resp)){
                    continue;
                }
                break;
            }
            case CMD_READ_FUSE_ISP:
            case CMD_READ_LOCK_ISP:
            case CMD_READ_SIGNATURE_ISP:
            case CMD_READ_OSCCAL_ISP:{
                if(stk500_cmd_read_fuse_isp(&cmd, &resp)){
                    continue;
                }
                break;
            }
            case CMD_PROGRAM_FUSE_ISP:
            case CMD_PROGRAM_LOCK_ISP:{
                if(stk500_cmd_program_fuse_isp(&cmd, &resp)){
                    continue;
                }
                break;
            }
            case CMD_SPI_MULTI:{
                if(stk500_cmd_spi_multi(&cmd, &resp)){
                    continue;
                }
                break;
            }

            default:
               /* Unexpected command */
               continue;
        }       
        /* Use the same sequence number for the response */
        resp.sequence_number = cmd.sequence_number;
        if(stk500_send_resp(&resp)){
            goto err;
        }
    }

    return 0;
err:
    return -1;
}

#pragma GCC diagnostic pop
