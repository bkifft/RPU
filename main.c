//based on https://github.com/jncronin/rpi-boot/blob/master/emmc.c by John Cronin <jncronin@tysos.org>
//tweaked to be run from userland by bkifft @GBAtemp

/* 
* Copyright (C) 2013 by John Cronin <jncronin@tysos.org>
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:

* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.

* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*/


#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <ctype.h>
#include "defines.h" //contains all the adresses, offsets, command structures and bitmasks
#include "util.h"

#define TIMEOUT_WAIT(stop_if_true, usec)                 \
do {                                                        \
        uint32_t count = 0;        \
        do                                                \
        {                                          \
                ++count; \
                usleep(1000);                                       \
                if(stop_if_true)                        \
                        break;                                \
        } while(count< (usec/1000));                        \
} while(0);

#define BLOCKSIZE       (0x1000000)
#define CHECKBIT(x, y)  ((x & (1 << y)) != 0) ? 1 : 0
#define INP_GPIO(g)   *(gpio + ((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g)   *(gpio + ((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio + (((g)/10))) |= (((a)<=3?(a) + 4:(a)==4?3:2)<<(((g)%10)*3))
#define GPIO_SET  *(gpio + 7)  
#define GPIO_CLR  *(gpio + 10) 
#define GPIO_READ(g)  *(gpio + 13) &= (1<<(g))

uint32_t* emmc;

int sd_read(struct block_device *, uint8_t *, size_t buf_size, uint32_t, char);
int sd_write(struct block_device *, uint8_t *, size_t buf_size, uint32_t, char);


void print_arg1_reg(uint32_t* emmc){
 uint32_t arg1 =  *(emmc + EMMC_ARG1/4);//div 4 as the offsets are in bytes
 printf("ARG1:-----------------------------------------\n");
 printf("ARG1: 			%#08X\n", arg1); 
}

void print_resp1_reg(uint32_t* emmc){
 uint32_t resp1 =  *(emmc + EMMC_RESP1/4);//div 4 as the offsets are in bytes
 printf("RESP1:-----------------------------------------\n");
 printf("RESP1:             %#08X\n", resp1); 
}

void print_resp2_reg(uint32_t* emmc){
 uint32_t resp2 =  *(emmc + EMMC_RESP2/4);//div 4 as the offsets are in bytes
 printf("RESP2:-----------------------------------------\n");
 printf("RESP2:             %#08X\n", resp2); 
}


void print_resp3_reg(uint32_t* emmc){
 uint32_t resp3 =  *(emmc + EMMC_RESP3/4);//div 4 as the offsets are in bytes
 printf("RESP1:-----------------------------------------\n");
 printf("RESP1: 			%#08X\n", resp3); 
}

void print_cmdtm_reg(uint32_t* emmc){
 uint32_t cmdtm =  *(emmc + EMMC_CMDTM/4);//div 4 as the offsets are in bytes
 printf("CMDTM:-----------------------------------------\n");
 printf("CMD_INDEX:         %i\n", ((cmdtm & 0x3F000000)>>24));
 printf("CMD_TYPE:          %i%i\n", CHECKBIT(cmdtm, 23),CHECKBIT(cmdtm, 22));  
 printf("CMD_ISDATA:        %i\n", CHECKBIT(cmdtm, 21));  
 printf("CMD_IXCHK_EN:      %i\n", CHECKBIT(cmdtm, 20));  
 printf("CMD_CRCCHK_EN:     %i\n", CHECKBIT(cmdtm, 19));
 printf("CMD_RSPNS_TYPE:    %i%i\n", CHECKBIT(cmdtm, 17),CHECKBIT(cmdtm, 16));
 printf("TM_MULTI_BLOCK:    %i\n", CHECKBIT(cmdtm, 5));
 printf("TM_DAT_DIR:        %i\n", CHECKBIT(cmdtm, 4));  
 printf("TM_AUTO_CMD_EN:    %i%i\n", CHECKBIT(cmdtm, 3),CHECKBIT(cmdtm, 2));
 printf("TM_BLKCNT_T:       %i\n", CHECKBIT(cmdtm, 1));  
 
 
}

void print_control0_reg(uint32_t* emmc){
 uint32_t  ctrl0 =  (*(emmc + EMMC_CONTROL0/4)); //div 4 as the offsets are in bytes
 printf("CONTROL0:--------------------------------------\n");
 printf("ALT_BOOT_EN:   %i\n",          CHECKBIT(ctrl0, 22));
 printf("BOOT_EN:       %i\n",          CHECKBIT(ctrl0, 21));
 printf("SPI_MODE:      %i\n",          CHECKBIT(ctrl0, 20));
 printf("GAP_IEN:       %i\n",          CHECKBIT(ctrl0, 19));
 printf("READWAIT_EN:   %i\n",          CHECKBIT(ctrl0, 18));
 printf("GAP_RESTART:   %i\n",			CHECKBIT(ctrl0, 17));
 printf("GAP_STOP:      %i\n",          CHECKBIT(ctrl0, 16));
 printf("HCTL_8BIT:     %i\n",          CHECKBIT(ctrl0, 5));
 printf("HCTL_HS_EN:    %i\n",          CHECKBIT(ctrl0, 2));
 printf("HCTL_DWIDTH:   %i\n",          CHECKBIT(ctrl0, 1));
}

void print_control1_reg(uint32_t* emmc){
 uint32_t  ctrl1 =  (*(emmc + EMMC_CONTROL1/4)); //div 4 as the offsets are in bytes
 printf("CONTROL1:--------------------------------------\n");
 printf("SRST_DATA:     %i\n",          CHECKBIT(ctrl1, 26));
 printf("SRST_CMD:      %i\n",          CHECKBIT(ctrl1, 25));
 printf("SRST_HC:       %i\n",          CHECKBIT(ctrl1, 24));
 printf("DATA_TOUNIT:   %i\n",          ((ctrl1 & 0x0F0000)>>16));
 printf("CLK_FREQ8:     %i\n",          ((ctrl1 & 0xFF00)>>8));  //FIXME: should be combined with next value
 printf("CLK_FREQ_MS2:  %i\n",			((ctrl1 & 0xC0)>>6));
 printf("CLK_GENSEL:    %i\n",          CHECKBIT(ctrl1, 5));
 printf("CLK_EN:        %i\n",          CHECKBIT(ctrl1, 2));
 printf("CLK_STABLE:    %i\n",          CHECKBIT(ctrl1, 1));
 printf("CLK_INTLEN:    %i\n",          CHECKBIT(ctrl1, 0));
}

void print_control2_reg(uint32_t* emmc){
 uint32_t  ctrl2 =  (*(emmc + EMMC_CONTROL1/4)); //div 4 as the offsets are in bytes
 printf("CONTROL2:--------------------------------------\n");
 printf("TUNED:         %i\n",          CHECKBIT(ctrl2, 23));
 printf("TUNEON:        %i\n",          CHECKBIT(ctrl2, 22));
 printf("UHSMODE:       %i%i%i\n",      CHECKBIT(ctrl2, 18),CHECKBIT(ctrl2, 17),CHECKBIT(ctrl2, 16));
 printf("NOTC12_ERR :   %i\n",          CHECKBIT(ctrl2, 7));
 printf("ACBAD_ERR:     %i\n",          CHECKBIT(ctrl2, 4));  //FIXME: should be combined with next value
 printf("ACEND_ERR:     %i\n",			CHECKBIT(ctrl2, 3));
 printf("ACCRC_ERR      %i\n",          CHECKBIT(ctrl2, 2));
 printf("ACTO_ERR:      %i\n",          CHECKBIT(ctrl2, 1));
 printf("ACNOX_ERR:     %i\n",          CHECKBIT(ctrl2, 0));
}

void print_blksizecnt_reg(uint32_t* emmc){
 uint32_t  blksizecnt =  (*(emmc + EMMC_BLKSIZECNT/4)); //div 4 as the offsets are in bytes 
 printf("BLK_CNT:	%i\n",          ((blksizecnt & 0xFFFF0000)>>16));
 printf("BLKSIZE:	%i\n",          (blksizecnt & 0x1FF));

}

void print_response_reg(uint32_t* emmc){
 printf("RESPONSE:-----------------------------------\n");
 printf("RESP3: 0x%08X\n", *(emmc + EMMC_RESP3/4));
 printf("RESP2: 0x%08X\n", *(emmc + EMMC_RESP2/4));
 printf("RESP1: 0x%08X\n", *(emmc + EMMC_RESP1/4));
 printf("RESP0: 0x%08X\n", *(emmc + EMMC_RESP0/4));
}
 
void clear_response_reg(uint32_t* emmc ){
 *(emmc + EMMC_RESP0/4) = 0x0; //clear the response 
 *(emmc + EMMC_RESP1/4) = 0x0; //"
 *(emmc + EMMC_RESP2/4) = 0x0; //"
 *(emmc + EMMC_RESP3/4) = 0x0; //"
}
 

void print_status_reg(uint32_t* emmc){
 uint32_t  stat =  (*(emmc + EMMC_STATUS/4)); //div 4 as the offsets are in bytes
 printf("STATUS:--------------------------------------\n");
 printf("DAT7:          %i\n",          CHECKBIT(stat,28));
 printf("DAT6:          %i\n",          CHECKBIT(stat,27));
 printf("DAT5:          %i\n",          CHECKBIT(stat,26));
 printf("DAT4:          %i\n",          CHECKBIT(stat,25));
 printf("CMD_LEVEL:     %i\n",          CHECKBIT(stat,24));

 printf("DAT3:          %i\n",          CHECKBIT(stat,23));
 printf("DAT2:          %i\n",          CHECKBIT(stat,22));
 printf("DAT1:          %i\n",          CHECKBIT(stat,21));
 printf("DAT0:          %i\n",          CHECKBIT(stat,20));
 printf("ALT_DAT_RDY:   %i\n",          CHECKBIT(stat,11));
 printf("R_TRAN:        %i\n",          CHECKBIT(stat,9));
 printf("W_TRAN:        %i\n",          CHECKBIT(stat,8));

 printf("D_ACT:         %i\n",          CHECKBIT(stat,2));
 printf("D_INH:         %i\n",          CHECKBIT(stat,1));
 printf("CMD_INH:       %i\n",          CHECKBIT(stat,0));

}


void print_interrupt_reg(uint32_t* emmc){
 uint32_t  interrupt =  (*(emmc + EMMC_INTERRUPT/4)); //div 4 as the offsets are in bytes
 printf("INTERRUPT:--------------------------------------\n");
 printf("ACMD_ERR:       %i\n",          CHECKBIT(interrupt,24));
 printf("DEND_ERR:       %i\n",          CHECKBIT(interrupt,22));
 printf("DCRC_ERR:       %i\n",          CHECKBIT(interrupt,21));
 printf("DTO_ERR:        %i\n",          CHECKBIT(interrupt,20));
 printf("CBAD_ERR:       %i\n",          CHECKBIT(interrupt,19));
 printf("CEND_ERR:       %i\n",          CHECKBIT(interrupt,18));
 printf("CCRC_ERR:       %i\n",          CHECKBIT(interrupt,17));
 printf("CTO_ERR:        %i\n",          CHECKBIT(interrupt,16));
 printf("ERR:            %i\n",          CHECKBIT(interrupt,15));
 printf("ENDBOOT:        %i\n",          CHECKBIT(interrupt,14));
 printf("BOOTACK:        %i\n",          CHECKBIT(interrupt,13));
 printf("RETUNE:         %i\n",          CHECKBIT(interrupt,12));
 printf("CARD:           %i\n",          CHECKBIT(interrupt,8));
 printf("READ_RDY:       %i\n",          CHECKBIT(interrupt,5));
 printf("WRITE_RDY:      %i\n",          CHECKBIT(interrupt,4));
 printf("BLOCK_GATE:     %i\n",          CHECKBIT(interrupt,2));
 printf("DATA_DONE:      %i\n",          CHECKBIT(interrupt,1));
 printf("CMD_DONE:       %i\n",          CHECKBIT(interrupt,0));
}

static char *sd_versions[] = { "unknown", "1.0 and 1.01", "1.10",
    "2.00", "3.0x", "4.xx" };

void mmio_write(void* reg, uint32_t data)
{
 
        *(volatile uint32_t *)(reg) = data;
 
}

uint32_t mmio_read(void* reg)
{
 
        return *(volatile uint32_t *)(reg);
 
}

static void sd_power_off()
{
        /* Power off the SD card */
        uint32_t control0 = mmio_read(EMMC_BASE + EMMC_CONTROL0);
        control0 &= ~(1 << 8);        // Set SD Bus Power bit off in Power Control Register
        mmio_write(EMMC_BASE + EMMC_CONTROL0, control0);
}

static void spi_mode_on()
{
        uint32_t control0 = mmio_read(EMMC_BASE + EMMC_CONTROL0);
        control0 &= 1 << 20;        // spi bit
        mmio_write(EMMC_BASE + EMMC_CONTROL0, control0);
		printf("SPI %s\n", (CHECKBIT(control0, 20) == 1) ? "on":"off");
}

static uint32_t sd_get_base_clock_hz(){
 return 250000000;
}

static int bcm_2708_power_off(){
 return -1;
}

static int bcm_2708_power_on(){
 return -1;
}

static int bcm_2708_power_cycle()
{
        if(bcm_2708_power_off() < 0)
                return -1;

        usleep(5000);

        return bcm_2708_power_on();
}
 
static uint32_t sd_get_clock_divider(uint32_t base_clock, uint32_t target_rate)
{
    // TODO: implement use of preset value registers

    uint32_t targetted_divisor = 0;
    if(target_rate > base_clock)
        targetted_divisor = 1;
    else
    {
        targetted_divisor = base_clock / target_rate;
        uint32_t mod = base_clock % target_rate;
        if(mod)
            targetted_divisor--;
    }

    // Decide on the clock mode to use

    // Currently only 10-bit divided clock mode is supported

    if(1)//was a check for host controller interface version
    {
        // HCI version 3 or greater supports 10-bit divided clock mode
        // This requires a power-of-two divider

        // Find the first bit set
        int divisor = -1;
        for(int first_bit = 31; first_bit >= 0; first_bit--)
        {
            uint32_t bit_test = (1 << first_bit);
            if(targetted_divisor & bit_test)
            {
                divisor = first_bit;
                targetted_divisor &= ~bit_test;
                if(targetted_divisor)
                {
                    // The divisor is not a power-of-two, increase it
                    divisor++;
                }
                break;
            }
        }

        if(divisor == -1)
            divisor = 31;
        if(divisor >= 32)
            divisor = 31;

        if(divisor != 0)
            divisor = (1 << (divisor - 1));

        if(divisor >= 0x400)
            divisor = 0x3ff;

        uint32_t freq_select = divisor & 0xff;
        uint32_t upper_bits = (divisor >> 8) & 0x3;
        uint32_t ret = (freq_select << 8) | (upper_bits << 6) | (0 << 5);

        int denominator = 1;
        if(divisor != 0)
            denominator = divisor * 2;
        int actual_clock = base_clock / denominator;
        printf("EMMC: base_clock: %i, target_rate: %i, divisor: %08x, "
               "actual_clock: %i, ret: %08x\n", base_clock, target_rate,
               divisor, actual_clock, ret);


        return ret;
    }
    else
    {
        printf("EMMC: unsupported host version\n");
        return SD_GET_CLOCK_DIVIDER_FAIL;
    }

}

// Switch the clock rate whilst running
static int sd_switch_clock_rate(uint32_t base_clock, uint32_t target_rate)
{
    // Decide on an appropriate divider
    uint32_t divider = sd_get_clock_divider(base_clock, target_rate);
    if(divider == SD_GET_CLOCK_DIVIDER_FAIL)
    {
        printf("EMMC: couldn't get a valid divider for target rate %i Hz\n",
               target_rate);
        return -1;
    }

    // Wait for the command inhibit (CMD and DAT) bits to clear
    while(mmio_read(EMMC_BASE + EMMC_STATUS) & 0x3)
        usleep(1000);

    // Set the SD clock off
    uint32_t control1 = mmio_read(EMMC_BASE + EMMC_CONTROL1);
    control1 &= ~(1 << 2);
    mmio_write(EMMC_BASE + EMMC_CONTROL1, control1);
    usleep(2000);

    // Write the new divider
        control1 &= ~0xffe0;                // Clear old setting + clock generator select
    control1 |= divider;
    mmio_write(EMMC_BASE + EMMC_CONTROL1, control1);
    usleep(2000);

    // Enable the SD clock
    control1 |= (1 << 2);
    mmio_write(EMMC_BASE + EMMC_CONTROL1, control1);
    usleep(2000);

    printf("EMMC: successfully set clock rate to %i Hz\n", target_rate);
    return 0;
}

// Reset the CMD line
static int sd_reset_cmd()
{
    uint32_t control1 = mmio_read(EMMC_BASE + EMMC_CONTROL1);
        control1 |= SD_RESET_CMD;
        mmio_write(EMMC_BASE + EMMC_CONTROL1, control1);
        TIMEOUT_WAIT((mmio_read(EMMC_BASE + EMMC_CONTROL1) & SD_RESET_CMD) == 0, 1000000);
        if((mmio_read(EMMC_BASE + EMMC_CONTROL1) & SD_RESET_CMD) != 0)
        {
                printf("EMMC: CMD line did not reset properly\n");
                return -1;
        }
        return 0;
}

// Reset the CMD line
static int sd_reset_dat()
{
    uint32_t control1 = mmio_read(EMMC_BASE + EMMC_CONTROL1);
        control1 |= SD_RESET_DAT;
        mmio_write(EMMC_BASE + EMMC_CONTROL1, control1);
        TIMEOUT_WAIT((mmio_read(EMMC_BASE + EMMC_CONTROL1) & SD_RESET_DAT) == 0, 1000000);
        if((mmio_read(EMMC_BASE + EMMC_CONTROL1) & SD_RESET_DAT) != 0)
        {
                printf("EMMC: DAT line did not reset properly\n");
                return -1;
        }
        return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
static void sd_issue_command_int(struct emmc_block_dev *dev, uint32_t cmd_reg, uint32_t argument, useconds_t timeout)
{
    dev->last_cmd_reg = cmd_reg;
    dev->last_cmd_success = 0;

    // This is as per HCSS 3.7.1.1/3.7.2.2

    // Check Command Inhibit
    while(mmio_read(EMMC_BASE + EMMC_STATUS) & 0x1)
        usleep(1000);

    // Is the command with busy?
    if((cmd_reg & SD_CMD_RSPNS_TYPE_MASK) == SD_CMD_RSPNS_TYPE_48B)
    {
        // With busy

        // Is is an abort command?
        if((cmd_reg & SD_CMD_TYPE_MASK) != SD_CMD_TYPE_ABORT)
        {
            // Not an abort command

            // Wait for the data line to be free
            while(mmio_read(EMMC_BASE + EMMC_STATUS) & 0x2)
                usleep(1000);
        }
    }

    // Set block size and block count
    // For now, block size = 512 bytes, block count = 1,
    if(dev->blocks_to_transfer > 0xffff)
    {
        printf("SD: blocks_to_transfer too great (%i)\n",
               dev->blocks_to_transfer);
        dev->last_cmd_success = 0;
        return;
    }
    uint32_t blksizecnt = dev->block_size | (dev->blocks_to_transfer << 16);
    mmio_write(EMMC_BASE + EMMC_BLKSIZECNT, blksizecnt);

    // Set argument 1 reg
    mmio_write(EMMC_BASE + EMMC_ARG1, argument);

    // Set command reg
    mmio_write(EMMC_BASE + EMMC_CMDTM, cmd_reg);

    usleep(2000);

    // Wait for command complete interrupt
    TIMEOUT_WAIT(mmio_read(EMMC_BASE + EMMC_INTERRUPT) & 0x8001, timeout);
    uint32_t irpts = mmio_read(EMMC_BASE + EMMC_INTERRUPT);

    

    // Test for errors
    if((irpts & 0xffff0001) != 0x1)
    {

        printf("SD: error occured whilst waiting for command complete interrupt\n");
		print_interrupt_reg(emmc);
        dev->last_error = irpts & 0xffff0000;
        dev->last_interrupt = irpts;
        return;
    }

// Clear command complete status
    mmio_write(EMMC_BASE + EMMC_INTERRUPT, 0xffff0001);
    usleep(2000);

    // Get response data
    switch(cmd_reg & SD_CMD_RSPNS_TYPE_MASK)
    {
        case SD_CMD_RSPNS_TYPE_48:
        case SD_CMD_RSPNS_TYPE_48B:
            dev->last_r0 = mmio_read(EMMC_BASE + EMMC_RESP0);
            break;

        case SD_CMD_RSPNS_TYPE_136:
            dev->last_r0 = mmio_read(EMMC_BASE + EMMC_RESP0);
            dev->last_r1 = mmio_read(EMMC_BASE + EMMC_RESP1);
            dev->last_r2 = mmio_read(EMMC_BASE + EMMC_RESP2);
            dev->last_r3 = mmio_read(EMMC_BASE + EMMC_RESP3);
            break;
    }

    // If with data, wait for the appropriate interrupt
    if((cmd_reg & SD_CMD_ISDATA))
    {
        uint32_t wr_irpt;
        int is_write = 0;
        if(cmd_reg & SD_CMD_DAT_DIR_CH)
            wr_irpt = (1 << 5); // read
        else
        {
            is_write = 1;
            wr_irpt = (1 << 4); // write
        }

        int cur_block = 0;
        uint32_t *cur_buf_addr = (uint32_t *)dev->buf;
        while(cur_block < dev->blocks_to_transfer)
        {

                        if(dev->blocks_to_transfer > 1)
                                printf("SD: multi block transfer, awaiting block %i ready\n",
                                cur_block);

            TIMEOUT_WAIT(mmio_read(EMMC_BASE + EMMC_INTERRUPT) & (wr_irpt | 0x8000), timeout);
            irpts = mmio_read(EMMC_BASE + EMMC_INTERRUPT);
            mmio_write(EMMC_BASE + EMMC_INTERRUPT, 0xffff0000 | wr_irpt);

            if((irpts & (0xffff0000 | wr_irpt)) != wr_irpt)
            {

            printf("SD: error occured whilst waiting for data ready interrupt\n");

                dev->last_error = irpts & 0xffff0000;
                dev->last_interrupt = irpts;
                return;
            }

            // Transfer the block
            size_t cur_byte_no = 0;
            while(cur_byte_no < dev->block_size)
            {
                if(is_write)
                                {
                                        uint32_t data = read_word((uint8_t *)cur_buf_addr, 0);
										mmio_write(EMMC_BASE + EMMC_DATA, data);
                                }
                else
                                {
                                        uint32_t data = mmio_read(EMMC_BASE + EMMC_DATA);
                                        write_word(data, (uint8_t *)cur_buf_addr, 0);
                                }
                cur_byte_no += 4;
                cur_buf_addr++;
            }


                        printf("SD: block %i transfer complete\n", cur_block);


            cur_block++;
        }
    }

    // Wait for transfer complete (set if read/write transfer or with busy)
    if((((cmd_reg & SD_CMD_RSPNS_TYPE_MASK) == SD_CMD_RSPNS_TYPE_48B) ||
       (cmd_reg & SD_CMD_ISDATA)) )
    {
        // First check command inhibit (DAT) is not already 0
        if((mmio_read(EMMC_BASE + EMMC_STATUS) & 0x2) == 0)
            mmio_write(EMMC_BASE + EMMC_INTERRUPT, 0xffff0002);
        else
        {
            TIMEOUT_WAIT(mmio_read(EMMC_BASE + EMMC_INTERRUPT) & 0x8002, timeout);
            irpts = mmio_read(EMMC_BASE + EMMC_INTERRUPT);
            mmio_write(EMMC_BASE + EMMC_INTERRUPT, 0xffff0002);

            // Handle the case where both data timeout and transfer complete
            // are set - transfer complete overrides data timeout: HCSS 2.2.17
            if(((irpts & 0xffff0002) != 0x2) && ((irpts & 0xffff0002) != 0x100002))
            {

                printf("SD: error occured whilst waiting for transfer complete interrupt\n");

                dev->last_error = irpts & 0xffff0000;
                dev->last_interrupt = irpts;
                return;
            }
            mmio_write(EMMC_BASE + EMMC_INTERRUPT, 0xffff0002);
        }
    }

    // Return success
    dev->last_cmd_success = 1;
}

static void sd_handle_card_interrupt(struct emmc_block_dev *dev)
{
    // Handle a card interrupt


    uint32_t status = mmio_read(EMMC_BASE + EMMC_STATUS);

    printf("SD: card interrupt\n");
    printf("SD: controller status: %08x\n", status);


    // Get the card status
    if(dev->card_rca)
    {
        sd_issue_command_int(dev, sd_commands[SEND_STATUS], dev->card_rca << 16,
                         500000);
        if(FAIL(dev))
        {

            printf("SD: unable to get card status\n");

        }
        else
        {

            printf("SD: card status: %08x\n", dev->last_r0);

        }
    }
    else
    {

        printf("SD: no card currently selected\n");

    }
}

static void sd_handle_interrupts(struct emmc_block_dev *dev)
{
    uint32_t irpts = mmio_read(EMMC_BASE + EMMC_INTERRUPT);
    uint32_t reset_mask = 0;

    if(irpts & SD_COMMAND_COMPLETE)
    {

        printf("SD: spurious command complete interrupt\n");

        reset_mask |= SD_COMMAND_COMPLETE;
    }

    if(irpts & SD_TRANSFER_COMPLETE)
    {

        printf("SD: spurious transfer complete interrupt\n");

        reset_mask |= SD_TRANSFER_COMPLETE;
    }

    if(irpts & SD_BLOCK_GAP_EVENT)
    {

        printf("SD: spurious block gap event interrupt\n");

        reset_mask |= SD_BLOCK_GAP_EVENT;
    }

    if(irpts & SD_DMA_INTERRUPT)
    {

        printf("SD: spurious DMA interrupt\n");

        reset_mask |= SD_DMA_INTERRUPT;
    }

    if(irpts & SD_BUFFER_WRITE_READY)
    {

        printf("SD: spurious buffer write ready interrupt\n");

        reset_mask |= SD_BUFFER_WRITE_READY;
        sd_reset_dat();
    }

    if(irpts & SD_BUFFER_READ_READY)
    {

        printf("SD: spurious buffer read ready interrupt\n");

        reset_mask |= SD_BUFFER_READ_READY;
        sd_reset_dat();
    }

    if(irpts & SD_CARD_INSERTION)
    {

        printf("SD: card insertion detected\n");

        reset_mask |= SD_CARD_INSERTION;
    }

    if(irpts & SD_CARD_REMOVAL)
    {

        printf("SD: card removal detected\n");

        reset_mask |= SD_CARD_REMOVAL;
        dev->card_removal = 1;
    }

    if(irpts & SD_CARD_INTERRUPT)
    {

        printf("SD: card interrupt detected\n");

        sd_handle_card_interrupt(dev);
        reset_mask |= SD_CARD_INTERRUPT;
    }

    if(irpts & 0x8000)
    {

        printf("SD: spurious error interrupt: %08x\n", irpts);

        reset_mask |= 0xffff0000;
    }

    mmio_write(EMMC_BASE + EMMC_INTERRUPT, reset_mask);
}


static void sd_issue_command(struct emmc_block_dev *dev, uint32_t command, uint32_t argument, useconds_t timeout)
{
    // First, handle any pending interrupts
    sd_handle_interrupts(dev);

    // Stop the command issue if it was the card remove interrupt that was
    // handled
    if(dev->card_removal)
    {
        dev->last_cmd_success = 0;
        return;
    }

    // Now run the appropriate commands by calling sd_issue_command_int()
    if(command & IS_APP_CMD)
    {
        command &= 0xff;

        printf("SD: issuing command ACMD%i\n", command);


        if(sd_acommands[command] == SD_CMD_RESERVED(0))
        {
            printf("SD: invalid command ACMD%i\n", command);
            dev->last_cmd_success = 0;
            return;
        }
        dev->last_cmd = APP_CMD;

        uint32_t rca = 0;
        if(dev->card_rca)
            rca = dev->card_rca << 16;
        sd_issue_command_int(dev, sd_commands[APP_CMD], rca, timeout);
        if(dev->last_cmd_success)
        {
            dev->last_cmd = command | IS_APP_CMD;
            sd_issue_command_int(dev, sd_acommands[command], argument, timeout);
        }
    }
    else
    {

        printf("SD: issuing command CMD%i\n", command);


        if(sd_commands[command] == SD_CMD_RESERVED(0))
        {
            printf("SD: invalid command CMD%i\n", command);
            dev->last_cmd_success = 0;
            return;
        }

        dev->last_cmd = command;
        sd_issue_command_int(dev, sd_commands[command], argument, timeout);
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
int sd_card_init(struct emmc_block_dev *emmc_dev, char force_erase)
{
    // Check the sanity of the sd_commands and sd_acommands structures
    if(sizeof(sd_commands) != (64 * sizeof(uint32_t)))
    {
        printf("EMMC: fatal error, sd_commands of incorrect size: %i"
               " expected %i\n", sizeof(sd_commands),
               64 * sizeof(uint32_t));
        return -1;
    }
    if(sizeof(sd_acommands) != (64 * sizeof(uint32_t)))
    {
        printf("EMMC: fatal error, sd_acommands of incorrect size: %i"
               " expected %i\n", sizeof(sd_acommands),
               64 * sizeof(uint32_t));
        return -1;
    }

              // Reset the controller
	
        printf("EMMC: resetting controller\n");

        uint32_t control1 = mmio_read(EMMC_BASE + EMMC_CONTROL1);
        control1 |= (1 << 24);
        // Disable clock
        control1 &= ~(1 << 2);
        control1 &= ~(1 << 0);
        mmio_write(EMMC_BASE + EMMC_CONTROL1, control1);
        TIMEOUT_WAIT((mmio_read(EMMC_BASE + EMMC_CONTROL1) & (0x7 << 24)) == 0, 1000000);
        if((mmio_read(EMMC_BASE + EMMC_CONTROL1) & (0x7 << 24)) != 0)
        {
                printf("EMMC: controller did not reset properly\n");
                return -1;
        }
		
		printf("EMMC: control0: %08x, control1: %08x, control2: %08x\n",
                        mmio_read(EMMC_BASE + EMMC_CONTROL0),
                        mmio_read(EMMC_BASE + EMMC_CONTROL1),
            mmio_read(EMMC_BASE + EMMC_CONTROL2));


        // Read the capabilities registers
        capabilities_0 = mmio_read(EMMC_BASE + EMMC_CAPABILITIES_0);
        capabilities_1 = mmio_read(EMMC_BASE + EMMC_CAPABILITIES_1);

        printf("EMMC: capabilities: %08x%08x\n", capabilities_1, capabilities_0);


        // Check for a valid card

        printf("EMMC: checking for an inserted card\n");

		TIMEOUT_WAIT(mmio_read(EMMC_BASE + EMMC_STATUS) & (1 << 16), 500000);
        uint32_t status_reg = mmio_read(EMMC_BASE + EMMC_STATUS);
        if((status_reg & (1 << 16)) == 0)
        {
                printf("EMMC: no card inserted\n");
                return -1;
        }

        printf("EMMC: status: %08x\n", status_reg);


        // Clear control2
        mmio_write(EMMC_BASE + EMMC_CONTROL2, 0);

        // Get the base clock rate
        uint32_t base_clock = sd_get_base_clock_hz();
        if(base_clock == 0)
        {
         printf("EMMC: assuming clock rate to be 100MHz\n");
         base_clock = 100000000;
        }

        // Set clock rate to something slow

        printf("EMMC: setting clock rate\n");

        control1 = mmio_read(EMMC_BASE + EMMC_CONTROL1);
        control1 |= 1;                        // enable clock

        // Set to identification frequency (400 kHz)
        uint32_t f_id = sd_get_clock_divider(base_clock, SD_CLOCK_ID);
        if(f_id == SD_GET_CLOCK_DIVIDER_FAIL)
        {
                printf("EMMC: unable to get a valid clock divider for ID frequency\n");
                return -1;
        }
        control1 |= f_id;

        control1 |= (7 << 16);                // data timeout = TMCLK * 2^10
        mmio_write(EMMC_BASE + EMMC_CONTROL1, control1);
        TIMEOUT_WAIT(mmio_read(EMMC_BASE + EMMC_CONTROL1) & 0x2, 0x1000000);
        if((mmio_read(EMMC_BASE + EMMC_CONTROL1) & 0x2) == 0)
        {
                printf("EMMC: controller's clock did not stabilise within 1 second\n");
                return -1;
        }

        printf("EMMC: control0: %08x, control1: %08x\n",
                        mmio_read(EMMC_BASE + EMMC_CONTROL0),
                        mmio_read(EMMC_BASE + EMMC_CONTROL1));


        // Enable the SD clock

        printf("EMMC: enabling SD clock\n");

        usleep(2000);
        control1 = mmio_read(EMMC_BASE + EMMC_CONTROL1);
        control1 |= 4;
        mmio_write(EMMC_BASE + EMMC_CONTROL1, control1);
        usleep(2000);

        // Mask off sending interrupts to the ARM
        mmio_write(EMMC_BASE + EMMC_IRPT_EN, 0);
        // Reset interrupts
        mmio_write(EMMC_BASE + EMMC_INTERRUPT, 0xffffffff);
        // Have all interrupts sent to the INTERRUPT register
        uint32_t irpt_mask = 0xffffffff & (~SD_CARD_INTERRUPT);

    irpt_mask |= SD_CARD_INTERRUPT;

        mmio_write(EMMC_BASE + EMMC_IRPT_MASK, irpt_mask);

        usleep(2000);

    // Prepare the device structure
        struct emmc_block_dev *ret;
        if(emmc_dev == NULL)
                ret = (struct emmc_block_dev *)malloc(sizeof(struct emmc_block_dev));
        else
                ret = emmc_dev;

        memset(ret, 0, sizeof(struct emmc_block_dev));
        
		
		ret->bd.driver_name = driver_name;
        ret->bd.device_name = device_name;
        ret->bd.block_size = 512;
        ret->bd.read = sd_read;

    ret->bd.write = sd_write;

    ret->bd.supports_multiple_block_read = 1;
    ret->bd.supports_multiple_block_write = 1;
	
        ret->base_clock = base_clock;

        printf("CMD0: idle\n");
		// Send CMD0 to the card (reset to idle state)
        sd_issue_command(ret, GO_IDLE_STATE, 0, 500000);
		print_response_reg(emmc);
        if(FAIL(ret))
        {
        printf("SD: no CMD0 response\n");
        return -1;
        }

		printf("CMD1:(arg0) init and querry OCR\n");
		// Send CMD1 
        sd_issue_command(ret, SEND_OP_COND, 0x40FF8080, 1000000);
		print_response_reg(emmc);
        if(FAIL(ret))
        {
        printf("SD: no CMD1 (0) response\n");
        return -1;
        }
		
		usleep(1000000);
		uint32_t counter =0;
		printf("init loop: reissue CMD1 untill out of idle");
		while((counter<10) &&(CHECKBIT(ret->last_r0, 31)) == 0){
		 sd_issue_command(ret, SEND_OP_COND, 0x40FF8080, 1000000);
		 counter++;
		 printf("iteration %i\n", counter);
		 print_response_reg(emmc);
		 usleep(1000000);
         if(FAIL(ret))
         {
         printf("SD: no CMD1 (0) response\n");
         return -1;
         }
		
		}
		

/*
		printf("CMD1:(arg0) init and supply OCR 0xFF8080 \n");
		// Send CMD1 
        sd_issue_command(ret, SEND_OP_COND, 0xFF8080, 1000000);
		print_response_reg(emmc);
        if(FAIL(ret))
        {
        printf("SD: no CMD1 (0xFF8080) response\n");
        return -1;
        }
	*/	
		printf("CMD2: CID and id mode\n");
        // Send CMD2 to get the cards CID
        sd_issue_command(ret, ALL_SEND_CID, 0, 500000);
        if(FAIL(ret))
        {
         printf("SD: error sending ALL_SEND_CID\n");
         return -1;
        }
        uint32_t card_cid_0 = ret->last_r0;
        uint32_t card_cid_1 = ret->last_r1;
        uint32_t card_cid_2 = ret->last_r2;
        uint32_t card_cid_3 = ret->last_r3;


        printf("SD: card CID: %08x%08x%08x%08x\n", card_cid_3, card_cid_2, card_cid_1, card_cid_0);

        uint32_t *dev_id = (uint32_t *)malloc(4 * sizeof(uint32_t));
        dev_id[0] = card_cid_0;
        dev_id[1] = card_cid_1;
        dev_id[2] = card_cid_2;
        dev_id[3] = card_cid_3;
        ret->bd.device_id = (uint8_t *)dev_id;
        ret->bd.dev_id_len = 4 * sizeof(uint32_t);
		
		ret->card_rca = 0xBEEF;
		
		printf("CMD3: assign RCA and standby mode\n");
        // Send CMD3 to assign rca andenter the data state
        sd_issue_command(ret, SEND_RELATIVE_ADDR, ret->card_rca << 16, 500000);
		print_response_reg(emmc);
        if(FAIL(ret))
    {
        printf("SD: error sending SEND_RELATIVE_ADDR\n");
        free(ret);
        return -1;
    }

        uint32_t cmd3_resp = ret->last_r0;

        printf("SD: CMD3 response: %08x\n", cmd3_resp);


         
        uint32_t crc_error = (cmd3_resp >> 15) & 0x1;
        uint32_t illegal_cmd = (cmd3_resp >> 14) & 0x1;
        uint32_t error = (cmd3_resp >> 13) & 0x1;
        uint32_t status = (cmd3_resp >> 9) & 0xf;
        uint32_t ready = (cmd3_resp >> 8) & 0x1;

        if(crc_error)
        {
                printf("SD: CRC error\n");
                free(ret);
                free(dev_id);
                return -1;
        }

        if(illegal_cmd)
        {
                printf("SD: illegal command\n");
                free(ret);
                free(dev_id);
                return -1;
        }

        if(error)
        {
                printf("SD: generic error\n");
                free(ret);
                free(dev_id);
                return -1;
        }

        if(!ready)
        {
                printf("SD: not ready for data\n");
                free(ret);
                free(dev_id);
                return -1;
        }


        printf("SD: RCA: %04x\n", ret->card_rca);


		// Send CMD9 to get the cards CSD
		printf("CMD9: get CSD\n");
        sd_issue_command(ret, SEND_CSD, ret->card_rca << 16, 500000);
		print_response_reg(emmc);
        if(FAIL(ret))
        {
         printf("SD: error sending SEND_CSD\n");
         return -1;
        }
        uint32_t card_csd_0 = ret->last_r0;
        uint32_t card_csd_1 = ret->last_r1;
        uint32_t card_csd_2 = ret->last_r2;
        uint32_t card_csd_3 = ret->last_r3;


        printf("SD: card CSD: %08x%08x%08x%08x\n", card_csd_3, card_csd_2, card_csd_1, card_csd_0);



        // Now select the card (toggles it to transfer state)
		printf("CMD7: switch to transfer mode\n");
        sd_issue_command(ret, SELECT_CARD, ret->card_rca << 16, 500000);
		print_response_reg(emmc);
        if(FAIL(ret))
        {
         printf("SD: error sending CMD7\n");
         free(ret);
         return -1;
        }

        uint32_t cmd7_resp = ret->last_r0;
        status = (cmd7_resp >> 9) & 0xf;

        if((status != 3) && (status != 4))
        {
                printf("SD: invalid status (%i)\n", status);
                free(ret);
                free(dev_id);
                return -1;
        }
		
		
		printf("CMD13: get status register\n");
        sd_issue_command(ret, SEND_STATUS, ret->card_rca << 16, 500000);
		print_response_reg(emmc);
        if(FAIL(ret))
        {
         printf("SD: error sending CMD13\n");
         free(ret);
         return -1;
        }


        // CMD16: block length 32
        printf("CMD16: setting blocklength to 512\n");
         sd_issue_command(ret, SET_BLOCKLEN, 32, 500000);
		 print_response_reg(emmc);
         if(FAIL(ret))
         {
         printf("SD: error sending SET_BLOCKLEN\n");
         return -1;
         }
        
        ret->block_size = 32;
        uint32_t controller_block_size = mmio_read(EMMC_BASE + EMMC_BLKSIZECNT);
        controller_block_size &= (~0xfff);
        controller_block_size |= 0x1;
        mmio_write(EMMC_BASE + EMMC_BLKSIZECNT, controller_block_size);

        

        // Reset interrupt register
        mmio_write(EMMC_BASE + EMMC_INTERRUPT, 0xffffffff);

        emmc_dev = ret;

        return 0;
}



static int sd_ensure_data_mode(struct emmc_block_dev *edev, char spi)
{
        if(edev->card_rca == 0)
        {
                // Try again to initialise the card
                int ret = sd_card_init(edev, spi);
                if(ret != 0)
                        return ret;
        }

        printf("SD: ensure_data_mode() obtaining status register for card_rca %08x: ",
                edev->card_rca);

    sd_issue_command(edev, SEND_STATUS, edev->card_rca << 16, 500000);
    if(FAIL(edev))
    {
        printf("SD: ensure_data_mode() error sending CMD13\n");
        edev->card_rca = 0;
        return -1;
    }

        uint32_t status = edev->last_r0;
        uint32_t cur_state = (status >> 9) & 0xf;

        printf("status %i\n", cur_state);

        if(cur_state == 3)
        {
                // Currently in the stand-by state - select it
                sd_issue_command(edev, SELECT_CARD, edev->card_rca << 16, 500000);
                if(FAIL(edev))
                {
                        printf("SD: ensure_data_mode() no response from CMD17\n");
                        edev->card_rca = 0;
                        return -1;
                }
        }
        else if(cur_state == 5)
        {
                // In the data transfer state - cancel the transmission
                sd_issue_command(edev, STOP_TRANSMISSION, 0, 500000);
                if(FAIL(edev))
                {
                        printf("SD: ensure_data_mode() no response from CMD12\n");
                        edev->card_rca = 0;
                        return -1;
                }

                // Reset the data circuit
                sd_reset_dat();
        }
        else if(cur_state != 4)
        {
                // Not in the transfer state - re-initialise
                int ret = sd_card_init(edev, spi);
                if(ret != 0)
                        return ret;
        }

        // Check again that we're now in the correct mode
        if(cur_state != 4)
        {

                printf("SD: ensure_data_mode() rechecking status: ");
        sd_issue_command(edev, SEND_STATUS, edev->card_rca << 16, 500000);
        if(FAIL(edev))
                {
                        printf("SD: ensure_data_mode() no response from CMD13\n");
                        edev->card_rca = 0;
                        return -1;
                }
                status = edev->last_r0;
                cur_state = (status >> 9) & 0xf;


                printf("%i\n", cur_state);


                if(cur_state != 4)
                {
                        printf("SD: unable to initialise SD card to "
                                        "data mode (state %i)\n", cur_state);
                        edev->card_rca = 0;
                        return -1;
                }
        }

        return 0;
}

static int sd_do_data_command(struct emmc_block_dev *edev, int is_write, uint8_t *buf, size_t buf_size, uint32_t block_no)
{
        // PLSS table 4.20 - SDSC cards use byte addresses rather than block addresses
        if(!edev->card_supports_sdhc)
                block_no *= 512;

        // This is as per HCSS 3.7.2.1
        if(buf_size < edev->block_size)
        {
         printf("SD: do_data_command() called with buffer size (%i) less than "
            "block size (%i)\n", buf_size, edev->block_size);
        return -1;
        }

        edev->blocks_to_transfer = buf_size / edev->block_size;
        if(buf_size % edev->block_size)
        {
         printf("SD: do_data_command() called with buffer size (%i) not an "
            "exact multiple of block size (%i)\n", buf_size, edev->block_size);
        return -1;
        }
        edev->buf = buf;

        // Decide on the command to use
        int command;
        if(is_write)
        {
         if(edev->blocks_to_transfer > 1)
            command = WRITE_MULTIPLE_BLOCK;
        else
            command = WRITE_BLOCK;
        }
        else
    {
        if(edev->blocks_to_transfer > 1)
            command = READ_MULTIPLE_BLOCK;
        else
            command = READ_SINGLE_BLOCK;
    }

        int retry_count = 0;
        int max_retries = 3;
        while(retry_count < max_retries)
        {

        sd_issue_command(edev, command, block_no, 5000000);

        if(SUCCESS(edev))
            break;
        else
        {
            printf("SD: error sending CMD%i, ", command);
            printf("error = %08x. ", edev->last_error);
            retry_count++;
            if(retry_count < max_retries)
                printf("Retrying...\n");
            else
                printf("Giving up.\n");
        }
        }
        if(retry_count == max_retries)
    {
        edev->card_rca = 0;
        return -1;
    }

    return 0;
}

int sd_read(struct block_device *dev, uint8_t *buf, size_t buf_size, uint32_t block_no, char spi)
{
        // Check the status of the card
        struct emmc_block_dev *edev = (struct emmc_block_dev *)dev;
    if(sd_ensure_data_mode(edev, spi) != 0)
        return -1;


        printf("SD: read() card ready, reading from block %u\n", block_no);


    if(sd_do_data_command(edev, 0, buf, buf_size, block_no) < 0)
        return -1;


        printf("SD: data read successful\n");


        return buf_size;
}


int sd_write(struct block_device *dev, uint8_t *buf, size_t buf_size, uint32_t block_no, char spi)
{
        // Check the status of the card
        struct emmc_block_dev *edev = (struct emmc_block_dev *)dev;
    if(sd_ensure_data_mode(edev, spi) != 0)
        return -1;


        printf("SD: write() card ready, reading from block %u\n", block_no);


    if(sd_do_data_command(edev, 1, buf, buf_size, block_no) < 0)
        return -1;


        printf("SD: write read successful\n");


        return buf_size;
}


void view_register(uint32_t* emmc){
 char in;
 
 while(toupper(in) != 'Q'){
 
  printf(" (C)MDTM | (A)RG1 | (R)esponse | (S)tatus | (I)nterrupt | Control(0-2) | (Q)uit\n");

  scanf(" %c",&in);
  printf("\n");
  
  switch(toupper(in)){
   case 'C': print_cmdtm_reg(emmc); continue;
   case 'A': print_arg1_reg(emmc); continue;
   case 'R': print_response_reg(emmc); continue; 
   case 'S': print_status_reg(emmc); continue;
   case 'I': print_interrupt_reg(emmc); continue;
   case '0': print_control0_reg(emmc); continue;
   case '1': print_control1_reg(emmc); continue;
   case '2': print_control2_reg(emmc); continue;
   case 'Q': break;
   default: continue; 
  }
 }
 

}



void dedication(){
 printf("You might ask: \"bkifft, why the fuck are you dedicating this tool to the user crazyace2011?\" \n\n");
 printf("Easy: this shithead gave me the spite fuelled energy to write it.\n\n");
 printf("Quotes from http://gbatemp.net/threads/has-anyone-with-a-brick-been-able-to-recover.360647/:\n\n \"im trying to understand something everyone is spitting out information that they truly don't know. the emmc is wiped or locked the nand is wiped out. no one has the hardware to know 100% but everyone is talking like they know if you knew you would have a way of fixing not just talking about whats wrong. people are just claiming to know what is wrong when they don't have the equipment back up the theory.\"\n\n");
 printf("\"im not saying you per say im just saying that everyone is talking like they are Einstein and know what is going on. who's to say that something is blocking the emmc controller not the emmc controller itself I don't know what it could be but im not throwing out stuff. im not ranting that you are doing it. its just people say something tech about the insides of a 3ds but the thing is no one know whats going on inside the 3ds and what the brick code actually did to the unit. yes we know that the system isn't responding to the nand that was installed by Nintendo but we don't know exactly what the gateway brick code did.\"\n\n"); 
 printf("\"I already said that I don't know how but you smart ass people think you know but honestly you don't know shit about it either. no one said you had to answer to my comment so stfu and ignore my post\"\n\n");
 printf("\"like we need more pointless 3ds brick threads real mature. must be a bunch of little kids that think they know everything. typical\"\n\n");
 printf("Quote from http://www.maxconsole.com/maxcon_forums/threads/280010-Update-on-RMAing-my-3DS?p=1671397#post1671397:\n\n\"im on gbatemp and there is a bunch of little kids that think they know everything and every theory. its like when a child tells a parent I know I know I know gets annoying\"\n");
 
}


int main(){

 

 int fd;
 uint32_t *gpio;
 void *peri_base;
 int i;
 int counter;
 int cmd;
 uint32_t arg;
 char in;
 
 
 printf("\e[1;1H\e[2J");
 printf("\n\n\n                  Dedicated to crazyace2011 @GBAtemp (crazyace @maxconsole) and every other elderly person who can count to potato.\n\n");
 
 
 fd = open("/dev/mem", O_RDWR|O_SYNC);
 if ( 0 > fd) {
  perror("Error opening /dev/mem");
  exit(EXIT_FAILURE);
 }
 
 peri_base = mmap(NULL, BLOCKSIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, BCM2708_PERI_BASE);
 
 if ((void*)-1 == peri_base){
  perror("Error while mapping");
  exit(EXIT_FAILURE);
 }

 emmc = peri_base + EMMC_OFFSET;
 gpio = peri_base + GPIO_OFFSET;

 
 struct emmc_block_dev emmc_device;
 
 
 
 in = '\0';
 
 while(toupper(in) != 'Q'){
 
  printf("(S)afe run (Querry only) | (V)iew Register | (D)edication | (Q)uit\n");

  scanf(" %c",&in);
  printf("\n");
  
  switch(toupper(in)){
   case 'S': sd_card_init(&emmc_device, '0');; continue;
   case 'V': view_register(emmc); continue;
   case 'D': dedication(); continue;
   case 'Q': break;
   default: continue; 
  }
 }
 
  
 
 
 printf("\n");
 if (0 > munmap(peri_base, BLOCKSIZE)){
  perror("munmap failed:");
  exit(EXIT_FAILURE);
 }
}
