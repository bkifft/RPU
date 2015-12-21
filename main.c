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

#define DEBUG

#define _BSD_SOURCE		//ugly hack but the rewrite follows (nned to replace usleep() with the imho way less handy nanosleep()

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <dirent.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <ctype.h>
#include <errno.h>

#include "defines.h"		//contains all the adresses, offsets, command structures and bitmasks
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

typedef uint32_t useconds_t;	//more ugly hacks
uint32_t *emmc;


static int sd_ensure_data_mode (struct emmc_block_dev *edev, char spi);


void
print_arg1_reg (uint32_t * emmc)
{
  uint32_t arg1 = *(emmc + EMMC_ARG1 / 4);	//div 4 as the offsets are in bytes
  printf ("ARG1:-----------------------------------------\n");
  printf ("ARG1: 			%#08X\n", arg1);
}

void
print_resp1_reg (uint32_t * emmc)
{
  uint32_t resp1 = *(emmc + EMMC_RESP1 / 4);	//div 4 as the offsets are in bytes
  printf ("RESP1:-----------------------------------------\n");
  printf ("RESP1:             %#08X\n", resp1);
}

void
print_resp2_reg (uint32_t * emmc)
{
  uint32_t resp2 = *(emmc + EMMC_RESP2 / 4);	//div 4 as the offsets are in bytes
  printf ("RESP2:-----------------------------------------\n");
  printf ("RESP2:             %#08X\n", resp2);
}


void
print_resp3_reg (uint32_t * emmc)
{
  uint32_t resp3 = *(emmc + EMMC_RESP3 / 4);	//div 4 as the offsets are in bytes
  printf ("RESP1:-----------------------------------------\n");
  printf ("RESP1: 			%#08X\n", resp3);
}

void
print_cmdtm_reg (uint32_t * emmc)
{
  uint32_t cmdtm = *(emmc + EMMC_CMDTM / 4);	//div 4 as the offsets are in bytes
  printf ("CMDTM:-----------------------------------------\n");
  printf ("CMD_INDEX:         %i\n", ((cmdtm & 0x3F000000) >> 24));
  printf ("CMD_TYPE:          %i%i\n", CHECKBIT (cmdtm, 23),
	  CHECKBIT (cmdtm, 22));
  printf ("CMD_ISDATA:        %i\n", CHECKBIT (cmdtm, 21));
  printf ("CMD_IXCHK_EN:      %i\n", CHECKBIT (cmdtm, 20));
  printf ("CMD_CRCCHK_EN:     %i\n", CHECKBIT (cmdtm, 19));
  printf ("CMD_RSPNS_TYPE:    %i%i\n", CHECKBIT (cmdtm, 17),
	  CHECKBIT (cmdtm, 16));
  printf ("TM_MULTI_BLOCK:    %i\n", CHECKBIT (cmdtm, 5));
  printf ("TM_DAT_DIR:        %i\n", CHECKBIT (cmdtm, 4));
  printf ("TM_AUTO_CMD_EN:    %i%i\n", CHECKBIT (cmdtm, 3),
	  CHECKBIT (cmdtm, 2));
  printf ("TM_BLKCNT_T:       %i\n", CHECKBIT (cmdtm, 1));


}

void
print_control0_reg (uint32_t * emmc)
{
  uint32_t ctrl0 = (*(emmc + EMMC_CONTROL0 / 4));	//div 4 as the offsets are in bytes
  printf ("CONTROL0:--------------------------------------\n");
  printf ("ALT_BOOT_EN:   %i\n", CHECKBIT (ctrl0, 22));
  printf ("BOOT_EN:       %i\n", CHECKBIT (ctrl0, 21));
  printf ("SPI_MODE:      %i\n", CHECKBIT (ctrl0, 20));
  printf ("GAP_IEN:       %i\n", CHECKBIT (ctrl0, 19));
  printf ("READWAIT_EN:   %i\n", CHECKBIT (ctrl0, 18));
  printf ("GAP_RESTART:   %i\n", CHECKBIT (ctrl0, 17));
  printf ("GAP_STOP:      %i\n", CHECKBIT (ctrl0, 16));
  printf ("HCTL_8BIT:     %i\n", CHECKBIT (ctrl0, 5));
  printf ("HCTL_HS_EN:    %i\n", CHECKBIT (ctrl0, 2));
  printf ("HCTL_DWIDTH:   %i\n", CHECKBIT (ctrl0, 1));
}

void
print_control1_reg (uint32_t * emmc)
{
  uint32_t ctrl1 = (*(emmc + EMMC_CONTROL1 / 4));	//div 4 as the offsets are in bytes
  printf ("CONTROL1:--------------------------------------\n");
  printf ("SRST_DATA:     %i\n", CHECKBIT (ctrl1, 26));
  printf ("SRST_CMD:      %i\n", CHECKBIT (ctrl1, 25));
  printf ("SRST_HC:       %i\n", CHECKBIT (ctrl1, 24));
  printf ("DATA_TOUNIT:   %i\n", ((ctrl1 & 0x0F0000) >> 16));
  printf ("CLK_FREQ8:     %i\n", ((ctrl1 & 0xFF00) >> 8));	//FIXME: should be combined with next value
  printf ("CLK_FREQ_MS2:  %i\n", ((ctrl1 & 0xC0) >> 6));
  printf ("CLK_GENSEL:    %i\n", CHECKBIT (ctrl1, 5));
  printf ("CLK_EN:        %i\n", CHECKBIT (ctrl1, 2));
  printf ("CLK_STABLE:    %i\n", CHECKBIT (ctrl1, 1));
  printf ("CLK_INTLEN:    %i\n", CHECKBIT (ctrl1, 0));
}

void
print_control2_reg (uint32_t * emmc)
{
  uint32_t ctrl2 = (*(emmc + EMMC_CONTROL1 / 4));	//div 4 as the offsets are in bytes
  printf ("CONTROL2:--------------------------------------\n");
  printf ("TUNED:         %i\n", CHECKBIT (ctrl2, 23));
  printf ("TUNEON:        %i\n", CHECKBIT (ctrl2, 22));
  printf ("UHSMODE:       %i%i%i\n", CHECKBIT (ctrl2, 18),
	  CHECKBIT (ctrl2, 17), CHECKBIT (ctrl2, 16));
  printf ("NOTC12_ERR :   %i\n", CHECKBIT (ctrl2, 7));
  printf ("ACBAD_ERR:     %i\n", CHECKBIT (ctrl2, 4));	//FIXME: should be combined with next value
  printf ("ACEND_ERR:     %i\n", CHECKBIT (ctrl2, 3));
  printf ("ACCRC_ERR      %i\n", CHECKBIT (ctrl2, 2));
  printf ("ACTO_ERR:      %i\n", CHECKBIT (ctrl2, 1));
  printf ("ACNOX_ERR:     %i\n", CHECKBIT (ctrl2, 0));
}

void
print_blksizecnt_reg (uint32_t * emmc)
{
  uint32_t blksizecnt = (*(emmc + EMMC_BLKSIZECNT / 4));	//div 4 as the offsets are in bytes 
  printf ("BLK_CNT:	%i\n", ((blksizecnt & 0xFFFF0000) >> 16));
  printf ("BLKSIZE:	%i\n", (blksizecnt & 0x1FF));

}

void
print_response_reg (uint32_t * emmc)
{
  printf ("RESPONSE:-----------------------------------\n");
  printf ("RESP3: 0x%08X\n", *(emmc + EMMC_RESP3 / 4));
  printf ("RESP2: 0x%08X\n", *(emmc + EMMC_RESP2 / 4));
  printf ("RESP1: 0x%08X\n", *(emmc + EMMC_RESP1 / 4));
  printf ("RESP0: 0x%08X\n", *(emmc + EMMC_RESP0 / 4));
}

void
clear_response_reg (uint32_t * emmc)
{
  *(emmc + EMMC_RESP0 / 4) = 0x0;	//clear the response 
  *(emmc + EMMC_RESP1 / 4) = 0x0;	//"
  *(emmc + EMMC_RESP2 / 4) = 0x0;	//"
  *(emmc + EMMC_RESP3 / 4) = 0x0;	//"
}


void
print_status_reg (uint32_t * emmc)
{
  uint32_t stat = (*(emmc + EMMC_STATUS / 4));	//div 4 as the offsets are in bytes
  printf ("STATUS:--------------------------------------\n");
  printf ("DAT7:          %i\n", CHECKBIT (stat, 28));
  printf ("DAT6:          %i\n", CHECKBIT (stat, 27));
  printf ("DAT5:          %i\n", CHECKBIT (stat, 26));
  printf ("DAT4:          %i\n", CHECKBIT (stat, 25));
  printf ("CMD_LEVEL:     %i\n", CHECKBIT (stat, 24));

  printf ("DAT3:          %i\n", CHECKBIT (stat, 23));
  printf ("DAT2:          %i\n", CHECKBIT (stat, 22));
  printf ("DAT1:          %i\n", CHECKBIT (stat, 21));
  printf ("DAT0:          %i\n", CHECKBIT (stat, 20));
  printf ("ALT_DAT_RDY:   %i\n", CHECKBIT (stat, 11));
  printf ("R_TRAN:        %i\n", CHECKBIT (stat, 9));
  printf ("W_TRAN:        %i\n", CHECKBIT (stat, 8));

  printf ("D_ACT:         %i\n", CHECKBIT (stat, 2));
  printf ("D_INH:         %i\n", CHECKBIT (stat, 1));
  printf ("CMD_INH:       %i\n", CHECKBIT (stat, 0));

}


void
print_interrupt_reg (uint32_t * emmc)
{
  uint32_t interrupt = (*(emmc + EMMC_INTERRUPT / 4));	//div 4 as the offsets are in bytes
  printf ("INTERRUPT:--------------------------------------\n");
  printf ("ACMD_ERR:       %i\n", CHECKBIT (interrupt, 24));
  printf ("DEND_ERR:       %i\n", CHECKBIT (interrupt, 22));
  printf ("DCRC_ERR:       %i\n", CHECKBIT (interrupt, 21));
  printf ("DTO_ERR:        %i\n", CHECKBIT (interrupt, 20));
  printf ("CBAD_ERR:       %i\n", CHECKBIT (interrupt, 19));
  printf ("CEND_ERR:       %i\n", CHECKBIT (interrupt, 18));
  printf ("CCRC_ERR:       %i\n", CHECKBIT (interrupt, 17));
  printf ("CTO_ERR:        %i\n", CHECKBIT (interrupt, 16));
  printf ("ERR:            %i\n", CHECKBIT (interrupt, 15));
  printf ("ENDBOOT:        %i\n", CHECKBIT (interrupt, 14));
  printf ("BOOTACK:        %i\n", CHECKBIT (interrupt, 13));
  printf ("RETUNE:         %i\n", CHECKBIT (interrupt, 12));
  printf ("CARD:           %i\n", CHECKBIT (interrupt, 8));
  printf ("READ_RDY:       %i\n", CHECKBIT (interrupt, 5));
  printf ("WRITE_RDY:      %i\n", CHECKBIT (interrupt, 4));
  printf ("BLOCK_GATE:     %i\n", CHECKBIT (interrupt, 2));
  printf ("DATA_DONE:      %i\n", CHECKBIT (interrupt, 1));
  printf ("CMD_DONE:       %i\n", CHECKBIT (interrupt, 0));
}

static char *sd_versions[] = { "unknown", "1.0 and 1.01", "1.10",
  "2.00", "3.0x", "4.xx"
};

void
mmio_write (void *reg, uint32_t data)
{

  *(volatile uint32_t *) (reg) = data;

}

uint32_t
mmio_read (void *reg)
{

  return *(volatile uint32_t *) (reg);

}

static void
sd_power_off ()
{
  /* Power off the SD card */
  uint32_t control0 = mmio_read (EMMC_BASE + EMMC_CONTROL0);
  control0 &= ~(1 << 8);	// Set SD Bus Power bit off in Power Control Register
  mmio_write (EMMC_BASE + EMMC_CONTROL0, control0);
}

static void
spi_mode_on ()
{
  uint32_t control0 = mmio_read (EMMC_BASE + EMMC_CONTROL0);
  control0 &= 1 << 20;		// spi bit
  mmio_write (EMMC_BASE + EMMC_CONTROL0, control0);
  printf ("SPI %s\n", (CHECKBIT (control0, 20) == 1) ? "on" : "off");
}

static uint32_t
sd_get_base_clock_hz ()
{
  return 250000000;
}

static int
bcm_2708_power_off ()
{
  return -1;
}

static int
bcm_2708_power_on ()
{
  return -1;
}

static int
bcm_2708_power_cycle ()
{
  if (bcm_2708_power_off () < 0)
    return -1;

  usleep (5000);

  return bcm_2708_power_on ();
}

static uint32_t
sd_get_clock_divider (uint32_t base_clock, uint32_t target_rate)
{
  // TODO: implement use of preset value registers

  uint32_t targetted_divisor = 0;
  if (target_rate > base_clock)
    targetted_divisor = 1;
  else
    {
      targetted_divisor = base_clock / target_rate;
      uint32_t mod = base_clock % target_rate;
      if (mod)
	targetted_divisor--;
    }

  // Decide on the clock mode to use

  // Currently only 10-bit divided clock mode is supported

  if (1)			//was a check for host controller interface version
    {
      // HCI version 3 or greater supports 10-bit divided clock mode
      // This requires a power-of-two divider

      // Find the first bit set
      int divisor = -1;
      for (int first_bit = 31; first_bit >= 0; first_bit--)
	{
	  uint32_t bit_test = (1 << first_bit);
	  if (targetted_divisor & bit_test)
	    {
	      divisor = first_bit;
	      targetted_divisor &= ~bit_test;
	      if (targetted_divisor)
		{
		  // The divisor is not a power-of-two, increase it
		  divisor++;
		}
	      break;
	    }
	}

      if (divisor == -1)
	divisor = 31;
      if (divisor >= 32)
	divisor = 31;

      if (divisor != 0)
	divisor = (1 << (divisor - 1));

      if (divisor >= 0x400)
	divisor = 0x3ff;

      uint32_t freq_select = divisor & 0xff;
      uint32_t upper_bits = (divisor >> 8) & 0x3;
      uint32_t ret = (freq_select << 8) | (upper_bits << 6) | (0 << 5);

      int denominator = 1;
      if (divisor != 0)
	denominator = divisor * 2;
      int actual_clock = base_clock / denominator;
#ifdef DEBUG
      printf ("EMMC: base_clock: %i, target_rate: %i, divisor: %08x, "
	      "actual_clock: %i, ret: %08x\n", base_clock, target_rate,
	      divisor, actual_clock, ret);
#endif

      return ret;

    }
  else
    {
      printf ("EMMC: unsupported host version\n");
      return SD_GET_CLOCK_DIVIDER_FAIL;
    }

}

// Switch the clock rate whilst running
static int
sd_switch_clock_rate (uint32_t base_clock, uint32_t target_rate)
{
  // Decide on an appropriate divider
  uint32_t divider = sd_get_clock_divider (base_clock, target_rate);
  if (divider == SD_GET_CLOCK_DIVIDER_FAIL)
    {
      printf ("EMMC: couldn't get a valid divider for target rate %i Hz\n",
	      target_rate);
      return -1;
    }

  // Wait for the command inhibit (CMD and DAT) bits to clear
  while (mmio_read (EMMC_BASE + EMMC_STATUS) & 0x3)
    usleep (1000);

  // Set the SD clock off
  uint32_t control1 = mmio_read (EMMC_BASE + EMMC_CONTROL1);
  control1 &= ~(1 << 2);
  mmio_write (EMMC_BASE + EMMC_CONTROL1, control1);
  usleep (2000);

  // Write the new divider
  control1 &= ~0xffe0;		// Clear old setting + clock generator select
  control1 |= divider;
  mmio_write (EMMC_BASE + EMMC_CONTROL1, control1);
  usleep (2000);

  // Enable the SD clock
  control1 |= (1 << 2);
  mmio_write (EMMC_BASE + EMMC_CONTROL1, control1);
  usleep (2000);
#ifdef DEBUG
  printf ("EMMC: successfully set clock rate to %i Hz\n", target_rate);
#endif
  return 0;
}

// Reset the CMD line
static int
sd_reset_cmd ()
{
  uint32_t control1 = mmio_read (EMMC_BASE + EMMC_CONTROL1);
  control1 |= SD_RESET_CMD;
  mmio_write (EMMC_BASE + EMMC_CONTROL1, control1);
  TIMEOUT_WAIT ((mmio_read (EMMC_BASE + EMMC_CONTROL1) & SD_RESET_CMD) == 0,
		1000000);
  if ((mmio_read (EMMC_BASE + EMMC_CONTROL1) & SD_RESET_CMD) != 0)
    {
      printf ("EMMC: CMD line did not reset properly\n");
      return -1;
    }
  return 0;
}

// Reset the CMD line
static int
sd_reset_dat ()
{
  uint32_t control1 = mmio_read (EMMC_BASE + EMMC_CONTROL1);
  control1 |= SD_RESET_DAT;
  mmio_write (EMMC_BASE + EMMC_CONTROL1, control1);
  TIMEOUT_WAIT ((mmio_read (EMMC_BASE + EMMC_CONTROL1) & SD_RESET_DAT) == 0,
		1000000);
  if ((mmio_read (EMMC_BASE + EMMC_CONTROL1) & SD_RESET_DAT) != 0)
    {
      printf ("EMMC: DAT line did not reset properly\n");
      return -1;
    }
  return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
static void
sd_issue_command_int (struct emmc_block_dev *dev, uint32_t cmd_reg,
		      uint32_t argument, useconds_t timeout)
{
  dev->last_cmd_reg = cmd_reg;
  dev->last_cmd_success = 0;

  // This is as per HCSS 3.7.1.1/3.7.2.2

  // Check Command Inhibit
  while (mmio_read (EMMC_BASE + EMMC_STATUS) & 0x1)
    usleep (1000);

  // Is the command with busy?
  if ((cmd_reg & SD_CMD_RSPNS_TYPE_MASK) == SD_CMD_RSPNS_TYPE_48B)
    {
      // With busy

      // Is is an abort command?
      if ((cmd_reg & SD_CMD_TYPE_MASK) != SD_CMD_TYPE_ABORT)
	{
	  // Not an abort command

	  // Wait for the data line to be free
	  while (mmio_read (EMMC_BASE + EMMC_STATUS) & 0x2)
	    usleep (1000);
	}
    }

  // Set block size and block count
  // For now, block size = 512 bytes, block count = 1,
  if (dev->blocks_to_transfer > 0xffff)
    {
      printf ("SD_send_int: blocks_to_transfer too great (%i)\n",
	      dev->blocks_to_transfer);
      dev->last_cmd_success = 0;
      return;
    }
  uint32_t blksizecnt = dev->block_size | (dev->blocks_to_transfer << 16);
  mmio_write (EMMC_BASE + EMMC_BLKSIZECNT, blksizecnt);

  // Set argument 1 reg
  mmio_write (EMMC_BASE + EMMC_ARG1, argument);

  // Set command reg
  mmio_write (EMMC_BASE + EMMC_CMDTM, cmd_reg);

  usleep (2000);

  // Wait for command complete interrupt
  TIMEOUT_WAIT (mmio_read (EMMC_BASE + EMMC_INTERRUPT) & 0x8001, timeout);
  uint32_t irpts = mmio_read (EMMC_BASE + EMMC_INTERRUPT);



  // Test for errors
  if ((irpts & 0xffff0001) != 0x1)
    {

      printf
	("SD_send_int: error occured whilst waiting for command complete interrupt\n");
      print_interrupt_reg (emmc);
      dev->last_error = irpts & 0xffff0000;
      dev->last_interrupt = irpts;
      return;
    }

// Clear command complete status
  mmio_write (EMMC_BASE + EMMC_INTERRUPT, 0xffff0001);
  usleep (2000);

  // Get response data
  switch (cmd_reg & SD_CMD_RSPNS_TYPE_MASK)
    {
    case SD_CMD_RSPNS_TYPE_48:
    case SD_CMD_RSPNS_TYPE_48B:
      dev->last_r0 = mmio_read (EMMC_BASE + EMMC_RESP0);
      break;

    case SD_CMD_RSPNS_TYPE_136:
      dev->last_r0 = mmio_read (EMMC_BASE + EMMC_RESP0);
      dev->last_r1 = mmio_read (EMMC_BASE + EMMC_RESP1);
      dev->last_r2 = mmio_read (EMMC_BASE + EMMC_RESP2);
      dev->last_r3 = mmio_read (EMMC_BASE + EMMC_RESP3);
      break;
    }

  // If with data, wait for the appropriate interrupt
  if ((cmd_reg & SD_CMD_ISDATA))
    {
      uint32_t wr_irpt;
      int is_write = 0;
      if (cmd_reg & SD_CMD_DAT_DIR_CH)
	wr_irpt = (1 << 5);	// read
      else
	{
	  is_write = 1;
	  wr_irpt = (1 << 4);	// write
	}

      int cur_block = 0;
      uint32_t *cur_buf_addr = (uint32_t *) dev->buf;
      while (cur_block < dev->blocks_to_transfer)
	{

	  if (dev->blocks_to_transfer > 1)
	    printf
	      ("SD_send_int: multi block transfer, awaiting block %i ready\n",
	       cur_block);

	  TIMEOUT_WAIT (mmio_read (EMMC_BASE + EMMC_INTERRUPT) &
			(wr_irpt | 0x8000), timeout);
	  irpts = mmio_read (EMMC_BASE + EMMC_INTERRUPT);
	  mmio_write (EMMC_BASE + EMMC_INTERRUPT, 0xffff0000 | wr_irpt);

	  if ((irpts & (0xffff0000 | wr_irpt)) != wr_irpt)
	    {

	      printf
		("SD_send_int: error occured whilst waiting for data ready interrupt\n");

	      dev->last_error = irpts & 0xffff0000;
	      dev->last_interrupt = irpts;
	      return;
	    }

	  // Transfer the block
	  size_t cur_byte_no = 0;
	  while (cur_byte_no < dev->block_size)
	    {
	      if (is_write)
		{
		  uint32_t data = read_word ((uint8_t *) cur_buf_addr, 0);
		  mmio_write (EMMC_BASE + EMMC_DATA, data);
		}
	      else
		{
		  uint32_t data = mmio_read (EMMC_BASE + EMMC_DATA);
		  write_word (data, (uint8_t *) cur_buf_addr, 0);
		}
	      cur_byte_no += 4;
	      cur_buf_addr++;
	    }
#ifdef DEBUG
	  printf ("SD_send_int: block %i transfer complete\n", cur_block);
#endif

	  cur_block++;
	}
    }

  // Wait for transfer complete (set if read/write transfer or with busy)
  if ((((cmd_reg & SD_CMD_RSPNS_TYPE_MASK) == SD_CMD_RSPNS_TYPE_48B) ||
       (cmd_reg & SD_CMD_ISDATA)))
    {
      // First check command inhibit (DAT) is not already 0
      if ((mmio_read (EMMC_BASE + EMMC_STATUS) & 0x2) == 0)
	mmio_write (EMMC_BASE + EMMC_INTERRUPT, 0xffff0002);
      else
	{
	  TIMEOUT_WAIT (mmio_read (EMMC_BASE + EMMC_INTERRUPT) & 0x8002,
			timeout);
	  irpts = mmio_read (EMMC_BASE + EMMC_INTERRUPT);
	  mmio_write (EMMC_BASE + EMMC_INTERRUPT, 0xffff0002);

	  // Handle the case where both data timeout and transfer complete
	  // are set - transfer complete overrides data timeout: HCSS 2.2.17
	  if (((irpts & 0xffff0002) != 0x2)
	      && ((irpts & 0xffff0002) != 0x100002))
	    {

	      printf
		("SD_send_int: error occured whilst waiting for transfer complete interrupt\n");

	      dev->last_error = irpts & 0xffff0000;
	      dev->last_interrupt = irpts;
	      return;
	    }
	  mmio_write (EMMC_BASE + EMMC_INTERRUPT, 0xffff0002);
	}
    }

  // Return success
  dev->last_cmd_success = 1;
}

static void
sd_handle_card_interrupt (struct emmc_block_dev *dev)
{
  // Handle a card interrupt


  uint32_t status = mmio_read (EMMC_BASE + EMMC_STATUS);

#ifdef DEBUG
  printf ("SD_handle_intr: card interrupt\n");
  printf ("SD_handle_intr: controller status: %08x\n", status);
#endif

  // Get the card status
  if (dev->card_rca)
    {
      sd_issue_command_int (dev, sd_commands[SEND_STATUS],
			    dev->card_rca << 16, 500000);
      if (FAIL (dev))
	{

	  printf ("SD_handle_intr: unable to get card status\n");

	}
      else
	{
#ifdef DEBUG
	  printf ("SD_handle_intr: card status: %08x\n", dev->last_r0);
#endif
	}
    }
  else
    {

      printf ("SD_handle_intr: no card currently selected\n");

    }
}

static void
sd_handle_interrupts (struct emmc_block_dev *dev)
{
  uint32_t irpts = mmio_read (EMMC_BASE + EMMC_INTERRUPT);
  uint32_t reset_mask = 0;

  if (irpts & SD_COMMAND_COMPLETE)
    {

      printf ("SD_interrupt: spurious command complete interrupt\n");

      reset_mask |= SD_COMMAND_COMPLETE;
    }

  if (irpts & SD_TRANSFER_COMPLETE)
    {

      printf ("SD_interrupt: spurious transfer complete interrupt\n");

      reset_mask |= SD_TRANSFER_COMPLETE;
    }

  if (irpts & SD_BLOCK_GAP_EVENT)
    {

      printf ("SD_interrupt: spurious block gap event interrupt\n");

      reset_mask |= SD_BLOCK_GAP_EVENT;
    }

  if (irpts & SD_DMA_INTERRUPT)
    {

      printf ("SD_interrupt: spurious DMA interrupt\n");

      reset_mask |= SD_DMA_INTERRUPT;
    }

  if (irpts & SD_BUFFER_WRITE_READY)
    {

      printf ("SD_interrupt: spurious buffer write ready interrupt\n");

      reset_mask |= SD_BUFFER_WRITE_READY;
      sd_reset_dat ();
    }

  if (irpts & SD_BUFFER_READ_READY)
    {

      printf ("SD_interrupt: spurious buffer read ready interrupt\n");

      reset_mask |= SD_BUFFER_READ_READY;
      sd_reset_dat ();
    }

  if (irpts & SD_CARD_INSERTION)
    {

      printf ("SD_interrupt: card insertion detected\n");

      reset_mask |= SD_CARD_INSERTION;
    }

  if (irpts & SD_CARD_REMOVAL)
    {

      printf ("SD_interrupt: card removal detected\n");

      reset_mask |= SD_CARD_REMOVAL;
      dev->card_removal = 1;
    }

  if (irpts & SD_CARD_INTERRUPT)
    {

      printf ("SD_interrupt: card interrupt detected\n");

      sd_handle_card_interrupt (dev);
      reset_mask |= SD_CARD_INTERRUPT;
    }

  if (irpts & 0x8000)
    {

      printf ("SD_interrupt: spurious error interrupt: %08x\n", irpts);

      reset_mask |= 0xffff0000;
    }

  mmio_write (EMMC_BASE + EMMC_INTERRUPT, reset_mask);
}


static void
sd_issue_command (struct emmc_block_dev *dev, uint32_t command,
		  uint32_t argument, useconds_t timeout)
{
  // First, handle any pending interrupts
  sd_handle_interrupts (dev);

  // Stop the command issue if it was the card remove interrupt that was
  // handled
  if (dev->card_removal)
    {
      dev->last_cmd_success = 0;
      return;
    }

  // Now run the appropriate commands by calling sd_issue_command_int()
  if (command & IS_APP_CMD)
    {
      command &= 0xff;
#ifdef DEBUG
      printf ("sd_issue_command: issuing command ACMD%i\n", command);
#endif

      if (sd_acommands[command] == SD_CMD_RESERVED (0))
	{
	  printf ("sd_issue_command: invalid command ACMD%i\n", command);
	  dev->last_cmd_success = 0;
	  return;
	}
      dev->last_cmd = APP_CMD;

      uint32_t rca = 0;
      if (dev->card_rca)
	rca = dev->card_rca << 16;
      sd_issue_command_int (dev, sd_commands[APP_CMD], rca, timeout);
      if (dev->last_cmd_success)
	{
	  dev->last_cmd = command | IS_APP_CMD;
	  sd_issue_command_int (dev, sd_acommands[command], argument,
				timeout);
	}
    }
  else
    {
#ifdef DEBUG
      printf ("sd_issue_command: issuing command CMD%i\n", command);
#endif

      if (sd_commands[command] == SD_CMD_RESERVED (0))
	{
	  printf ("sd_issue_command:: invalid command CMD%i\n", command);
	  dev->last_cmd_success = 0;
	  return;
	}

      dev->last_cmd = command;
      sd_issue_command_int (dev, sd_commands[command], argument, timeout);
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
int
sd_card_init (struct emmc_block_dev *emmc_dev, char mode, int variant)
{
  // Check the sanity of the sd_commands and sd_acommands structures
  if (sizeof (sd_commands) != (64 * sizeof (uint32_t)))
    {
      printf ("EMMC: fatal error, sd_commands of incorrect size: %i"
	      " expected %i\n", sizeof (sd_commands), 64 * sizeof (uint32_t));
      return -1;
    }
  if (sizeof (sd_acommands) != (64 * sizeof (uint32_t)))
    {
      printf ("EMMC: fatal error, sd_acommands of incorrect size: %i"
	      " expected %i\n", sizeof (sd_acommands),
	      64 * sizeof (uint32_t));
      return -1;
    }

  uint32_t controller_block_size;

////////////////////////////////////////////////////////////////////////////////////////////////////////
//host init

  // Reset the controller

  printf ("EMMC: resetting controller\n");

  uint32_t control1 = mmio_read (EMMC_BASE + EMMC_CONTROL1);
  control1 |= (1 << 24);
  // Disable clock
  control1 &= ~(1 << 2);
  control1 &= ~(1 << 0);
  mmio_write (EMMC_BASE + EMMC_CONTROL1, control1);
  TIMEOUT_WAIT ((mmio_read (EMMC_BASE + EMMC_CONTROL1) & (0x7 << 24)) == 0,
		1000000);
  if ((mmio_read (EMMC_BASE + EMMC_CONTROL1) & (0x7 << 24)) != 0)
    {
      printf ("EMMC: controller did not reset properly\n");
      return -1;
    }
#ifdef DEBUG
  printf ("EMMC: control0: %08x, control1: %08x, control2: %08x\n",
	  mmio_read (EMMC_BASE + EMMC_CONTROL0),
	  mmio_read (EMMC_BASE + EMMC_CONTROL1),
	  mmio_read (EMMC_BASE + EMMC_CONTROL2));
#endif

  // Read the capabilities registers
  capabilities_0 = mmio_read (EMMC_BASE + EMMC_CAPABILITIES_0);
  capabilities_1 = mmio_read (EMMC_BASE + EMMC_CAPABILITIES_1);
#ifdef DEBUG
  printf ("EMMC: capabilities: %08x%08x\n", capabilities_1, capabilities_0);
#endif

  // Check for a valid card
#ifdef DEBUG
  printf ("EMMC: checking for an inserted card\n");
#endif
  TIMEOUT_WAIT (mmio_read (EMMC_BASE + EMMC_STATUS) & (1 << 16), 500000);
  uint32_t status_reg = mmio_read (EMMC_BASE + EMMC_STATUS);
  if ((status_reg & (1 << 16)) == 0)
    {
      printf ("EMMC: no card inserted\n");
      return -1;
    }
#ifdef DEBUG
  printf ("EMMC: status: %08x\n", status_reg);
#endif

  // Clear control2
  mmio_write (EMMC_BASE + EMMC_CONTROL2, 0);

  // Get the base clock rate
  uint32_t base_clock = sd_get_base_clock_hz ();
  if (base_clock == 0)
    {
      printf ("EMMC: assuming clock rate to be 100MHz\n");
      base_clock = 100000000;
    }

  // Set clock rate to something slow
#ifdef DEBUG
  printf ("EMMC: setting clock rate\n");
#endif
  control1 = mmio_read (EMMC_BASE + EMMC_CONTROL1);
  control1 |= 1;		// enable clock

  // Set to identification frequency (400 kHz)
  uint32_t f_id = 0x3ff;	//fix for crc failure when using strange cable, max divider == minimum clock rate //sd_get_clock_divider (base_clock, SD_CLOCK_ID);
  if (f_id == SD_GET_CLOCK_DIVIDER_FAIL)
    {
      printf ("EMMC: unable to get a valid clock divider for ID frequency\n");
      return -1;
    }
  control1 |= f_id;

  control1 |= (15 << 16);	//(7 << 16);                // data timeout = TMCLK * 2^10 //HAIL MARRY
  mmio_write (EMMC_BASE + EMMC_CONTROL1, control1);
  TIMEOUT_WAIT (mmio_read (EMMC_BASE + EMMC_CONTROL1) & 0x2, 0x1000000);
  if ((mmio_read (EMMC_BASE + EMMC_CONTROL1) & 0x2) == 0)
    {
      printf ("EMMC: controller's clock did not stabilise within 1 second\n");
      return -1;
    }
#ifdef DEBUG
  printf ("EMMC: control0: %08x, control1: %08x\n",
	  mmio_read (EMMC_BASE + EMMC_CONTROL0),
	  mmio_read (EMMC_BASE + EMMC_CONTROL1));
#endif

  // Enable the SD clock
#ifdef DEBUG
  printf ("EMMC: enabling SD clock\n");
#endif
  usleep (2000);
  control1 = mmio_read (EMMC_BASE + EMMC_CONTROL1);
  control1 |= 4;
  mmio_write (EMMC_BASE + EMMC_CONTROL1, control1);
  usleep (2000);

  // Mask off sending interrupts to the ARM, we'll poll the interrrupt register ourselves
  mmio_write (EMMC_BASE + EMMC_IRPT_EN, 0);
  // Reset interrupts
  mmio_write (EMMC_BASE + EMMC_INTERRUPT, 0xffffffff);
  // Have all interrupts sent to the INTERRUPT register
  uint32_t irpt_mask = 0xffffffff & (~SD_CARD_INTERRUPT);

  irpt_mask |= SD_CARD_INTERRUPT;

  mmio_write (EMMC_BASE + EMMC_IRPT_MASK, irpt_mask);

  usleep (2000);



  // Prepare the device structure
  struct emmc_block_dev *ret;
  ret = emmc_dev;

  memset (ret, 0, sizeof (struct emmc_block_dev));


  ret->bd.block_size = 1;



  ret->base_clock = base_clock;




/////////////////////////////////////////////////////////////////////////////////////////////////
////MMC init


  printf ("CMD0: idle\n");
  // Send CMD0 to the card (reset to idle state)
  sd_issue_command (ret, GO_IDLE_STATE, 0, 500000);	//"3 2 1  everyone is fast asleep now"
  //print_response_reg(emmc);
  if (FAIL (ret))
    {
      printf ("SD_init: no CMD0 response\n");
      print_response_reg (emmc);
      return -1;
    }

  printf ("CMD1(0) : init and querry OCR\n");
  // Send CMD1 
  sd_issue_command (ret, SEND_OP_COND, 0x0, 1000000);	//"anybody there?"
  uint32_t voltage_range_response = ret->last_r0;
  if (FAIL (ret))
    {
      printf ("SD_init: no CMD1 (0) response\n");
      print_response_reg (emmc);
      return -1;
    }

  usleep (1000000);
  uint32_t counter = 0;
  printf ("CMD1(0x%08X) : repeat untill ready\n", voltage_range_response);
  while ((counter < 23) && (CHECKBIT (ret->last_r0, 31)) == 0)
    {
      sd_issue_command (ret, SEND_OP_COND, voltage_range_response, 1000000);	//"sure, we can handle that voltage spec. no probs"
      counter++;
#ifdef DEBUG
      printf ("iteration %i\n", counter);
#endif
      usleep (1000000);
      if (FAIL (ret))
	{
	  print_response_reg (emmc);
	  printf ("SD_init: no CMD1 (0) response\n");
	  return -1;
	}

    }


  printf ("CMD2: CID and id mode\n");
  // Send CMD2 to get the cards CID
  sd_issue_command (ret, ALL_SEND_CID, 0, 500000);	//"everyone, shout your CID. the one who shouts loudest gets activated"
  if (FAIL (ret))
    {
      printf ("SD_init: error sending ALL_SEND_CID\n");
      return -1;
    }
  uint32_t card_cid_0 = ret->last_r0;
  uint32_t card_cid_1 = ret->last_r1;
  uint32_t card_cid_2 = ret->last_r2;
  uint32_t card_cid_3 = ret->last_r3;


  printf
    ("\nWarning! The CID is an unique serialnumber which might be traceable. Do not publish it in any way!\n");
  printf ("\tCID: %08X%08X%08X%08X\n\n", card_cid_3, card_cid_2, card_cid_1,
	  card_cid_0);

  uint32_t *dev_id = (uint32_t *) malloc (4 * sizeof (uint32_t));
  dev_id[3] = card_cid_3;
  dev_id[2] = card_cid_2;
  dev_id[1] = card_cid_1;
  dev_id[0] = card_cid_0;

  ret->bd.device_id = (uint8_t *) dev_id;
  ret->bd.dev_id_len = 4 * sizeof (uint32_t);

  ret->card_rca = 0xBEEF;

  printf ("CMD3: assign RCA and standby mode\n");
  // Send CMD3 to assign rca andenter the data state
  sd_issue_command (ret, SEND_RELATIVE_ADDR, ret->card_rca << 16, 500000);	//"you there who shouted loudest! from now on i'll call you beef!"

  if (FAIL (ret))
    {
      printf ("SD_init: error sending SEND_RELATIVE_ADDR\n");
      print_response_reg (emmc);
      return -1;
    }

  uint32_t cmd3_resp = ret->last_r0;

#ifdef DEBUG
  printf ("SD_init: CMD3 response: %08x\n", cmd3_resp);
#endif


  uint32_t crc_error = (cmd3_resp >> 15) & 0x1;
  uint32_t illegal_cmd = (cmd3_resp >> 14) & 0x1;
  uint32_t error = (cmd3_resp >> 13) & 0x1;
  uint32_t status = (cmd3_resp >> 9) & 0xf;
  uint32_t ready = (cmd3_resp >> 8) & 0x1;

  if (crc_error)
    {
      printf ("SD_init: CRC error\n");
      free (dev_id);
      return -1;
    }

  if (illegal_cmd)
    {
      printf ("SD_init: illegal command\n");
      free (dev_id);
      return -1;
    }

  if (error)
    {
      printf ("SD_init: generic error\n");
      free (dev_id);
      return -1;
    }

  if (!ready)
    {
      printf ("SD_init: not ready for data\n");
      free (dev_id);
      return -1;
    }

#ifdef DEBUG
  printf ("SD_init: RCA: %04x\n", ret->card_rca);
#endif

  // Send CMD9 to get the cards CSD
  printf ("CMD9: get CSD\n");
  sd_issue_command (ret, SEND_CSD, ret->card_rca << 16, 500000);	//"hey beef, tell me more about you"

  if (FAIL (ret))
    {
      printf ("SD_init: error sending SEND_CSD\n");
      print_response_reg (emmc);
      return -1;
    }
  uint32_t card_csd_0 = ret->last_r0;
  uint32_t card_csd_1 = ret->last_r1;
  uint32_t card_csd_2 = ret->last_r2;
  uint32_t card_csd_3 = ret->last_r3;

  uint32_t *dev_csd = (uint32_t *) malloc (5 * sizeof (uint32_t)); //extra space on the end
  dev_csd[4] = 0;
  dev_csd[3] = byte_swap(card_csd_0);
  dev_csd[2] = byte_swap(card_csd_1);
  dev_csd[1] = byte_swap(card_csd_2);
  dev_csd[0] = byte_swap(card_csd_3);

  printf ("\n\tCSD: %08X%08X%08X%08X\n\n", card_csd_3, card_csd_2, card_csd_1,
	  card_csd_0);



  // Now select the card (toggles it to transfer state)
  printf ("CMD7: switch to transfer mode\n");
  sd_issue_command (ret, SELECT_CARD, ret->card_rca << 16, 500000);	//"ok beef, get ready to rumble"

  if (FAIL (ret))
    {
      printf ("SD_init: error sending CMD7\n");
      print_response_reg (emmc);
      return -1;
    }

  uint32_t cmd7_resp = ret->last_r0;
  status = (cmd7_resp >> 9) & 0xf;

  if ((status != 3) && (status != 4))
    {
      printf ("SD_init: invalid status (%i)\n", status);
      print_response_reg (emmc);
      free (dev_id);
      return -1;
    }


  printf ("CMD13: get status register\n");
  sd_issue_command (ret, SEND_STATUS, ret->card_rca << 16, 500000);	//"beef, what are you up to right now"

  if (FAIL (ret))
    {
      printf ("SD_init: error sending CMD13\n");
      print_response_reg (emmc);
      return -1;
    }

  printf ("MMC status: 0x%08X\n", ret->last_r0);
  printf ("\n\tMMC is %slocked.\n\n",
	  (CHECKBIT (ret->last_r0, 25) == 1) ? "" : "not ");



dev_csd = (uint32_t*)(((uint8_t*)dev_csd) + 1);//yeah, now it makes sense. i've got no clue why this happens, though -.- also i isued scheme for about five years, so i like brackets more then operator precedence

/*
printf("DEBUG: CSD: ");
for (int i = 0; i<16;++i) printf("%02X", ((uint8_t*)dev_csd)[i]); 
printf("\n");

printf("DEBUG: CID: ");
for (int i = 0; i<16;++i) printf("%02X", ((uint8_t*)dev_id)[i]);
printf("\n");
*/

////////////////////////////////////////////////////////////////////////////////////////
//force erase



  if (('F' == mode) && (CHECKBIT (ret->last_r0, 25) == 1))
    {

      // CMD16: block length 1
      printf ("CMD16: setting blocklength to 1\n");
      sd_issue_command (ret, SET_BLOCKLEN, 1, 500000);	//"beef, if i give you extra commands after an instruction they will only contain one word"

      if (FAIL (ret))
	{
	  print_response_reg (emmc);
	  printf ("SD_init: error sending SET_BLOCKLEN\n");
	  return -1;
	}

      ret->block_size = 1;
      controller_block_size = mmio_read (EMMC_BASE + EMMC_BLKSIZECNT);
      controller_block_size &= (~0xfff);
      controller_block_size |= 0x1;
      mmio_write (EMMC_BASE + EMMC_BLKSIZECNT, controller_block_size);


      uint8_t force_erase_payload[513];
      memset (force_erase_payload, 0, 513);
      force_erase_payload[0] = 0b00001000;

      ret->buf = &force_erase_payload;
      ret->blocks_to_transfer = 1;

      printf ("CMD42: force erase\n");
      int retry_count = 0;
      int max_retries = 1;
      while (retry_count < max_retries)
	{

	  sd_issue_command (ret, LOCK_UNLOCK, ret->card_rca << 16, 180000000);	//"beef, forget everything you know, you are no longer protected"

	  if (SUCCESS (ret))
	    break;
	  else
	    {
	      printf ("SD_init: error sending CMD%i, ", LOCK_UNLOCK);
	      printf ("error = %08x. ", ret->last_error);
	      retry_count++;
	      if (retry_count < max_retries)
		printf ("Retrying...\n");
	      else
		printf ("Giving up.\n");
	    }
	}
      if (retry_count == max_retries)
	{
	  ret->card_rca = 0;
	  return -1;
	}

      printf ("CMD13: get status register\n");
      sd_issue_command (ret, SEND_STATUS, ret->card_rca << 16, 500000);	//"beef, what are you up to right now"

      if (FAIL (ret))
	{
	  printf ("SD_init: error sending CMD13\n");
	  print_response_reg (emmc);
	  return -1;
	}

      printf ("MMC status: 0x%08X\n", ret->last_r0);
      printf ("\n\tMMC is %slocked.\n\n",
	      (CHECKBIT (ret->last_r0, 25) == 1) ? "" : "not ");
    }

///////////////////////////////////////////////////////////////////////////////////////////
//lock

  if ('L' == mode)
    {
      int retry_count = 0;
      int max_retries = 1;

//      printf("locking only in the debug version\n");
//      return -1;

      // CMD16: block length 16
      printf ("CMD16: setting blocklength to 16\n");
      sd_issue_command (ret, SET_BLOCKLEN, 16, 500000);	//"beef, if i give you extra commands after an instruction they will contain three words"

      if (FAIL (ret))
	{
	  print_response_reg (emmc);
	  printf ("SD_init: error sending SET_BLOCKLEN\n");
	  return -1;
	}

      ret->block_size = 16;
      controller_block_size = mmio_read (EMMC_BASE + EMMC_BLKSIZECNT);
      controller_block_size &= (~0xfff);
      controller_block_size |= 16;
      mmio_write (EMMC_BASE + EMMC_BLKSIZECNT, controller_block_size);


///////////////////////WP on
      ret->blocks_to_transfer = 1;
      ret->buf = (uint8_t *)dev_csd;
       
	   ((uint8_t *) ret->buf)[14] |= 0x10; //bit twelve counted from the right and 0
	   
	   if(((uint8_t *) ret->buf)[14] & 0x20 != 0){
	      printf("perm write protect would be set. bailing out\n");
	      return -1;
	   }
/*
printf("DEBUG: CSD with temp write set: ");
for (int i = 0; i<16;++i) printf("%02X", ((uint8_t*)dev_csd)[i]); 
printf("\n");*/

      
         printf ("CMD27: write CSD\n");
         retry_count = 0;
         max_retries = 1;
         while (retry_count < max_retries)
         {

         sd_issue_command (ret, PROGRAM_CSD, ret->card_rca << 16, 180000000);       //"beef, accept this new CSD"

         if (SUCCESS (ret))
         break;
         else
         {
         printf ("SD_init: error sending CMD%i, ", PROGRAM_CSD);
         printf ("error = %08x. ", ret->last_error);
         retry_count++;
         if (retry_count < max_retries)
         printf ("Retrying...\n");
         else
         printf ("Giving up.\n");
         }
         }
         if (retry_count == max_retries)
         {
         ret->card_rca = 0;
         return -1;
         }
         //print_response_reg (emmc);

       


///////////////////lock


      ret->buf = (uint8_t *) dev_id;
      uint32_t key[4];
      key[0] = 0x17C6987E;
      key[1] = 0x4401EDDE;
      key[2] = 0x371AC568;
      key[3] = 0x65FFB562;

      for (int i = 0; i <= 3; ++i)
	key[i] = byte_swap (key[i]);



      ((uint32_t *) ret->buf)[0] ^= key[0];
      ((uint32_t *) ret->buf)[1] ^= key[1];
      ((uint32_t *) ret->buf)[2] ^= key[2];
      ((uint32_t *) ret->buf)[3] ^= key[3];

      ((uint8_t *) ret->buf)[0] = 0b00000101;	//set password and lock
      ((uint8_t *) ret->buf)[1] = 14;	//14 byte password

      ret->blocks_to_transfer = 1;

      printf ("CMD42: set pw and lock\n");
      retry_count = 0;
      max_retries = 1;
      while (retry_count < max_retries)
	{

	  sd_issue_command (ret, LOCK_UNLOCK, ret->card_rca << 16, 180000000);	//"beef, take this password and be locked now"

	  if (SUCCESS (ret))
	    break;
	  else
	    {
	      printf ("SD_init: error sending CMD%i, ", LOCK_UNLOCK);
	      printf ("error = %08x. ", ret->last_error);
	      retry_count++;
	      if (retry_count < max_retries)
		printf ("Retrying...\n");
	      else
		printf ("Giving up.\n");
	    }
	}
      if (retry_count == max_retries)
	{
	  ret->card_rca = 0;
	  return -1;
	}
      printf ("CMD13: get status register\n");
      sd_issue_command (ret, SEND_STATUS, ret->card_rca << 16, 500000);	//"beef, what are you up to right now"

      if (FAIL (ret))
	{
	  printf ("SD_init: error sending CMD13\n");
	  print_response_reg (emmc);
	  return -1;
	}

      printf ("MMC status: 0x%08X\n", ret->last_r0);
      printf ("\n\tMMC is %slocked.\n\n",
	      (CHECKBIT (ret->last_r0, 25) == 1) ? "" : "not ");


    }
///////////////////////////////////////////////////////////////////////////////////////////
//unlock

  if ('U' == mode)
    {

      // CMD16: block length 16
      printf ("CMD16: setting blocklength to 16\n");
      sd_issue_command (ret, SET_BLOCKLEN, 16, 500000);	//"beef, if i give you extra commands after an instruction they will contain sixteen words"

      if (FAIL (ret))
	{
	  print_response_reg (emmc);
	  printf ("SD_init: error sending SET_BLOCKLEN\n");
	  return -1;
	}

      ret->block_size = 16;
      controller_block_size = mmio_read (EMMC_BASE + EMMC_BLKSIZECNT);
      controller_block_size &= (~0xfff);
      controller_block_size |= 16;
      mmio_write (EMMC_BASE + EMMC_BLKSIZECNT, controller_block_size);


/////////unlock

      ret->buf = (uint8_t *) dev_id;

      uint32_t key[4];
      key[0] = 0x17C6987E;
      key[1] = 0x4401EDDE;
      key[2] = 0x371AC568;
      key[3] = 0x65FFB562;

      for (int i = 0; i <= 3; ++i)
	key[i] = byte_swap (key[i]);	//byteorder is fun fun fun, in the sun sun sun

      ((uint32_t *) ret->buf)[0] ^= key[0];
      ((uint32_t *) ret->buf)[1] ^= key[1];
      ((uint32_t *) ret->buf)[2] ^= key[2];
      ((uint32_t *) ret->buf)[3] ^= key[3];

      ((uint8_t *) ret->buf)[0] = 0b00000010;	//remove password
      ((uint8_t *) ret->buf)[1] = 14;	//14 byte password

/*printf("DEBUG: unlock payload: ");
for (int i = 0; i<16;++i) printf("%02X", ((uint8_t*)ret->buf)[i]);
printf("\n");
*/

      ret->blocks_to_transfer = 1;

      printf ("CMD42: unlock and clear password\n");
      int retry_count = 0;
      int max_retries = 1;
      while (retry_count < max_retries)
	{

	  sd_issue_command (ret, LOCK_UNLOCK, ret->card_rca << 16, 180000000);	//"beef, heres the password, if its right unlock and forget the password"

	  if (SUCCESS (ret))
	    break;
	  else
	    {
	      printf ("SD_init: error sending CMD%i, ", LOCK_UNLOCK);
	      printf ("error = %08x. ", ret->last_error);
	      retry_count++;
	      if (retry_count < max_retries)
		printf ("Retrying...\n");
	      else
		printf ("Giving up.\n");
	    }
	}
      if (retry_count == max_retries)
	{
	  ret->card_rca = 0;
	  return -1;
	}
	
///////////////////////WP off
      ret->blocks_to_transfer = 1;
      ret->buf = (uint8_t *)dev_csd;
       
	   ((uint8_t *) ret->buf)[14] &= ~(0x10); //bit twelve counted from the right and 0


	   if(((uint8_t *) ret->buf)[14] & 0x20 != 0){
	     printf("perm write protect would be set. bailing out\n");
	     return -1;
	   }
/*printf("DEBUG: CSD with temp write prot unset: ");
for (int i = 0; i<16;++i) printf("%02X", ((uint8_t*)dev_csd)[i]); 
printf("\n");*/

      
         printf ("CMD27: write CSD\n");
         retry_count = 0;
         max_retries = 1;
         while (retry_count < max_retries)
         {

         sd_issue_command (ret, PROGRAM_CSD, ret->card_rca << 16, 180000000);       //"beef, accept this new CSD"

         if (SUCCESS (ret))
         break;
         else
         {
         printf ("SD_init: error sending CMD%i, ", PROGRAM_CSD);
         printf ("error = %08x. ", ret->last_error);
         retry_count++;
         if (retry_count < max_retries)
         printf ("Retrying...\n");
         else
         printf ("Giving up.\n");
         }
         }
         if (retry_count == max_retries)
         {
         ret->card_rca = 0;
         return -1;
         }
         //print_response_reg (emmc);

      printf ("CMD13: get status register\n");
      sd_issue_command (ret, SEND_STATUS, ret->card_rca << 16, 500000);	//"beef, what are you up to right now"

      if (FAIL (ret))
	{
	  printf ("SD_init: error sending CMD13\n");
	  print_response_reg (emmc);
	  return -1;
	}

      printf ("MMC status: 0x%08X\n", ret->last_r0);
      printf ("\n\tMMC is %slocked.\n\n",
	      (CHECKBIT (ret->last_r0, 25) == 1) ? "" : "not ");



    }

///////////////////////////////////////////////////////////////////////////////////////////
//remove_writeprotect //ugly hack for spacejump

  if ('R' == mode)
    {

      // CMD16: block length 16
      printf ("CMD16: setting blocklength to 16\n");
      sd_issue_command (ret, SET_BLOCKLEN, 16, 500000);	//"beef, if i give you extra commands after an instruction they will contain sixteen words"

      if (FAIL (ret))
	{
	  print_response_reg (emmc);
	  printf ("SD_init: error sending SET_BLOCKLEN\n");
	  return -1;
	}

      ret->block_size = 16;
      controller_block_size = mmio_read (EMMC_BASE + EMMC_BLKSIZECNT);
      controller_block_size &= (~0xfff);
      controller_block_size |= 16;
      mmio_write (EMMC_BASE + EMMC_BLKSIZECNT, controller_block_size);



///////////////////////WP off
      ret->blocks_to_transfer = 1;
      ret->buf = (uint8_t *)dev_csd;
       
	   ((uint8_t *) ret->buf)[14] &= ~(0x10); //bit twelve counted from the right and 0


	   if(((uint8_t *) ret->buf)[14] & 0x20 != 0){
	     printf("perm write protect would be set. bailing out\n");
	     return -1;
	   }
/*printf("DEBUG: CSD with temp write prot unset: ");
for (int i = 0; i<16;++i) printf("%02X", ((uint8_t*)dev_csd)[i]); 
printf("\n");*/

      
         printf ("CMD27: write CSD\n");
         int retry_count = 0;
         int max_retries = 1;
         while (retry_count < max_retries)
         {

         sd_issue_command (ret, PROGRAM_CSD, ret->card_rca << 16, 180000000);       //"beef, accept this new CSD"

         if (SUCCESS (ret))
         break;
         else
         {
         printf ("SD_init: error sending CMD%i, ", PROGRAM_CSD);
         printf ("error = %08x. ", ret->last_error);
         retry_count++;
         if (retry_count < max_retries)
         printf ("Retrying...\n");
         else
         printf ("Giving up.\n");
         }
         }
         if (retry_count == max_retries)
         {
         ret->card_rca = 0;
         return -1;
         }
         //print_response_reg (emmc);

      printf ("CMD13: get status register\n");
      sd_issue_command (ret, SEND_STATUS, ret->card_rca << 16, 500000);	//"beef, what are you up to right now"

      if (FAIL (ret))
	{
	  printf ("SD_init: error sending CMD13\n");
	  print_response_reg (emmc);
	  return -1;
	}

      printf ("MMC status: 0x%08X\n", ret->last_r0);
      printf ("\n\tMMC is %slocked.\n\n",
	      (CHECKBIT (ret->last_r0, 25) == 1) ? "" : "not ");



    }

  // Reset interrupt register
  mmio_write (EMMC_BASE + EMMC_INTERRUPT, 0xffffffff);

  emmc_dev = ret;

  return 0;
}



static int
sd_ensure_data_mode (struct emmc_block_dev *edev, char spi)
{
  if (edev->card_rca == 0)
    {
      // Try again to initialise the card
      int ret = sd_card_init (edev, spi, 0);
      if (ret != 0)
	return ret;
    }

  printf
    ("SD: ensure_data_mode() obtaining status register for card_rca %08x: ",
     edev->card_rca);

  sd_issue_command (edev, SEND_STATUS, edev->card_rca << 16, 500000);
  if (FAIL (edev))
    {
      printf ("SD: ensure_data_mode() error sending CMD13\n");
      edev->card_rca = 0;
      return -1;
    }

  uint32_t status = edev->last_r0;
  uint32_t cur_state = (status >> 9) & 0xf;

  printf ("status %i\n", cur_state);

  if (cur_state == 3)
    {
      // Currently in the stand-by state - select it
      sd_issue_command (edev, SELECT_CARD, edev->card_rca << 16, 500000);
      if (FAIL (edev))
	{
	  printf ("SD: ensure_data_mode() no response from CMD17\n");
	  edev->card_rca = 0;
	  return -1;
	}
    }
  else if (cur_state == 5)
    {
      // In the data transfer state - cancel the transmission
      sd_issue_command (edev, STOP_TRANSMISSION, 0, 500000);
      if (FAIL (edev))
	{
	  printf ("SD: ensure_data_mode() no response from CMD12\n");
	  edev->card_rca = 0;
	  return -1;
	}

      // Reset the data circuit
      sd_reset_dat ();
    }
  else if (cur_state != 4)
    {
      // Not in the transfer state - re-initialise
      int ret = sd_card_init (edev, spi, 0);
      if (ret != 0)
	return ret;
    }

  // Check again that we're now in the correct mode
  if (cur_state != 4)
    {

      printf ("SD: ensure_data_mode() rechecking status: ");
      sd_issue_command (edev, SEND_STATUS, edev->card_rca << 16, 500000);
      if (FAIL (edev))
	{
	  printf ("SD: ensure_data_mode() no response from CMD13\n");
	  edev->card_rca = 0;
	  return -1;
	}
      status = edev->last_r0;
      cur_state = (status >> 9) & 0xf;


      printf ("%i\n", cur_state);


      if (cur_state != 4)
	{
	  printf ("SD: unable to initialise SD card to "
		  "data mode (state %i)\n", cur_state);
	  edev->card_rca = 0;
	  return -1;
	}
    }

  return 0;
}

static int
sd_do_data_command (struct emmc_block_dev *edev, int is_write, uint8_t * buf,
		    size_t buf_size, uint32_t block_no)
{


  // This is as per HCSS 3.7.2.1
  if (buf_size < edev->block_size)
    {
      printf ("SD: do_data_command() called with buffer size (%i) less than "
	      "block size (%i)\n", buf_size, edev->block_size);
      return -1;
    }

  edev->blocks_to_transfer = buf_size / edev->block_size;
  if (buf_size % edev->block_size)
    {
      printf ("SD: do_data_command() called with buffer size (%i) not an "
	      "exact multiple of block size (%i)\n", buf_size,
	      edev->block_size);
      return -1;
    }
  edev->buf = buf;

  // Decide on the command to use
  int command;
  if (is_write)
    {
      if (edev->blocks_to_transfer > 1)
	command = WRITE_MULTIPLE_BLOCK;
      else
	command = WRITE_BLOCK;
    }
  else
    {
      if (edev->blocks_to_transfer > 1)
	command = READ_MULTIPLE_BLOCK;
      else
	command = READ_SINGLE_BLOCK;
    }

  int retry_count = 0;
  int max_retries = 3;
  while (retry_count < max_retries)
    {

      sd_issue_command (edev, command, block_no, 5000000);

      if (SUCCESS (edev))
	break;
      else
	{
	  printf ("SD: error sending CMD%i, ", command);
	  printf ("error = %08x. ", edev->last_error);
	  retry_count++;
	  if (retry_count < max_retries)
	    printf ("Retrying...\n");
	  else
	    printf ("Giving up.\n");
	}
    }
  if (retry_count == max_retries)
    {
      edev->card_rca = 0;
      return -1;
    }

  return 0;
}


void
view_register (uint32_t * emmc)
{
  char in;

  while (toupper (in) != 'Q')
    {

      printf
	("(C)MDTM | (A)RG1 | (R)esponse | (S)tatus | (I)nterrupt | Control(0-2) | (Q)uit\n");

      scanf (" %c", &in);
      printf ("\n");

      switch (toupper (in))
	{
	case 'C':
	  print_cmdtm_reg (emmc);
	  continue;
	case 'A':
	  print_arg1_reg (emmc);
	  continue;
	case 'R':
	  print_response_reg (emmc);
	  continue;
	case 'S':
	  print_status_reg (emmc);
	  continue;
	case 'I':
	  print_interrupt_reg (emmc);
	  continue;
	case '0':
	  print_control0_reg (emmc);
	  continue;
	case '1':
	  print_control1_reg (emmc);
	  continue;
	case '2':
	  print_control2_reg (emmc);
	  continue;
	case 'Q':
	  break;
	default:
	  continue;
	}
    }


}


void
force_erase (struct emmc_block_dev *emmc_dev)
{
  printf
    ("WARNING: This will permanently erase the (e)MMC aka. NAND!\n\nContinue only if you do have a NAND dump of your 3DS!\n\n Enter H (big h) to continue.\n");
  char in;
  scanf (" %c", &in);
  if ('H' == in)
    sd_card_init (emmc_dev, 'F', 0);
  return;
}

void
lock (struct emmc_block_dev *emmc_dev)
{
  printf
    ("WARNING: This will lock the (e)MMC!\n\n Enter H (big h) to continue.\n");
  char in;
  scanf (" %c", &in);
  if ('H' == in)
    sd_card_init (emmc_dev, 'L', 0);
  return;
}

void
unlock (struct emmc_block_dev *emmc_dev)
{
  sd_card_init (emmc_dev, 'U', 0);
  return;
}

void
removeprotect (struct emmc_block_dev *emmc_dev)
{
  sd_card_init (emmc_dev, 'R', 0);
  return;
}


void
dedication ()
{
  printf
    ("You might ask: \"bkifft, why the fuck are you dedicating this tool to the user crazyace2011?\" \n\n");
  printf
    ("Easy: this shithead gave me the spite fuelled energy to write it.\n\n");
  printf
    ("Quotes from http://gbatemp.net/threads/has-anyone-with-a-brick-been-able-to-recover.360647/:\n\n\"im trying to understand something everyone is spitting out information that they truly don't know. the emmc is wiped or locked the nand is wiped out. no one has the hardware to know 100% but everyone is talking like they know if you knew you would have a way of fixing not just talking about whats wrong. people are just claiming to know what is wrong when they don't have the equipment back up the theory.\"\n\n");
  printf
    ("\"im not saying you per say im just saying that everyone is talking like they are Einstein and know what is going on. who's to say that something is blocking the emmc controller not the emmc controller itself I don't know what it could be but im not throwing out stuff. im not ranting that you are doing it. its just people say something tech about the insides of a 3ds but the thing is no one know whats going on inside the 3ds and what the brick code actually did to the unit. yes we know that the system isn't responding to the nand that was installed by Nintendo but we don't know exactly what the gateway brick code did.\"\n\n");
  printf
    ("\"I already said that I don't know how but you smart ass people think you know but honestly you don't know shit about it either. no one said you had to answer to my comment so stfu and ignore my post\"\n\n");
  printf
    ("\"like we need more pointless 3ds brick threads real mature. must be a bunch of little kids that think they know everything. typical\"\n\n");
  printf
    ("Quote from http://www.maxconsole.com/maxcon_forums/threads/280010-Update-on-RMAing-my-3DS?p=1671397#post1671397:\n\n\"im on gbatemp and there is a bunch of little kids that think they know everything and every theory. its like when a child tells a parent I know I know I know gets annoying\"\n");
  printf
    ("\n\nAnyway, true shoutout to my man inian who played my brick guinea pig and all the fellas who gave constructive feedback on the \"Has anyone with a brick been able to recover\" thread, you know who you are.\n\n");
  printf ("Big thanks to the anonymous donor for the Vernam cipher key.\n\n");
}


int
main ()
{



  int fd;
  uint32_t *gpio;
  void *peri_base;
  int i;
  int counter;
  int cmd;
  uint32_t arg;
  char in;



  fd = open ("/dev/mem", O_RDWR | O_SYNC);
  if (0 > fd)
    {
      perror ("Error opening /dev/mem");
      printf
	("This tool needs super user rights. Run it as user root or use sudo\n");
      exit (EXIT_FAILURE);
    }

  if ((opendir ("/sys/bus/mmc/")) || (ENOENT != errno))
    {
      printf
	("It seems the MMC/SD drivers are loaded. Please boot a kernel without them.\n");
      exit (EXIT_FAILURE);
    }



  printf ("\e[1;1H\e[2J");
  printf("\n\nNew enhanced formula: true unbrick instead of a force erase. (May contain nuts or GW secret keys.)\n");
  printf
    ("\n                  Dedicated to crazyace2011 @GBAtemp (crazyace @maxconsole)\n");
  printf
    ("                  and every other elderly person who can count to potato.\n\n");
  printf ("\nThis tool is erotic cartoon ware.\n");
  printf
    ("If you like it please send one erotic cartoon picture to \nrpu.bkifft.gbatemp@gmail.com (even if you draw one in paint yourself, everyone likes to draw the cock and balls).\n");

  printf ("\nA statement from our sponsor (who gave me the Unlock key):\n");
  printf
    ("\"If you are reading this your 3DS has most likely been bricked by a Virus called Gateway 3DS.\n");
  printf ("If so return it and get a refund immediately.\n");
  printf
    ("Because what they have done is they made a soft-mod for the 3DS but then decided\n");
  printf ("that they would earn more money if they added their own AP.\n");
  printf
    ("They also added a lot of obfuscation (to prevent pirates from pirating their card and software),\n");
  printf
    ("which most likely also is the reason why some versions are not stable (and the brick code is triggered).\n");
  printf
    ("And as you already see on your 3DS they added brick code in the 2.0_2b Version.\n");
  printf
    ("This brick code is not even written correctly (else this unbricker wouldn't work).\n");
  printf ("So they even failed at programming brick code.\n");
  printf ("\nTo sum it all up you bought a badly programmed Virus.\n");
  printf
    ("\nBuy your games, don't pirate them. You see what happens when you pirate.\n");
  printf ("I hope you learned from your mistake.\"\n");
  printf
    ("\n\n\n\nWARNING: Do not run this tool with a kernel that has the MMC/SD subsystem enabled!\n");
  printf
    ("\(The Mini Linux Image comes without them and the tool should refuse to run if the drivers are loaded)\n");

  printf ("\nWill continue in 10 seconds.\n");

  sleep (10);



  peri_base =
    mmap (NULL, BLOCKSIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
	  BCM2708_PERI_BASE);

  if ((void *) -1 == peri_base)
    {
      perror ("Error while mapping");
      exit (EXIT_FAILURE);
    }



  emmc = peri_base + EMMC_OFFSET;
  gpio = peri_base + GPIO_OFFSET;


  struct emmc_block_dev emmc_device;



  in = '\0';

  while (toupper (in) != 'Q')
    {

    //printf	("\n(D)edication | (S)afe run (Querry only) | (U)nlock (Safe) | (L)ock (Dangerous!) | (F)orce erase (Dangerous!) | (Q)uit\n");
	
	  printf
	("\n(D)edication | (S)afe run (Querry only) | (U)nbrick (Safe) | (Q)uit\n");


      scanf (" %c", &in);
      printf ("\n");

      switch (toupper (in))
	{
	case 'S':
	  sd_card_init (&emmc_device, '0', 0);;
	  continue;
	case 'F':
	  force_erase (&emmc_device);
	  continue;
	case 'L':
	  lock (&emmc_device);
	  continue;

	case 'U':
	  unlock (&emmc_device);
	  continue;
	case 'R':
	  removeprotect (&emmc_device);
	  continue;
	case 'V':
	  view_register (emmc);
	  continue;
	case 'D':
	  dedication ();
	  continue;
	case 'Q':
	  break;
	default:
	  continue;
	}
    }




  printf ("\n");
  if (0 > munmap (peri_base, BLOCKSIZE))
    {
      perror ("munmap failed:");
      exit (EXIT_FAILURE);
    }
}
