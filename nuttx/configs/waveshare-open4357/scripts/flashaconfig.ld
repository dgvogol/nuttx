/****************************************************************************
 * configs/waveshare-open4357/scripts/flashaconfig.ld
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2014 David G <david.guyueyi@gmail.com>
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/*
 * Power-Up Reset Overview
 * -----------------------
 *
 * The ARM core starts executing code on reset with the program counter set
 * to 0x0000 0000.  The LPC43xx contains a shadow pointer register that
 * allows areas of memory to be mapped to address 0x0000 0000. The default,
 * reset value of the shadow pointer is 0x1040 0000 so that on reset code in
 * the boot ROM is always executed first.
 *
 * The boot starts after reset is released.  The IRC is selected as CPU clock
 * and the Cortex-M4 starts the boot loader. By default the JTAG access to the
 * chip is disabled at reset.  The boot ROM determines the boot mode based on
 * the OTP BOOT_SRC value or reset state pins.  For flash-based parts, the part
 * boots from internal flash by default.  Otherwse, the boot ROM copies the
 * image to internal SRAM at location 0x1000 0000, sets the ARM's shadow
 * pointer to 0x1000 0000, and jumps to that location.
 *
 * However, using JTAG the executable image can be also loaded directly into
 * and executed from SRAM.
 */

/* The LPC4357 on the Waveshare-Open4357 has the following memory resources:
 *
 * 1. 136KB total internal SRAM:
 *    * 32KB internal local SRAM beginning at address 0x1000:0000
 *    * 40KB internal local SRAM beginning at address 0x1008:0000
 *    * 64KB internal AHB SRAM beginning at address 0x2000:0000
 * 2. 1024KB internal FLASH:
 *    * 512KB FLASH bank A beginning at address 0x1A00:0000
 *    * 512KB FLASH bank B beginning at address 0x1B00:0000
 * 3. Waveshare-Open4357 external memory:
 *    * 4096KB NOR FLASH selected by EMC CS0, at address 0x1C00:0000
 *    * 32MB SDR SDRAM selected by EMC DYCS0, at address 0x2800:0000
 *
 * Here we assume that:
 *
 * 1. We will be running out of internal FLASH bank A at 0x1A00:0000, and
 * 2. All .data and .bss will all fit into the internal SRAM block.
 *
 */

MEMORY
{
    FLASH_A   (rx)  : ORIGIN = 0x1A000000, LENGTH = 512K    /* internal FLASH bank A */
    FLASH_B   (rx)  : ORIGIN = 0x1B000000, LENGTH = 512K    /* internal FLASH bank B */
    FLASH_E   (rx)  : ORIGIN = 0x1C000000, LENGTH = 4096K   /* external NOR FLASH    */
    ISRAM_L   (rwx) : ORIGIN = 0x10000000, LENGTH = 32K     /* local SRAM bank 1     */
    ISRAM_H   (rwx) : ORIGIN = 0x10080000, LENGTH = 40K     /* local SRAM bank 2     */
    ISRAM_AHB (rwx) : ORIGIN = 0x20000000, LENGTH = 64K     /* AHB SRAM              */
    SDRAM   (rwx)   : ORIGIN = 0x28000000, LENGTH = 32M     /* external SDR SDRAM    */
}

OUTPUT_ARCH(arm)
ENTRY(__start)         /* Treat __start as the anchor for dead code stripping */
EXTERN(_vectors)       /* Force the vectors to be included in the output */
SECTIONS
{
	.text : {
		_stext = ABSOLUTE(.);
		*(.vectors)
		*(.text .text.*)
		*(.fixup)
		*(.gnu.warning)
		*(.rodata .rodata.*)
		*(.gnu.linkonce.t.*)
		*(.glue_7)
		*(.glue_7t)
		*(.got)
		*(.gcc_except_table)
		*(.gnu.linkonce.r.*)
		_etext = ABSOLUTE(.);
	} > FLASH_A

	.init_section : {
		_sinit = ABSOLUTE(.);
		*(.init_array .init_array.*)
		_einit = ABSOLUTE(.);
	} > FLASH_A

	.ARM.extab : {
		*(.ARM.extab*)
	} > FLASH_A

	__exidx_start = ABSOLUTE(.);
	.ARM.exidx : {
		*(.ARM.exidx*)
	} > FLASH_A
	__exidx_end = ABSOLUTE(.);

	_eronly = ABSOLUTE(.);

	.data : {
		_sdata = ABSOLUTE(.);
		*(.data .data.*)
		*(.gnu.linkonce.d.*)
		CONSTRUCTORS
		_edata = ABSOLUTE(.);
	} > ISRAM_L AT > FLASH_A

	.bss : {			/* BSS				*/
		_sbss = ABSOLUTE(.);
		*(.bss .bss.*)
		*(.gnu.linkonce.b.*)
		*(COMMON)
		_ebss = ABSOLUTE(.);
	} > ISRAM_L
					/* Stabs debugging sections.	*/
	.stab 0 : { *(.stab) }
	.stabstr 0 : { *(.stabstr) }
	.stab.excl 0 : { *(.stab.excl) }
	.stab.exclstr 0 : { *(.stab.exclstr) }
	.stab.index 0 : { *(.stab.index) }
	.stab.indexstr 0 : { *(.stab.indexstr) }
	.comment 0 : { *(.comment) }
	.debug_abbrev 0 : { *(.debug_abbrev) }
	.debug_info 0 : { *(.debug_info) }
	.debug_line 0 : { *(.debug_line) }
	.debug_pubnames 0 : { *(.debug_pubnames) }
	.debug_aranges 0 : { *(.debug_aranges) }
}
