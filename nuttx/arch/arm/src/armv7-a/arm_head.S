/****************************************************************************
 * arch/arm/src/armv7-a/arm_head.S
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/board/board.h>

#include "arm.h"
#include "cp15.h"
#include "sctlr.h"
#include "mmu.h"
#include "chip.h"
#include "up_internal.h"

	.file	"arm_head.S"

/**********************************************************************************
 * Configuration
 **********************************************************************************/

#undef ALIGNMENT_TRAP
#undef CPU_CACHE_ROUND_ROBIN
#undef CPU_DCACHE_DISABLE
#undef CPU_ICACHE_DISABLE

/* There are three operational memory configurations:
 *
 * 1. We execute in place in FLASH (CONFIG_BOOT_RUNFROMFLASH=y).  In this case
 *    the boot logic must:
 *
 *    - Configure SDRAM,
 *    - Initialize the .data section in RAM, and
 *    - Clear .bss section
 */

#ifdef CONFIG_BOOT_RUNFROMFLASH
#  define DO_SDRAM_INIT 1

  /* Check for the identity mapping:  For this configuration, this would be
   * the case where the virtual beginning of FLASH is the same as the physical
   * beginning of FLASH.
   */

#  if !defined(CONFIG_FLASH_START) || !defined(CONFIG_FLASH_VSTART)
#    error "CONFIG_FLASH_START or CONFIG_FLASH_VSTART is not defined"
#  endif

#  if CONFIG_FLASH_START == CONFIG_FLASH_VSTART
#    define CONFIG_IDENTITY_TEXTMAP 1
#  endif

/* 2. We boot in FLASH but copy ourselves to DRAM from better performance.
 *    (CONFIG_BOOT_RUNFROMFLASH=n && CONFIG_BOOT_COPYTORAM=y).  In this case
 *    the boot logic must:
 *
 *    - Configure SDRAM,
 *    - Copy ourself to DRAM (after mapping it), and
 *    - Clear .bss section
 *
 *   In this case, we assume that the logic within this file executes from FLASH.
 */

#elif defined(CONFIG_BOOT_COPYTORAM)
#  error "configuration not implemented
#  define DO_SDRAM_INIT 1

  /* Check for the identity mapping:  For this configuration, this would be
   * the case where the virtual beginning of FLASH is the same as the physical
   * beginning of FLASH.
   */

#  if !defined(CONFIG_FLASH_START) || !defined(CONFIG_FLASH_VSTART)
#    error "CONFIG_FLASH_START or CONFIG_FLASH_VSTART is not defined"
#  endif

#  if CONFIG_FLASH_START == CONFIG_FLASH_VSTART
#    define CONFIG_IDENTITY_TEXTMAP 1
#  endif

/* 3. There is bootloader that copies us to DRAM (but probably not to the beginning)
 *    (CONFIG_BOOT_RUNFROMFLASH=n && CONFIG_BOOT_COPYTORAM=n). In this case SDRAM
 *    was initialized by the boot loader, and this boot logic must:
 *
 *    - Clear .bss section
 */

#else

  /* Check for the identity mapping:  For this configuration, this would be
   * the case where the virtual beginning of RAM is the same as the physical
   * beginning of RAM.
   */

#  if !defined(CONFIG_RAM_START) || !defined(CONFIG_RAM_VSTART)
#    error "CONFIG_RAM_START or CONFIG_RAM_VSTART is not defined"
#  endif

#  if CONFIG_RAM_START == CONFIG_RAM_VSTART
#    define CONFIG_IDENTITY_TEXTMAP 1
#  endif

#endif

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* RX_NSECTIONS determines the number of 1Mb sections to map for the
 * Read/eXecute address region.  This is based on NUTTX_TEXT_SIZE.
 */

#define WR_NSECTIONS ((NUTTX_RAM_SIZE+0x000fffff) >> 20)

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

/* The ARMv7-A L1 page table can be placed at the beginning or at the end of
 * the RAM space.  This decision is based on the placement of the vector
 * area: If the vectors are place in low memory at address 0x0000 0000, then
 * the page table is placed in high memory; if the vectors are placed in
 * high memory at address 0xfff0 0000, then the page table is locating at
 * the beginning of RAM.
 *
 * For the special case where (1) the program executes out of RAM, and (2)
 * the page is located at the beginning of RAM (i.e., the high vector case),
 * then the following macro can easily find the physical address of the
 * section that includes the first part of the text region:  Since the page
 * table is closely related to the NuttX base address in this case, we can
 * convert the page table base address to the base address of the section
 * containing both.
 */

/* This macro will modify r0, r1, r2 and r14 */

#ifdef CONFIG_DEBUG
	.macro	showprogress, code
	mov		r0, #\code
	bl		up_lowputc
	.endm
#else
	.macro	showprogress, code
	.endm
#endif

/****************************************************************************
 * Name: __start
 ****************************************************************************/

	.text
	.global	__start
	.type	__start, #function

__start:
	/* Make sure that we are in SVC mode with all IRQs disabled */

	mov		r0, #(PSR_MODE_SVC | PSR_I_BIT | PSR_F_BIT)
	msr		cpsr_c, r0

	/* Initialize DRAM using a macro provided by board-specific logic.
	 *
	 * This must be done in two cases:
	 * 1. CONFIG_BOOT_RUNFROMFLASH.  The system is running from FLASH
	 * 2. CONFIG_BOOT_COPYTORAM.  The system booted from FLASH but
	 *    will copy itself to SDRAM.
	 */

#ifdef DO_SDRAM_INIT
	config_sdram
#endif

	/* Clear the 16K level 1 page table */

	ldr		r5, .LCppgtable			/* r5=phys. page table */
#ifndef CONFIG_ARCH_ROMPGTABLE
	mov		r0, r5
	mov		r1, #0
	add		r2, r0, #PGTABLE_SIZE
.Lpgtableclear:
	str		r1, [r0], #4
	str		r1, [r0], #4
	str		r1, [r0], #4
	str		r1, [r0], #4
	teq		r0, r2
	bne		.Lpgtableclear

#ifdef ARMV7A_PGTABLE_MAPPING
	/* If the page table does not lie in the same address space as does the
	 * mapped RAM in either case.  So we will need to create a special
	 * mapping for the page table.
	 *
	 * Load information needed to map the page table.  After the ldmia, we
	 * will have
	 *
	 *   R1 = The aligned, physical base address of the page table
	 *   R2 = The aligned, virtual base address of the page table
	 *   R3 = The MMU flags to use with the .text space mapping
	 *   R5 = The physical address of the L1 page table (from above)
	 *
	 * The value in R1 could have been obtained by simply masking R5.
	 */

	adr		r0, .LCptinfo			/* Address of page table description */
	ldmia	r0, {r1, r2, r3}		/* Load the page table description */

	/* A single page is sufficient to map the page table */

	orr		r0, r1, r3				/* OR MMU flags into physical address */
	str		r0, [r5, r2, lsr #18]	/* Map using the virtual address as an index */
#endif

	/* Load information needed to map the .text region.  After the ldmia, we
	 * will have:
	 *
	 *   R1 = Aligned, physical address of the start of the .text region
	 *   R2 = Aligned, virtual address of the start of the .text region
	 *   R3 = MMU flags associated with the .txt region
	 *   R4 = The number of 1MB sections in the mapping
	 *   R5 = The physical address of the L1 page table (from above)
	 */

	adr		r0, .LCtextinfo			/* Address of text info */
	ldmia	r0, {r1, r2, r3, r4}	/* Load the text description */

	/* Create identity mapping for first MB of the .text section to support
	 * this start-up logic executing out of the physical address space.  This
	 * identity mapping will be removed by .Lvstart (see below).  Of course,
	 * we would only do this if the physical-virtual mapping is not already
	 * the identity mapping.
	 */

#ifndef CONFIG_IDENTITY_TEXTMAP
	orr		r0, r1, r3				/* OR MMU flags into physical address */
	str		r0, [r5, r1, lsr #18]	/* Identity mapping */
#endif

	/* Map the entire .text region.  We do this before enabling caches so
	 * we know that the data will be in place in the data cache.  We map the
	 * entire text region because we don't know which parts are needed for
	 * start-up.
	 *
	 * The page table base address is in R5.  Each 32-bit page table entry
	 * maps 1 MB of address space and is indexed by the lower 20 bits of
	 * the virtual address in R2
	 */

	add		r2, r5, r2, lsr #18		/* R2=Offset page table address */

	/* No loop until each page table entry has been written for the .text
	 * region.
	 */

.Lpgtextloop:
	orr		r0, r1, r3				/* R0: OR MMU flags into physical address */
	subs	r4, r4, #1				/* R4: Decrement the section count */
	str		r0, [r2], #4			/* Save page table entry, increment page table address */
	add		r1, r1, #(1024*1024)	/* R1: Increment the physical address */
	bne		.Lpgtextloop			/* Loop while R4 is non-zero */

#ifdef CONFIG_BOOT_RUNFROMFLASH
	/* If we are executing from FLASH, then we will need additional mappings for
	 * the primary RAM region that holds the .data, .bss, stack, and heap memory.
	 *
	 *   Here we expect to have:
	 *   r5 = Address of the base of the L1 table
	 *
	 *   REVISIT: We might need this second mapping under certain conditions
	 *   when executing from RAM too.  When the RAM region is larger than 1MB
	 *   and the page table is in the high end of RAM, then the single mapping
	 *   above will not be sufficient.
	 *
	 *   REVISIT: If the page table is not located in the primary RAM regions,
	 *   then we will also need an additional map the page table if the page
	 *   table resides in internal SRAM.
	 *
	 * Load information needed to map the .text region.  After the ldmia, we
	 * will have:
	 *
	 *   R1 = Aligned, physical address of the start of the .text region
	 *   R2 = Aligned, virtual address of the start of the .text region
	 *   R3 = MMU flags associated with the .txt region
	 *   R4 = The number of 1MB sections in the mapping
	 *   R5 = The physical address of the L1 page table (from above)
	 */

	adr		r0, .LCraminfo			/* Address of primary RAM info */
	ldmia	r0, {r1, r2, r3, r4}	/* Load the primary RAM description */
	add		r2, r5, r2, lsr #18		/* R2=Offset page table addres */

	/* Loop until each page table entry has been written for the primary RAM
	 * region.
	 */

.Lpgramloop:
	orr		r0, r1, r3				/* R0: OR MMU flags into physical address */
	subs	r4, r4, #1				/* R4: Decrement the section count */
	str		r0, [r2], #4			/* Save page table entry, increment page table address */
	add		r1, r1, #(1024*1024)	/* R1: Increment the physical address */
	bne		.Lpgramloop				/* Loop while R4 is non-zero */

#endif /* CONFIG_BOOT_RUNFROMFLASH */
#endif /* CONFIG_ARCH_ROMPGTABLE */

	/* The following logic will set up the ARMv7-A for normal operation.
	 *
	 * Here we expect to have:
	 *   r5 = Address of the base of the L1 table
	 */

	/* Invalidate caches and TLBs.
	 *
	 *   NOTE: "The ARMv7 Virtual Memory System Architecture (VMSA) does not
	 *   support a CP15 operation to invalidate the entire data cache. ...
	 *   In normal usage the only time the entire data cache has to be
	 *   invalidated is on reset."
	 *
	 * The instruction cache is virtually indexed and physically tagged but
	 * the data cache is physically indexed and physically tagged.  So it
	 * should not be an issue if the system comes up with a dirty Dcache;
	 * the ICache, however, must be invalidated.
	 */

	mov		r0, #0
	mcr		CP15_TLBIALL(r0,c7)	/* Invalidate the entire unified TLB */
	mcr		CP15_TLBIALL(r0,c6)
	mcr		CP15_TLBIALL(r0,c5)
	mcr		CP15_BPIALL(r0)		/* Invalidate entire branch prediction array */
	mcr		CP15_ICIALLU(r0)	/* Invalidate I-cache */

	/* Load the page table address.
	 *
	 * NOTES:
	 * - Here we assume that the page table address is aligned to at least
	 *   least a 16KB boundary (bits 0-13 are zero).  No masking is provided
	 *   to protect against an unaligned page table address.
	 * - The ARMv7-A has two page table address registers, TTBR0 and 1.
	 *   Only TTBR0 is used in this implementation but both are initialized.
	 *
	 * Here we expect to have:
	 *   r0 = Zero
	 *   r5 = Address of the base of the L1 table
	 */

	orr		r1, r5, #(TTBR0_RGN_WBWA | TTBR0_IRGN0)	/* Select cache properties */
	mcr		CP15_TTBR0(r1)
	mcr		CP15_TTBR1(r1)

	/* Set the TTB control register (TTBCR) to indicate that we are using
	 * TTBR0.  r0 still holds the value of zero.
	 *
	 *   N   : 0=Selects TTBR0 and 16KB page table size indexed by VA[31:20]
	 *   PD0 : 0=Perform translation table walks using TTBR0
	 *   PD1 : 0=Perform translation table walks using TTBR1 (but it is disabled)
	 *   EAE : 0=Use 32-bit translation system
	 */

	mcr		CP15_TTBCR(r0)

	/* Enable the MMU and caches
	 * lr = Resume at .Lvstart with the MMU enabled
	 */

	ldr		lr, .LCvstart			/* Abs. virtual address */

	/* Configure the domain access register (see mmu.h).  Only domain 0 is
	 * supported and it uses the permissions in the TLB.
	 */

	mov		r0, #DACR_CLIENT(0)
	mcr		CP15_DACR(r0)			/* Set domain access register */

	/* Configure the system control register (see sctrl.h) */

	mrc		CP15_SCTLR(r0)			/* Get control register */

	/* Clear bits to reset values.  This is only necessary in situations like, for
	 * example, we get here via a bootloader and the control register is in some
	 * unknown state.
	 *
	 *   SCTLR_A    Bit 1:  Strict alignment disabled (reset value)
	 *   SCTLR_C    Bit 2:  DCache disabled (reset value)
	 *
	 *   SCTLR_SW   Bit 10: SWP/SWPB not enabled (reset value)
	 *   SCTLR_I    Bit 12: ICache disabled (reset value)
	 *   SCTLR_V    Bit 13: Assume low vectors (reset value)
	 *   SCTLR_RR   Bit 14: The Cortex-A5 processor only supports a fixed random
	 *                      replacement strategy.
	 *   SCTLR_HA   Bit 17: Not supported by A5
	 *
	 *   SCTLR_EE   Bit 25: Little endian (reset value).
	 *   SCTLR_TRE  Bit 28: No memory region remapping (reset value)
	 *   SCTLR_AFE  Bit 29: Full, legacy access permissions behavior (reset value).
	 *   SCTLR_TE   Bit 30: All exceptions handled in ARM state (reset value).
	 */

	bic		r0, r0, #(SCTLR_A  | SCTLR_C)
	bic		r0, r0, #(SCTLR_SW | SCTLR_I   | SCTLR_V   | SCTLR_RR | SCTLR_HA)
	bic		r0, r0, #(SCTLR_EE | SCTLR_TRE | SCTLR_AFE | SCTLR_TE)

	/* Set bits to enable the MMU
	 *
	 *   SCTLR_M     Bit 0:  Enable the MMU
	 *   SCTLR_Z     Bit 11: Program flow prediction control always enabled on A5
	 */

	orr		r0, r0, #(SCTLR_M)
#ifndef CONFIG_ARCH_CORTEXA5
	orr		r0, r0, #(SCTLR_Z)
#endif

	/* Position vectors to 0xffff0000 if so configured.
	 *
	 *   SCTLR_V    Bit 13: High vectors
	 */

#ifndef CONFIG_ARCH_LOWVECTORS
	orr		r0, r0, #(SCTLR_V)
#endif

	/* Round Robin cache replacement
	 *
	 *   SCTLR_RR   Bit 14: The Cortex-A5 processor only supports a fixed random
	 *                      replacement strategy.
	 */

#if defined(CPU_CACHE_ROUND_ROBIN) && !defined(CONFIG_ARCH_CORTEXA5)
	orr		r0, r0, #(SCTLR_RR)
#endif

	/* Dcache enable
	 *
	 *   SCTLR_C    Bit 2:  DCache enable
	 */

#ifndef CPU_DCACHE_DISABLE
	orr		r0, r0, #(SCTLR_C)
#endif

	/* Icache enable
	 *
	 *   SCTLR_I    Bit 12: ICache enable
	 */

#ifndef CPU_ICACHE_DISABLE
	orr		r0, r0, #(SCTLR_I)
#endif

	/* Alignment abort enable
	 *
	 *   SCTLR_A    Bit 1:  Strict alignment enabled
	 */

#ifdef ALIGNMENT_TRAP
	orr		r0, r0, #(SCTLR_A)
#endif

	/* AP[0:2] Permissions model
	 *
	 *   SCTLR_AFE  Bit 29: Full, legacy access permissions behavior (reset value).
	 *
	 * When AFE=1, the page table AP[0] is used as an access flag and AP[2:1]
	 * control.  When AFE=0, AP[2:0] control access permissions.
	 */

#ifdef CONFIG_AFE_ENABLE
	orr		r0, r0, #(SCTLR_AFE)
#endif

	/* Then write the configured control register */

	mcr		CP15_SCTLR(r0)			/* Write control reg */
	.rept	12						/* Cortex A8 wants lots of NOPs here */
	nop
	.endr

	/* And "jump" to .Lvstart in the newly mapped virtual address space */

	mov		pc, lr

/****************************************************************************
 * PC_Relative Data
 ****************************************************************************/

	/* The physical base address of the page table */

	.type	.LCppgtable, %object
.LCppgtable:
	.long	PGTABLE_BASE_PADDR		/* Physical start of page table */
	.size	.LCppgtable, . -.LCppgtable

#ifdef ARMV7A_PGTABLE_MAPPING
	/* Page table region description.  The order of these fields must not
	 * change because the values are loaded using ldmia:
	 *
	 * 1) The aligned, physical base address of the page table
	 * 2) The aligned, virtual base address of the page table
	 * 3) The MMU flags to use with the .text space mapping
	 */

	.type	.LCptinfo, %object
.LCptinfo:
	.long	(PGTABLE_BASE_PADDR & 0xfff00000)	/* Physical base address */
	.long	(PGTABLE_BASE_VADDR & 0xfff00000)	/* Virtual base address */
	.long	MMU_MEMFLAGS						/* MMU flags for text section in RAM */
	.size	.LCptinfo, . -.LCptinfo
#endif

#ifndef CONFIG_ARCH_ROMPGTABLE
	/* Text region description.  The order of these fields must not change
	 * because the values are loaded using ldmia:
	 *
	 * 1) The aligned, physical base address of the .text section
	 * 2) The aligned, virtual base address of the .text section
	 * 3) The MMU flags to use with the .text space mapping
	 * 4) The number of 1MB sections in the .text region
	 *
	 * Values provided for NUTTX_TEXT_* must all be properly aligned to 1MB
	 * address boundaries
	 */

	.type	.LCtextinfo, %object
.LCtextinfo:
.LCptextbase:
	.long	NUTTX_TEXT_PADDR			/* Physical base address */
	.long	NUTTX_TEXT_VADDR			/* Virtual base address */
.LCtextflags:
#ifdef CONFIG_BOOT_RUNFROMFLASH
	.long	MMU_ROMFLAGS				/* MMU flags text section in FLASH/ROM */
#else
	.long	MMU_MEMFLAGS				/* MMU flags for text section in RAM */
#endif
	.long	(NUTTX_TEXT_SIZE >> 20)		/* Number of 1MB read-execute sections */
	.size	.LCtextinfo, . -.LCtextinfo

#ifdef CONFIG_BOOT_RUNFROMFLASH
	/* Primary RAM region description.  The order of these fields must not change
	 * because the values are loaded using ldmia:
	 *
	 * 1) The aligned, physical base address of the primary RAM section
	 * 2) The aligned, virtual base address of the primary RAM section
	 * 3) The MMU flags to use with the primary RAM space mapping
	 * 4) The number of 1MB sections in the primary RAM region
	 *
	 * Values provided for NUTTX_RAM_* must all be properly aligned to 1MB
	 * address boundaries
	 */

	.type	.LCraminfo, %object
.LCraminfo:
	.long	NUTTX_RAM_PADDR				/* Physical base address */
	.long	NUTTX_RAM_VADDR				/* Virtual base address */
	.long	MMU_MEMFLAGS				/* MMU flags for primary RAM section */
	.long	(NUTTX_RAM_SIZE >> 20)		/* Number of 1MB read-execute sections */
	.size	.LCraminfo, . -.LCraminfo

#endif /* CONFIG_BOOT_RUNFROMFLASH */
#endif /* CONFIG_ARCH_ROMPGTABLE */

	/* The virtual start address of the second phase boot logic */

	.type	.LCvstart, %object
.LCvstart:
	.long	.Lvstart
	.size	.LCvstart, . -.LCvstart

	.size	__start, .-__start

/****************************************************************************
 * Name: .Lvstart
 ***************************************************************************/

/* The following is executed after the MMU has been enabled. This uses
 * absolute addresses; this is not position independent.
 */
	.align	5
	.local	.Lvstart
	.type	.Lvstart, %function

.Lvstart:

	/* Remove the temporary mapping (if one was made).  The following assumes
	 * that the total RAM size is > 1Mb and extends that initial mapping to
	 * cover additional RAM sections.
	 */

#if !defined(CONFIG_ARCH_ROMPGTABLE) && !defined(CONFIG_IDENTITY_TEXTMAP)
	ldr		r5, .LCvpgtable			/* r5=Virtual page table base address */
	ldr		r3, .LCptextbase		/* r0=Physical base address of .text section */
	mov		r0, #0					/* flags + base = 0 */
	str		r3, [r5, r3, lsr #18]	/* identity mapping */
#endif /* !CONFIG_ARCH_ROMPGTABLE && !CONFIG_IDENTITY_TEXTMAP */

	/* Zero BSS and set up the stack pointer */

	adr		r0, .Linitparms
	ldmia	r0, {r0, r1, sp}

	/* Clear the frame pointer and .bss */

	mov     fp, #0

.Lbssinit:	
	cmp		r0, r1				/* Clear up to _bss_end_ */
	strcc	fp, [r0],#4
	bcc		.Lbssinit

	/* If the .data section is in a separate, uninitialized address space,
	 * then we will also need to copy the initial values of of the .data
	 * section from the .text region into that .data region.  This would
	 * be the case if we are executing from FLASH and the .data section
	 * lies in a different physical address region OR if we are support
	 * on-demand paging and the .data section lies in a different virtual
	 * address region.
	 */

#if defined(CONFIG_BOOT_RUNFROMFLASH)
	adr		r3, .Ldatainit
	ldmia	r3, {r0, r1, r2}

1:	ldmia	r0!, {r3 - r10}
	stmia	r1!, {r3 - r10}
	cmp		r1, r2
	blt		1b
#endif

	/* Perform early C-level, platform-specific initialization */

	bl		up_boot

#ifdef CONFIG_DEBUG_STACK
	/* Write a known value to the IDLE thread stack to support stack
	 * monitoring logic
	 */

	adr		r3, .Lstkinit
	ldmia	r3, {r0, r1, r2}	/* R0 = start of IDLE stack; R1 = Size of tack; R2 = coloration */

2:								/* Top of the loop */
	sub		r1, r1, #1			/* R1 = Number of words remaining */
	cmp		r1, #0				/* Check (nwords == 0) */
	str		r2, [r0], #4		/* Save stack color word, increment stack address */
	bne		2b					/* Bottom of the loop */

#endif

	/* Finally branch to the OS entry point */

	mov		lr, #0				/* LR = return address (none) */
	b		os_start			/* Branch to os_start */

	/* Text-section constants:
	 *
	 *   _sbss is the start of the BSS region (see ld.script)
	 *   _ebss is the end of the BSS regsion (see ld.script)
	 *
	 * The idle task stack starts at the end of BSS and is of size
	 * CONFIG_IDLETHREAD_STACKSIZE.  The heap continues from there until the
	 * end of memory.  See g_idle_topstack below.
	 */

#ifndef CONFIG_ARCH_ROMPGTABLE
	/* The virtual base address of the page table */

	.type	.LCvpgtable, %object
.LCvpgtable:
	.long	PGTABLE_BASE_VADDR		/* Virtual start of page table */
	.size	.LCvpgtable, . -.LCvpgtable

#endif /* CONFIG_ARCH_ROMPGTABLE */

	.type	.Linitparms, %object
.Linitparms:
	.long	_sbss
	.long	_ebss
	.long	_ebss+CONFIG_IDLETHREAD_STACKSIZE-4
	.size	.Linitparms, . -.Linitparms

#ifdef CONFIG_BOOT_RUNFROMFLASH
	.type	.Ldatainit, %object
.Ldatainit:
	.long	_eronly					/* Where .data defaults are stored in FLASH */
	.long	_sdata					/* Where .data needs to reside in SDRAM */
	.long	_edata
	.size	.Ldatainit, . -.Ldatainit
#endif

#ifdef CONFIG_DEBUG_STACK
	.type	.Lstkinit, %object
.Lstkinit:
	.long	_ebss					/* Beginning of the IDLE stack, then words of IDLE stack */
	.long	(CONFIG_IDLETHREAD_STACKSIZE >> 2)
	.long	STACK_COLOR				/* Stack coloration word */
	.size	.Lstkinit, . -.Lstkinit
#endif
	.size	.Lvstart, .-.Lvstart

	 /* Data section variables */

	/* This global variable is unsigned long g_idle_topstack and is
	 * exported from here only because of its coupling to .Linitparms
	 * above.
	 */

	.section	.rodata, "a"
	.align	4
	.globl	g_idle_topstack
	.type	g_idle_topstack, object
g_idle_topstack:
	.long	_ebss+CONFIG_IDLETHREAD_STACKSIZE
	.size	g_idle_topstack, .-g_idle_topstack
	.end
