/* Copyright (c) 2022 Intel Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

 #include "asm_memory_management.h"

	.section .text, "ax"
	.align 64
power_down_literals:
	.literal_position
ipc_flag:
	.word 0x80000000 // IPC_DIPCTDR_BUSY
sram_dis_loop_cnt:
	.word 4096

	.global ace_power_down
	.type ace_power_down, @function

 /**
 * @brief Perform power down.
 *
 * Depending on arguments, memories are switched off.
 *
 * @param A2 - argument for LPSRAM
 * @param A3 - pointer to array containing power gating mask.
 * Size of array is determined by MAX_MEMORY_SEGMENTS define.
 * @param A4 - send response to ipc
 */

#define IPC_HOST_BASE				0x00073000
#define b_enable_lpsram				a2
#define pu32_hpsram_mask			a3
#define b_ipc_response				a4
#define temp_reg0					a6
#define temp_reg1					a7
#define temp_reg2					a8
#define temp_reg3					a9
#define temp_reg4					a10
#define temp_reg5					a11
#define temp_reg6					a12
#define p_ipc_regs					a13
#define u32_ipc_response_mask		a14
#define pfl_reg						a15

ace_power_down:
	entry sp, 32
	/**
	 * effectively executes:
	 * xthal_dcache_region_lock(&literals, 128);
	 * xthal_dcache_region_lock(&powerdown, 256);
	 * xthal_dcache_region_lock(&pu32_hpsram_mask, 64);
	 */
	movi pfl_reg, power_down_literals
	dpfl pfl_reg, 0
	dpfl pfl_reg, 64

	movi pfl_reg, ace_power_down
	ipfl pfl_reg, 0
	ipfl pfl_reg, 64
	ipfl pfl_reg, 128
	ipfl pfl_reg, 192

	mov  pfl_reg, pu32_hpsram_mask
	dpfl pfl_reg, 0
	/* move some values to registries before switching off whole memory */
	/* load address of DIPCTDR register */
	movi p_ipc_regs, IPC_HOST_BASE
	movi u32_ipc_response_mask, 0x20000000
_PD_DISABLE_LPSRAM:
/**
 * effectively executes:
 * if (b_enable_lpsram) {
 *     ace_lpsram_power_down_entire();
 * }
 */
	beqz b_enable_lpsram, _PD_DISABLE_HPSRAM
	m_ace_lpsram_power_down_entire temp_reg0, temp_reg1, temp_reg2, temp_reg3

_PD_DISABLE_HPSRAM:
	/* if value in memory pointed by pu32_hpsram_mask = 0
	 (hpsram_pwrgating_mask) - do not disable hpsram. */
	beqz pu32_hpsram_mask, _PD_SEND_IPC
	/**
	 * effectively executes:
	 * for (size_t seg_index = (MAX_MEMORY_SEGMENTS - 1); seg_index >= 0;
	 * --seg_index) {
	 * ace_hpsram_power_change(seg_index, mask[seg_index]);
	 * }
	 * where mask is given in pu32_hpsram_mask register
	 */

	.set seg_index, MAX_MEMORY_SEGMENTS - 1
	.rept MAX_MEMORY_SEGMENTS
		l32i temp_reg0, pu32_hpsram_mask, 4 * seg_index
		m_ace_hpsram_power_change\
			/*segment_index=*/	seg_index,\
			/*mask=*/	temp_reg0,\
			temp_reg1,\
			temp_reg2,\
			temp_reg3,\
			temp_reg4,\
			temp_reg5
		.set seg_index, seg_index - 1
	.endr

_PD_SEND_IPC:
	/**
	 * Send IPC to host informing of PD completion - Clear BUSY
	 * bit by writing IPC_DIPCTDR_BUSY to IPC_DIPCTDR
	 * and writing IPC_DIPCTDA_DONE to IPC_DIPCTDA
	 */

	/**
	 * effecfively executes:
	 * if (b_ipc_response)
	 * {
	 *     temp_reg2 = *p_ipc_regs;
	 *     *(p_ipc_regs) = 0x80000000;
	 *     *(p_ipc_regs + 1) = 0x40000000;
	 *     temp_reg1 = temp_reg2 | u32_ipc_response_mask;
	 *     *(p_ipc_regs + 4) = temp_reg1;
	 * }
	 */
	beqz b_ipc_response, _PD_SLEEP
	movi temp_reg0, 1
	slli temp_reg1, temp_reg0, 31
	l32i temp_reg2, p_ipc_regs, 0
	/* clear busy bit by storing whole message in ADDRESS(HfIPCx) */
	s32i temp_reg1, p_ipc_regs, 0
	/* store msg received from host in DIPCTDA register */
	/* to enlighten done bit and trigger interrupt on host side */
	/* ace busy is cleared by writing 0 */
	movi temp_reg1, 0
	s32i temp_reg1, p_ipc_regs, 0x4
	/* Copy to a13 with IPC_RESPONSE_MASK set which is in a14 */
	or temp_reg1, temp_reg2, u32_ipc_response_mask
	/* Send reply IPC writing to DIPCIDR register */
	movi temp_reg0, 0
	s32i temp_reg0, p_ipc_regs, 0x18
	s32i temp_reg1, p_ipc_regs, 0x10
	l32i temp_reg1, p_ipc_regs, 0x10

_PD_SLEEP:
/* effecfively executes:
 * xmp_spin()
 * waiti 5
 */
	movi temp_reg0, 128
loop:
	addi temp_reg0, temp_reg0, -1
	bnez temp_reg0, loop

	extw
	extw
	waiti 5
	1:
	j 1b

.size ace_power_down , . - ace_power_down
