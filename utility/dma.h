/**
 * \file
 *
 * \brief SAM Direct Memory Access Controller Driver
 *
 * Copyright (C) 2014-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#ifndef DMA_H_INCLUDED
#define DMA_H_INCLUDED

// THIS IS A PARED-DOWN VERSION OF DMA.H FROM ATMEL ASFCORE 3.
// Please keep original copyright and license intact!

#ifdef __cplusplus
extern "C" {
#endif

#if (SAML21) || (SAML22) || (SAMC20) || (SAMC21) || defined(__DOXYGEN__) || defined(__SAMD51__)
#define FEATURE_DMA_CHANNEL_STANDBY
#endif

enum dma_transfer_trigger_action{
#ifdef __SAMD51__
    // SAMD51 has a 'burst' transfer which can be set to one
    // beat to accomplish same idea as SAMD21's 'beat' transfer.
    // Trigger name is ACTON_BEAT for backward compatibility.
    DMA_TRIGGER_ACTON_BLOCK       = DMAC_CHCTRLA_TRIGACT_BLOCK_Val,
    DMA_TRIGGER_ACTON_BEAT        = DMAC_CHCTRLA_TRIGACT_BURST_Val,
    DMA_TRIGGER_ACTON_TRANSACTION = DMAC_CHCTRLA_TRIGACT_TRANSACTION_Val,
#else
    DMA_TRIGGER_ACTON_BLOCK       = DMAC_CHCTRLB_TRIGACT_BLOCK_Val,
    DMA_TRIGGER_ACTON_BEAT        = DMAC_CHCTRLB_TRIGACT_BEAT_Val,
    DMA_TRIGGER_ACTON_TRANSACTION = DMAC_CHCTRLB_TRIGACT_TRANSACTION_Val,
#endif
};
//DMA回调类型
enum dma_callback_type {
    // First item here is for any transfer errors. A transfer error is
    // flagged if a bus error is detected during an AHB access or when
    // the DMAC fetches an invalid descriptor
    //这里的第一项是关于任何传输错误的，如果在AHB访问期间检测到总线错误或者当DMAC获取无效描述符时将标记传输错误
    DMA_CALLBACK_TRANSFER_ERROR,
    DMA_CALLBACK_TRANSFER_DONE,
    DMA_CALLBACK_CHANNEL_SUSPEND,
    DMA_CALLBACK_N, // Number of available callbacks    可用回调数
};

//DMAbit大小
enum dma_beat_size {
    DMA_BEAT_SIZE_BYTE = 0, // 8-bit
    DMA_BEAT_SIZE_HWORD,    // 16-bit
    DMA_BEAT_SIZE_WORD,     // 32-bit
};
//DMA事件输出选择
enum dma_event_output_selection {
    DMA_EVENT_OUTPUT_DISABLE = 0, // Disable event generation 禁用事件生成
    DMA_EVENT_OUTPUT_BLOCK,       // Event strobe when block xfer complete
    DMA_EVENT_OUTPUT_RESERVED,      
    DMA_EVENT_OUTPUT_BEAT,        // Event strobe when beat xfer complete
};                                  

enum dma_block_action {             
    DMA_BLOCK_ACTION_NOACT = 0,  
    // Channel in normal operation and sets transfer complete interrupt
    // flag after block transfer
    //通道正常运行并在块传输后完成中断标志
    DMA_BLOCK_ACTION_INT,
    // Trigger channel suspend after block transfer and sets channel
    // suspend interrupt flag once the channel is suspended
    //块传输后触发通道暂停，并在通道暂停后设置通道暂停中断标志。
    DMA_BLOCK_ACTION_SUSPEND,
    // Sets transfer complete interrupt flag after a block transfer and
    // trigger channel suspend. The channel suspend interrupt flag will
    // be set once the channel is suspended.
    //在块传输和触发通道挂起后，设置传输完成中断，通道暂停后，将设置通道暂停中断标志
    DMA_BLOCK_ACTION_BOTH,
};

// DMA step selection. This bit determines whether the step size setting
// is applied to source or destination address.
// DMA步骤选择。 该位确定步长设置是应用于源地址还是目标地址。
enum dma_step_selection {
    DMA_STEPSEL_DST = 0,
    DMA_STEPSEL_SRC,
};

// Address increment step size. These bits select the address increment step
// size. The setting apply to source or destination address, depending on
// STEPSEL setting.
//地址增量步长。 这些位选择地址增量步长。 该设置适用于源地址或目标地址，具体取决于STEPSEL设置。
enum dma_address_increment_stepsize {
    DMA_ADDRESS_INCREMENT_STEP_SIZE_1 = 0, // beat size * 1
    DMA_ADDRESS_INCREMENT_STEP_SIZE_2,     // beat size * 2
    DMA_ADDRESS_INCREMENT_STEP_SIZE_4,     // beat size * 4
    DMA_ADDRESS_INCREMENT_STEP_SIZE_8,     // etc...
    DMA_ADDRESS_INCREMENT_STEP_SIZE_16,
    DMA_ADDRESS_INCREMENT_STEP_SIZE_32,
    DMA_ADDRESS_INCREMENT_STEP_SIZE_64,
    DMA_ADDRESS_INCREMENT_STEP_SIZE_128,
};

//优先级
// higher numbers are higher priority
enum dma_priority {
    DMA_PRIORITY_0, // lowest (default)
    DMA_PRIORITY_1,
    DMA_PRIORITY_2,
    DMA_PRIORITY_3, // highest
};

#ifdef __cplusplus
}
#endif

#endif // DMA_H_INCLUDED
