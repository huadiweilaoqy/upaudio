/* Audio Library for Teensy 3.X
 * Copyright (c) 2014, Paul Stoffregen, paul@pjrc.com
 *
 * Development of this audio library was funded by PJRC.COM, LLC by sales of
 * Teensy and Audio Adaptor boards.  Please support PJRC's efforts to develop
 * open source software by purchasing Teensy or other PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "output_dac.h"
#include "utility/pdb.h"
#include "utility/dma.h"
#include "wiring_private.h"


DMAMEM static uint16_t dac_buffer[AUDIO_BLOCK_SAMPLES*2];
audio_block_t * AudioOutputAnalog::block_left_1st = NULL;
audio_block_t * AudioOutputAnalog::block_left_2nd = NULL;

bool AudioOutputAnalog::update_responsibility = false;

Adafruit_ZeroDMA *AudioOutputAnalog::dma0;
DmacDescriptor *AudioOutputAnalog::desc;
static ZeroDMAstatus    stat;

static uint32_t *saddr;

void AudioOutputAnalog::begin(void)
{
	dma0 = new Adafruit_ZeroDMA();

	stat = dma0->allocate();
	
	// pinPeripheral(PIN_DAC0, PIO_ANALOG);
	pinPeripheral(PIN_DAC1, PIO_ANALOG);
	
	while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
	DAC->CTRLA.bit.ENABLE = 0;     // disable DAC
	
	while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
	// DAC->DACCTRL[0].bit.ENABLE = 1;
	DAC->DACCTRL[1].bit.ENABLE = 1;
	
	while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
	DAC->CTRLA.bit.ENABLE = 1;     // enable DAC

	
	// slowly ramp up to DC voltage, approx 1/4 second
	//升高DAC电压
	for (int16_t i=0; i<=2048; i+=8) {
		while (/* !DAC->STATUS.bit.READY0 ||*/  !DAC->STATUS.bit.READY1);
		while (/* DAC->SYNCBUSY.bit.DATA0 ||*/  DAC->SYNCBUSY.bit.DATA1 );
		// DAC->DATA[0].reg = i;
		DAC->DATA[1].reg = i;
		delay(1);
	}

	//TODO: on SAMD51 lets find an unused timer and use that
	//设置一个定时器来进行中断定时，并且定时触发DMA请求

	GCLK->PCHCTRL[AUDIO_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK2_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
	AUDIO_TC->COUNT8.WAVE.reg = TC_WAVE_WAVEGEN_NFRQ;
	
	AUDIO_TC->COUNT8.CTRLA.reg &= ~TC_CTRLA_ENABLE;
	WAIT_TC8_REGS_SYNC(AUDIO_TC)

	AUDIO_TC->COUNT8.CTRLA.reg = TC_CTRLA_SWRST;
	WAIT_TC8_REGS_SYNC(AUDIO_TC)
	while (AUDIO_TC->COUNT8.CTRLA.bit.SWRST);
	
	AUDIO_TC->COUNT8.CTRLA.reg |= TC_CTRLA_MODE_COUNT8 | AUDIO_PRESCALER;
	WAIT_TC8_REGS_SYNC(AUDIO_TC)

	AUDIO_TC->COUNT8.PER.reg = (uint8_t)( AUDIO_CLKRATE / AUDIO_TC_FREQ);
	WAIT_TC8_REGS_SYNC(AUDIO_TC)
	
	AUDIO_TC->COUNT8.CTRLA.reg |= TC_CTRLA_ENABLE;
	WAIT_TC8_REGS_SYNC(AUDIO_TC)
	/*
		TC5->COUNT8.CTRLA.reg |= TC_CTRLA_ENABLE;
		while(TC5->COUNT8.SYNCBUSY.bit.ENABLE || TC5->COUNT8.SYNCBUSY.bit.SWRST);
	*/
	
	dma0->setTrigger(AUDIO_TC_DMAC_ID_OVF);
	dma0->setAction(DMA_TRIGGER_ACTON_BEAT);
	
	desc = dma0->addDescriptor(
	  dac_buffer,			   		// move data from here
	  (void *)(&DAC->DATA[1]), 			// to here
	  AUDIO_BLOCK_SAMPLES,                // this many...
	  DMA_BEAT_SIZE_WORD,               // bytes/hword/words
	  true,                             // increment source addr?
	  false);
	
	desc->BTCTRL.bit.BLOCKACT = DMA_BLOCK_ACTION_INT;


	dma0->loop(true);

	update_responsibility = update_setup();
	dma0->setCallback(AudioOutputAnalog::isr);
	dma0->startJob();

}

void AudioOutputAnalog::analogReference(int ref)
{
	// TODO: this should ramp gradually to the new DC level
	// if (ref == INTERNAL) {
	// 	DAC0_C0 &= ~DAC_C0_DACRFS; // 1.2V
	// } else {
	// 	DAC0_C0 |= DAC_C0_DACRFS;  // 3.3V
	// }
}


void AudioOutputAnalog::update(void)
{
	audio_block_t *block;
	block = receiveReadOnly(0); // input 0
	if (block) {
		__disable_irq();
		if (block_left_1st == NULL) {
			block_left_1st = block;
			__enable_irq();
		} else if (block_left_2nd == NULL) {
			block_left_2nd = block;
			__enable_irq();
		} else {
			audio_block_t *tmp = block_left_1st;
			block_left_1st = block_left_2nd;
			block_left_2nd = block;
			__enable_irq();
			release(tmp);
		}
	}
}

// TODO: the DAC has much higher bandwidth than the datasheet says
// can we output a 2X oversampled output, for easier filtering?

void AudioOutputAnalog::isr(Adafruit_ZeroDMA *dma)
{
	const int16_t *src, *end;
	int16_t *dest;
	audio_block_t *block;
	uint32_t saddr;

	__disable_irq();
	saddr = (uint32_t)dac_buffer;
	// dma.clearInterrupt();
	if (saddr < (uint32_t)dac_buffer + sizeof(dac_buffer) / 2) {
		// DMA is transmitting the first half of the buffer
		// so we must fill the second half
		dest = (int16_t *)&dac_buffer[AUDIO_BLOCK_SAMPLES];
		end = (int16_t *)&dac_buffer[AUDIO_BLOCK_SAMPLES*2];
	} else {
		// DMA is transmitting the second half of the buffer
		// so we must fill the first half
		dest = (int16_t *)dac_buffer;
		end = (int16_t *)&dac_buffer[AUDIO_BLOCK_SAMPLES];
	}
	block = AudioOutputAnalog::block_left_1st;
	if (block) {
		src = block->data;
		do {
			// TODO: this should probably dither
			*dest++ = ((*src++) + 32768) >> 4;
		} while (dest < end);
		AudioStream::release(block);
		AudioOutputAnalog::block_left_1st = AudioOutputAnalog::block_left_2nd;
		AudioOutputAnalog::block_left_2nd = NULL;
	} else {
		do {
			*dest++ = 2048;
		} while (dest < end);
	}
	__enable_irq();
	if (AudioOutputAnalog::update_responsibility) AudioStream::update_all();
}