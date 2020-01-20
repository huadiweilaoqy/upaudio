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

#include <Arduino.h>
#include "input_adc.h"
#include "utility/pdb.h"
#include "utility/dspinst.h"
#include "wiring_private.h"

#define COEF_HPF_DCBLOCK (1048300 << 10) // DC Removal filter coefficient in S1.30

DMAMEM static uint16_t analog_rx_buffer[AUDIO_BLOCK_SAMPLES];
audio_block_t *AudioInputAnalog::block_left = NULL;
uint16_t AudioInputAnalog::block_offset = 0;
int32_t AudioInputAnalog::hpf_y1 = 0;
int32_t AudioInputAnalog::hpf_x1 = 0;

bool AudioInputAnalog::update_responsibility = false;
Adafruit_ZeroDMA *AudioInputAnalog::dma;
DmacDescriptor *AudioInputAnalog::desc;
static ZeroDMAstatus stat;

void AudioInputAnalog::init(uint8_t pin)
{
	dma = new Adafruit_ZeroDMA;

	stat = dma->allocate();

	pinPeripheral(pin, PIO_ANALOG);
	GCLK->PCHCTRL[ADC0_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos); //use clock generator 1 (48Mhz)

	ADC0->CTRLA.bit.PRESCALER = ADC_CTRLA_PRESCALER_DIV32_Val;
	ADC0->CTRLB.reg = ADC_CTRLB_RESSEL_16BIT | ADC_CTRLB_FREERUN;
	while( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_CTRLB );  //wait for sync
	
	ADC0->SAMPCTRL.reg = 5;                        // sampling Time Length
	while( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_SAMPCTRL );  //wait for sync
	
	ADC0->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND;   // No Negative input (Internal Ground)
	while( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_INPUTCTRL );  //wait for sync

	ADC0->INPUTCTRL.bit.MUXPOS = g_APinDescription[pin].ulADCChannelNumber; // Selection for the positive ADC input
	while( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_INPUTCTRL ); //wait for sync

	// Averaging (see datasheet table in AVGCTRL register description)
	//TODO: this is weirdly set for a 13 bit result for now... we may want to change later
	ADC0->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 | ADC_AVGCTRL_ADJRES(0x0);
	while( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_AVGCTRL );  //wait for sync

	ADC0->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val;
	while( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync

	ADC0->CTRLA.bit.ENABLE = 0x01;             // Enable ADC
	while( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync

	// Start conversion
	ADC0->SWTRIG.bit.START = 1;	

	dma->setTrigger(AUDIO_TC_DMAC_ID_OVF);
	dma->setAction(DMA_TRIGGER_ACTON_BEAT);
	desc = dma->addDescriptor(
	(void *)(&ADC0->RESULT.reg),		// move data from here
	analog_rx_buffer,			// to here
	AUDIO_BLOCK_SAMPLES / 2,               // this many...
	DMA_BEAT_SIZE_HWORD,               // bytes/hword/words
	false,                             // increment source addr?
	true);
	desc->BTCTRL.bit.BLOCKACT = DMA_BLOCK_ACTION_INT;

	desc = dma->addDescriptor(
	(void *)(&ADC0->RESULT.reg),		// move data from here
	analog_rx_buffer + AUDIO_BLOCK_SAMPLES / 2,			// to here
	AUDIO_BLOCK_SAMPLES / 2,               // this many...
	DMA_BEAT_SIZE_HWORD,               // bytes/hword/words
	false,                             // increment source addr?
	true);
	desc->BTCTRL.bit.BLOCKACT = DMA_BLOCK_ACTION_INT;
	dma->loop(true);
	update_responsibility = update_setup();
	dma->setCallback(AudioInputAnalog::isr);
	dma->startJob();

}

void AudioInputAnalog::isr(Adafruit_ZeroDMA *dma)
{
	uint32_t daddr, offset;
	const uint16_t *src, *end;
	uint16_t *dest_left;
	audio_block_t *left;


	if (daddr < (uint32_t)analog_rx_buffer + sizeof(analog_rx_buffer) / 2)
	{
		// DMA is receiving to the first half of the buffer
		// need to remove data from the second half
		src = (uint16_t *)&analog_rx_buffer[AUDIO_BLOCK_SAMPLES / 2];
		end = (uint16_t *)&analog_rx_buffer[AUDIO_BLOCK_SAMPLES];
		if (update_responsibility)
			AudioStream::update_all();
	}
	else
	{
		// DMA is receiving to the second half of the buffer
		// need to remove data from the first half
		src = (uint16_t *)&analog_rx_buffer[0];
		end = (uint16_t *)&analog_rx_buffer[AUDIO_BLOCK_SAMPLES / 2];
	}
	left = block_left;
	if (left != NULL)
	{
		offset = block_offset;
		if (offset > AUDIO_BLOCK_SAMPLES / 2)
			offset = AUDIO_BLOCK_SAMPLES / 2;
		dest_left = (uint16_t *)&(left->data[offset]);
		block_offset = offset + AUDIO_BLOCK_SAMPLES / 2;
		do
		{
			*dest_left++ = *src++;
		} while (src < end);
	}
}

void AudioInputAnalog::update(void)
{
	audio_block_t *new_left = NULL, *out_left = NULL;
	uint32_t offset;
	int32_t tmp;
	int16_t s, *p, *end;

	//Serial.println("update");

	// allocate new block (ok if NULL)
	new_left = allocate();

	__disable_irq();
	offset = block_offset;
	if (offset < AUDIO_BLOCK_SAMPLES)
	{
		// the DMA didn't fill a block
		if (new_left != NULL)
		{
			// but we allocated a block
			if (block_left == NULL)
			{
				// the DMA doesn't have any blocks to fill, so
				// give it the one we just allocated
				block_left = new_left;
				block_offset = 0;
				__enable_irq();
				//Serial.println("fail1");
			}
			else
			{
				// the DMA already has blocks, doesn't need this
				__enable_irq();
				release(new_left);
				//Serial.print("fail2, offset=");
				//Serial.println(offset);
			}
		}
		else
		{
			// The DMA didn't fill a block, and we could not allocate
			// memory... the system is likely starving for memory!
			// Sadly, there's nothing we can do.
			__enable_irq();
			//Serial.println("fail3");
		}
		return;
	}
	// the DMA filled a block, so grab it and get the
	// new block to the DMA, as quickly as possible
	out_left = block_left;
	block_left = new_left;
	block_offset = 0;
	__enable_irq();

	//
	// DC Offset Removal Filter
	// 1-pole digital high-pass filter implementation
	//   y = a*(x[n] - x[n-1] + y[n-1])
	// The coefficient "a" is as follows:
	//  a = UNITY*e^(-2*pi*fc/fs)
	//  fc = 2 @ fs = 44100
	//
	p = out_left->data;
	end = p + AUDIO_BLOCK_SAMPLES;
	do
	{
		tmp = (uint16_t)(*p);
		tmp = (((int32_t)tmp) << 14);
		int32_t acc = hpf_y1 - hpf_x1;
		acc += tmp;
		hpf_y1 = FRACMUL_SHL(acc, COEF_HPF_DCBLOCK, 1);
		hpf_x1 = tmp;
		s = signed_saturate_rshift(hpf_y1, 16, 14);
		*p++ = s;
	} while (p < end);

	// then transmit the AC data
	transmit(out_left);
	release(out_left);
}
