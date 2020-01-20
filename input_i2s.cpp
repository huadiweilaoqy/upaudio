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

#include "input_i2s.h"
#include "output_i2s.h"

DMAMEM static uint32_t i2s_rx_buffer[AUDIO_BLOCK_SAMPLES];
audio_block_t * AudioInputI2S::block_left = NULL;
audio_block_t * AudioInputI2S::block_right = NULL;
uint16_t AudioInputI2S::block_offset = 0;
bool AudioInputI2S::update_responsibility = false;

Adafruit_ZeroI2S *AudioInputI2S::i2s;
Adafruit_ZeroDMA *AudioInputI2S::dma;
DmacDescriptor *AudioInputI2S::desc;
static ZeroDMAstatus    stat;

void AudioInputI2S::begin(void)
{

	dma = new Adafruit_ZeroDMA;

	stat = dma->allocate();
	i2s = new Adafruit_ZeroI2S(PIN_I2S_FS, PIN_I2S_SCK, PIN_I2S_SDO, PIN_I2S_SDI);
	AudioInputI2S::config_i2s();

	dma->setTrigger(I2S_DMAC_ID_RX_0);
	dma->setAction(DMA_TRIGGER_ACTON_BEAT);

	desc = dma->addDescriptor(
	 (void *)(&I2S->RXDATA.reg),						// move data from here
	 i2s_rx_buffer,			// to here
	  AUDIO_BLOCK_SAMPLES,               // this many...
	  DMA_BEAT_SIZE_HWORD,               // bytes/hword/words
	  false,                             // increment source addr?
	  true);

	update_responsibility = update_setup();
	dma->setCallback(AudioInputI2S::isr);

	dma->startJob();
}


void AudioInputI2S::isr(Adafruit_ZeroDMA *dma)
{
	uint32_t daddr, offset;
	const int16_t *src, *end;
	int16_t *dest_left, *dest_right;
	audio_block_t *left, *right;

	daddr = desc->DSTADDR.reg;
	daddr -= (AUDIO_BLOCK_SAMPLES) * 2;

	if (daddr == (uint32_t)i2s_rx_buffer) {
		// DMA is transmitting the second half of the buffer
		// so we must fill the first half
		src = (int16_t *)i2s_rx_buffer;
		end = (int16_t *)&i2s_rx_buffer[AUDIO_BLOCK_SAMPLES/2];
		dma->changeDescriptor(desc, NULL, (int16_t *)&i2s_rx_buffer[AUDIO_BLOCK_SAMPLES/2]);
	} else {
		// DMA is transmitting the first half of the buffer
		// so we must fill the second half
		src = (int16_t *)&i2s_rx_buffer[AUDIO_BLOCK_SAMPLES/2];
		end = (int16_t *)&i2s_rx_buffer[AUDIO_BLOCK_SAMPLES];
		dma->changeDescriptor(desc, NULL, (int16_t *)i2s_rx_buffer);
		if (AudioInputI2S::update_responsibility) AudioStream::update_all();
	}
	dma->startJob();
	left = AudioInputI2S::block_left;
	right = AudioInputI2S::block_right;
	if (left != NULL && right != NULL) {
		offset = AudioInputI2S::block_offset;
		if (offset <= AUDIO_BLOCK_SAMPLES/2) {
			dest_left = &(left->data[offset]);
			dest_right = &(right->data[offset]);
			AudioInputI2S::block_offset = offset + AUDIO_BLOCK_SAMPLES/2;
			do {
				//n = *src++;
				//*dest_left++ = (int16_t)n;
				//*dest_right++ = (int16_t)(n >> 16);
				*dest_left++ = *src++;
				*dest_right++ = *src++;
			} while (src < end);
		}
	}
	//digitalWriteFast(3, LOW);
}



void AudioInputI2S::update(void)
{
	audio_block_t *new_left=NULL, *new_right=NULL, *out_left=NULL, *out_right=NULL;

	// allocate 2 new blocks, but if one fails, allocate neither
	new_left = allocate();
	if (new_left != NULL) {
		new_right = allocate();
		if (new_right == NULL) {
			release(new_left);
			new_left = NULL;
		}
	}
	__disable_irq();
	if (block_offset >= AUDIO_BLOCK_SAMPLES) {
		// the DMA filled 2 blocks, so grab them and get the
		// 2 new blocks to the DMA, as quickly as possible
		out_left = block_left;
		block_left = new_left;
		out_right = block_right;
		block_right = new_right;
		block_offset = 0;
		__enable_irq();
		// then transmit the DMA's former blocks
		transmit(out_left, 0);
		release(out_left);
		transmit(out_right, 1);
		release(out_right);
		//Serial.print(".");
	} else if (new_left != NULL) {
		// the DMA didn't fill blocks, but we allocated blocks
		if (block_left == NULL) {
			// the DMA doesn't have any blocks to fill, so
			// give it the ones we just allocated
			block_left = new_left;
			block_right = new_right;
			block_offset = 0;
			__enable_irq();
		} else {
			// the DMA already has blocks, doesn't need these
			__enable_irq();
			release(new_left);
			release(new_right);
		}
	} else {
		// The DMA didn't fill blocks, and we could not allocate
		// memory... the system is likely starving for memory!
		// Sadly, there's nothing we can do.
		__enable_irq();
	}
}

void AudioInputI2S::config_i2s(void)
{

//check that i2s has not already been configured
	if(!I2S->CTRLA.bit.ENABLE){
		i2s->begin(I2S_16_BIT, 44100);
		i2s->enableMCLK();
		i2s->enableTx();
		i2s->enableRx();
	}
}

/******************************************************************/
