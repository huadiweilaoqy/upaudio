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

#include "input_adcs.h"
#include "utility/pdb.h"
#include "utility/dspinst.h"

#include "wiring_private.h"
#include "utility/dma.h"

#define COEF_HPF_DCBLOCK    (1048300<<10)  // DC Removal filter coefficient in S1.30

static uint16_t left_buffer[AUDIO_BLOCK_SAMPLES];
static uint16_t right_buffer[AUDIO_BLOCK_SAMPLES];
audio_block_t * AudioInputAnalogStereo::block_left = NULL;
audio_block_t * AudioInputAnalogStereo::block_right = NULL;
uint16_t AudioInputAnalogStereo::offset_left = 0;
uint16_t AudioInputAnalogStereo::offset_right = 0;
int32_t AudioInputAnalogStereo::hpf_y1[2] = { 0, 0 };
int32_t AudioInputAnalogStereo::hpf_x1[2] = { 0, 0 };
bool AudioInputAnalogStereo::update_responsibility = false;
Adafruit_ZeroDMA *AudioInputAnalogStereo::dma0;
Adafruit_ZeroDMA *AudioInputAnalogStereo::dma1;
DmacDescriptor *AudioInputAnalogStereo::desc;
static ZeroDMAstatus    stat;

static uint32_t *daddrL, *daddrR;

void AudioInputAnalogStereo::init(uint8_t pin0, uint8_t pin1)
{
	
	dma0 = new Adafruit_ZeroDMA;
	dma1 = new Adafruit_ZeroDMA;

	stat = dma0->allocate();
	stat = dma1->allocate();
	
	pinPeripheral(pin0, PIO_ANALOG);
	pinPeripheral(pin1, PIO_ANALOG);

	GCLK->PCHCTRL[ADC0_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos); //use clock generator 1 (48Mhz)

	ADC0->CTRLA.bit.PRESCALER = ADC_CTRLA_PRESCALER_DIV32_Val;
	ADC0->CTRLB.reg = ADC_CTRLB_RESSEL_16BIT | ADC_CTRLB_FREERUN;
	while( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_CTRLB );  //wait for sync
	
	ADC0->SAMPCTRL.reg = 5;                        // sampling Time Length
	while( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_SAMPCTRL );  //wait for sync
	
	ADC0->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND;   // No Negative input (Internal Ground)
	while( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_INPUTCTRL );  //wait for sync

	ADC0->INPUTCTRL.bit.MUXPOS = g_APinDescription[pin0].ulADCChannelNumber; // Selection for the positive ADC input
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

	//******* Initialize ADC1 *********//
	GCLK->PCHCTRL[ADC1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos); //use clock generator 1 (48Mhz)

	ADC1->CTRLA.bit.PRESCALER = ADC_CTRLA_PRESCALER_DIV32_Val;
	ADC1->CTRLB.reg = ADC_CTRLB_RESSEL_16BIT | ADC_CTRLB_FREERUN;;
	while( ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_CTRLB );  //wait for sync

	ADC1->SAMPCTRL.reg = 5;                        // sampling Time Length
	while( ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_SAMPCTRL );  //wait for sync
	
	ADC1->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND;   // No Negative input (Internal Ground)
	while( ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_INPUTCTRL );  //wait for sync

	ADC1->INPUTCTRL.bit.MUXPOS = g_APinDescription[pin1].ulADCChannelNumber;
	while( ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_INPUTCTRL ); //wait for sync

	//TODO: this is weirdly set for a 13 bit result for now... we may want to change later
	ADC1->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 | ADC_AVGCTRL_ADJRES(0x0);
	while( ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_AVGCTRL );  //wait for sync

	ADC1->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val;
	while( ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync

	ADC1->CTRLA.bit.ENABLE = 0x01;             // Enable ADC
	while( ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync

	// Start conversion
	ADC1->SWTRIG.bit.START = 1;

	//TODO: on SAMD51 lets find an unused timer and use that
	GCLK->PCHCTRL[AUDIO_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
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
	
	dma0->setTrigger(AUDIO_TC_DMAC_ID_OVF);
	dma1->setTrigger(AUDIO_TC_DMAC_ID_OVF);
	dma0->setAction(DMA_TRIGGER_ACTON_BEAT);
	dma1->setAction(DMA_TRIGGER_ACTON_BEAT);
	
	desc = dma0->addDescriptor(
	(void *)(&ADC0->RESULT.reg),		// move data from here
	left_buffer,			// to here
	AUDIO_BLOCK_SAMPLES / 2,               // this many...
	DMA_BEAT_SIZE_HWORD,               // bytes/hword/words
	false,                             // increment source addr?
	true);
	desc->BTCTRL.bit.BLOCKACT = DMA_BLOCK_ACTION_INT;

	desc = dma0->addDescriptor(
	(void *)(&ADC0->RESULT.reg),		// move data from here
	left_buffer + AUDIO_BLOCK_SAMPLES / 2,			// to here
	AUDIO_BLOCK_SAMPLES / 2,               // this many...
	DMA_BEAT_SIZE_HWORD,               // bytes/hword/words
	false,                             // increment source addr?
	true);
	desc->BTCTRL.bit.BLOCKACT = DMA_BLOCK_ACTION_INT;
	dma0->loop(true);
	
	desc = dma1->addDescriptor(
	(void *)(&ADC1->RESULT.reg),		// move data from here
	right_buffer,			// to here
	AUDIO_BLOCK_SAMPLES / 2,               // this many...
	DMA_BEAT_SIZE_HWORD,               // bytes/hword/words
	false,                             // increment source addr?
	true);
	desc->BTCTRL.bit.BLOCKACT = DMA_BLOCK_ACTION_INT;

	desc = dma1->addDescriptor(
	(void *)(&ADC1->RESULT.reg),		// move data from here
	right_buffer + AUDIO_BLOCK_SAMPLES / 2,			// to here
	AUDIO_BLOCK_SAMPLES / 2,               // this many...
	DMA_BEAT_SIZE_HWORD,               // bytes/hword/words
	false,                             // increment source addr?
	true);
	desc->BTCTRL.bit.BLOCKACT = DMA_BLOCK_ACTION_INT;
	dma1->loop(true);

	dma1->setCallback(AudioInputAnalogStereo::isr1);
	
	update_responsibility = update_setup();
	dma0->setCallback(AudioInputAnalogStereo::isr0);
	dma0->startJob();
	dma1->startJob();
}


void AudioInputAnalogStereo::isr0(Adafruit_ZeroDMA *dma)
{
   uint32_t offset;
	const uint16_t *src, *end;
	uint16_t *dest;

	//digitalWriteFast(32, HIGH);
	if (daddrL != (uint32_t *)(left_buffer + AUDIO_BLOCK_SAMPLES / 2)) {
		// DMA is receiving to the first half of the buffer
		// need to remove data from the second half
		src = (uint16_t *)&left_buffer[AUDIO_BLOCK_SAMPLES/2];
		end = (uint16_t *)&left_buffer[AUDIO_BLOCK_SAMPLES];
		daddrL = (uint32_t *)(left_buffer + AUDIO_BLOCK_SAMPLES / 2);
	} else {
		// DMA is receiving to the second half of the buffer
		// need to remove data from the first half
		src = (uint16_t *)&left_buffer[0];
		end = (uint16_t *)&left_buffer[AUDIO_BLOCK_SAMPLES/2];
		//if (update_responsibility) AudioStream::update_all();
		daddrL = (uint32_t *)left_buffer;
	}

	if (block_left != NULL) {
		offset = offset_left;
		if (offset > AUDIO_BLOCK_SAMPLES/2) offset = AUDIO_BLOCK_SAMPLES/2;
		offset_left = offset + AUDIO_BLOCK_SAMPLES/2;
		dest = (uint16_t *)&(block_left->data[offset]);
		do {
			*dest++ = *src++;
		} while (src < end);
	}
	//digitalWriteFast(32, LOW);
}

void AudioInputAnalogStereo::isr1(Adafruit_ZeroDMA *dma)
{
	uint32_t offset;
	const uint16_t *src, *end;
	uint16_t *dest;

	//digitalWriteFast(33, HIGH);
	if (daddrR != (uint32_t *)(right_buffer + AUDIO_BLOCK_SAMPLES / 2)) {
		// DMA is receiving to the first half of the buffer
		// need to remove data from the second half
		src = (uint16_t *)&right_buffer[AUDIO_BLOCK_SAMPLES/2];
		end = (uint16_t *)&right_buffer[AUDIO_BLOCK_SAMPLES];
		daddrR = (uint32_t *)(right_buffer + AUDIO_BLOCK_SAMPLES / 2);
		if (update_responsibility) AudioStream::update_all();
	} else {
		// DMA is receiving to the second half of the buffer
		// need to remove data from the first half
		src = (uint16_t *)&right_buffer[0];
		end = (uint16_t *)&right_buffer[AUDIO_BLOCK_SAMPLES/2];
		daddrR = (uint32_t *)right_buffer;
	}
	
	if (block_right != NULL) {
		offset = offset_right;
		if (offset > AUDIO_BLOCK_SAMPLES/2) offset = AUDIO_BLOCK_SAMPLES/2;
		offset_right = offset + AUDIO_BLOCK_SAMPLES/2;
		dest = (uint16_t *)&(block_right->data[offset]);
		do {
			*dest++ = *src++;
		} while (src < end);
	}
	//digitalWriteFast(33, LOW);
}


void AudioInputAnalogStereo::update(void)
{
	audio_block_t *new_left=NULL, *out_left=NULL;
	audio_block_t *new_right=NULL, *out_right=NULL;
	int32_t tmp;
	int16_t s, *p, *end;

	//Serial.println("update");

	// allocate new block (ok if both NULL)
	new_left = allocate();
	if (new_left == NULL) {
		new_right = NULL;
		} else {
		new_right = allocate();
		if (new_right == NULL) {
			release(new_left);
			new_left = NULL;
		}
	}
	__disable_irq();
	if (offset_left < AUDIO_BLOCK_SAMPLES || offset_right < AUDIO_BLOCK_SAMPLES) {
		// the DMA hasn't filled up both blocks
		if (block_left == NULL) {
			block_left = new_left;
			offset_left = 0;
			new_left = NULL;
		}
		if (block_right == NULL) {
			block_right = new_right;
			offset_right = 0;
			new_right = NULL;
		}
		__enable_irq();
		if (new_left) release(new_left);
		if (new_right) release(new_right);
		return;
	}
	// the DMA filled blocks, so grab them and get the
	// new blocks to the DMA, as quickly as possible
	out_left = block_left;
	out_right = block_right;
	block_left = new_left;
	block_right = new_right;
	offset_left = 0;
	offset_right = 0;
	__enable_irq();

	// gain up to 16 bits
	p = out_left->data;
	end = p + AUDIO_BLOCK_SAMPLES;
	do {
		tmp = (int16_t)(*p);
		*p++ = (tmp-((1<<12)/2))*16;
	} while (p < end);

	// DC removal, RIGHT
	p = out_right->data;
	end = p + AUDIO_BLOCK_SAMPLES;
	do {
		tmp = (int16_t)(*p);
		*p++ = (tmp-((1<<12)/2))*16;
	} while (p < end);


	// then transmit the AC data
	transmit(out_left, 0);
	release(out_left);
	transmit(out_right, 1);
	release(out_right);
}


