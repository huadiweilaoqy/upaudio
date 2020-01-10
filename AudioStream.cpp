/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2017 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include <Arduino.h>
#include "AudioStream.h"

//最大音频记忆
// #if defined(__MKL26Z64__)
//   #define MAX_AUDIO_MEMORY 6144
// #elif defined(__MK20DX128__)
//   #define MAX_AUDIO_MEMORY 12288
// #elif defined(__MK20DX256__)
//   #define MAX_AUDIO_MEMORY 49152
// #elif defined(__MK64FX512__)
//   #define MAX_AUDIO_MEMORY 163840
// #elif defined(__MK66FX1M0__)
//   #define MAX_AUDIO_MEMORY 229376
// #endif

//最大音频内存
#define MAX_AUDIO_MEMORY 49152
// 
#define NUM_MASKS  (((MAX_AUDIO_MEMORY / AUDIO_BLOCK_SAMPLES / 2) + 31) / 32)

audio_block_t * AudioStream::memory_pool;
uint32_t AudioStream::memory_pool_available_mask[NUM_MASKS];
uint16_t AudioStream::memory_pool_first_mask;

uint16_t AudioStream::cpu_cycles_total = 0;
uint16_t AudioStream::cpu_cycles_total_max = 0;
uint16_t AudioStream::memory_used = 0;
uint16_t AudioStream::memory_used_max = 0;

// Set up the pool of audio data blocks
// placing them all onto the free list
//设置音频数据块池，将它们全部放到空闲列表中

void AudioStream::initialize_memory(audio_block_t *data, unsigned int num)
{
	unsigned int i;
	//最大数量
	unsigned int maxnum = MAX_AUDIO_MEMORY / AUDIO_BLOCK_SAMPLES / 2;

	//Serial.println("AudioStream initialize_memory");
	//delay(10);

	if (num > maxnum) num = maxnum;
	__disable_irq();
	//音频流内存池等于初始memory
	memory_pool = data;
	memory_pool_first_mask = 0;
	for (i=0; i < NUM_MASKS; i++) {
		memory_pool_available_mask[i] = 0;
	}
	for (i=0; i < num; i++) {
		memory_pool_available_mask[i >> 5] |= (1 << (i & 0x1F));
	}
	for (i=0; i < num; i++) {
		data[i].memory_pool_index = i;
	}
	__enable_irq();

}

// Allocate 1 audio data block.  If successful the caller is the only owner of this new block
//分配一个音频数据块。如果成功，调用者是这个新块的惟一所有者
audio_block_t * AudioStream::allocate(void)
{
	uint32_t n, index, avail;
	uint32_t *p, *end;
	audio_block_t *block;
	uint32_t used;

	p = memory_pool_available_mask;
	end = p + NUM_MASKS;
	__disable_irq();
	index = memory_pool_first_mask;
	p += index;
	while (1) {
		//音频块已满
		if (p >= end) {
			__enable_irq();
			//Serial.println("alloc:null");
			return NULL;
		}
		avail = *p;
		if (avail) break;
		index++;
		p++;
	}
	n = __builtin_clz(avail);//返回avail前导0的个数
	avail &= ~(0x80000000 >> n);
	*p = avail;
	if (!avail) index++;
	memory_pool_first_mask = index;//重新更改起始mask
	used = memory_used + 1;		  //内存使用+1
	memory_used = used;			  //
	__enable_irq();
	index = p - memory_pool_available_mask;//返回当前是第几个音频块
	block = memory_pool + ((index << 5) + (31 - n));
	block->ref_count = 1;
	//音频块使用的最大值
	if (used > memory_used_max) memory_used_max = used;
	//Serial.print("alloc:");
	//Serial.println((uint32_t)block, HEX);
	return block;
}

// Release ownership of a data block.  If no
// other streams have ownership, the block is
// returned to the free pool
//释放数据块的所有权。如果没有其他流拥有所有权，则将块返回给空闲池
void AudioStream::release(audio_block_t *block)
{
	//if (block == NULL) return;
	uint32_t mask = (0x80000000 >> (31 - (block->memory_pool_index & 0x1F)));
	uint32_t index = block->memory_pool_index >> 5;

	__disable_irq();
	if (block->ref_count > 1) {
		block->ref_count--;
	} else {
		//Serial.print("reles:");
		//Serial.println((uint32_t)block, HEX);
		memory_pool_available_mask[index] |= mask;
		if (index < memory_pool_first_mask) memory_pool_first_mask = index;
		memory_used--;
	}
	__enable_irq();
}

// Transmit an audio data block
// to all streams that connect to an output.  The block
// becomes owned by all the recepients, but also is still
// owned by this object.  Normally, a block must be released
// by the caller after it's transmitted.  This allows the
// caller to transmit to same block to more than 1 output,
// and then release it once after all transmit calls.
// 将音频块发送到目标
void AudioStream::transmit(audio_block_t *block, unsigned char index)
{
	for (AudioConnection *c = destination_list; c != NULL; c = c->next_dest) {
		if (c->src_index == index) {
			if (c->dst.inputQueue[c->dest_index] == NULL) {
				c->dst.inputQueue[c->dest_index] = block;
				block->ref_count++;
			}
		}
	}
}


// Receive block from an input.  The block's data
// may be shared with other streams, so it must not be written
//接收来自输入的块，块共享，所以不能写入
audio_block_t * AudioStream::receiveReadOnly(unsigned int index)
{
	audio_block_t *in;

	if (index >= num_inputs) return NULL;
	in = inputQueue[index];
	inputQueue[index] = NULL;
	return in;
}

// Receive block from an input.  The block will not
// be shared, so its contents may be changed.
//接收来自输入的块，块不会被共享，所以它的内容可能会改变
audio_block_t * AudioStream::receiveWritable(unsigned int index)
{
	audio_block_t *in, *p;

	if (index >= num_inputs) return NULL;
	in = inputQueue[index];
	inputQueue[index] = NULL;
	if (in && in->ref_count > 1) {
		p = allocate();
		if (p) memcpy(p->data, in->data, sizeof(p->data));
		in->ref_count--;
		in = p;
	}
	return in;
}

//建立连接
void AudioConnection::connect(void)
{
	AudioConnection *p;

	if (isConnected) return;
	//初始化时输入的通道大于目标音频流的通道数
	if (dest_index > dst.num_inputs) return;
	__disable_irq();

	p = src.destination_list;//目标列表
	if (p == NULL) {		//如果目标列表为空，则让目标列表等于该类
		src.destination_list = this;
	} else {				
		//如果已有目标连接列表，则将该类放在目标列表的最后
		while (p->next_dest) {	
			if (&p->src == &this->src && &p->dst == &this->dst
				&& p->src_index == this->src_index && p->dest_index == this->dest_index) {
				//Source and destination already connected through another connection, abort
				__enable_irq();
				return;
			}
			p = p->next_dest;
		}
		p->next_dest = this;
	}
	//设置该类的下一个连接类为空
	this->next_dest = NULL;
	//源音频流连接数++
	src.numConnections++;
	//设置活动标志为1
	src.active = true;
	//目标连接数量++
	dst.numConnections++;
	//活动标志置1
	dst.active = true;
	//连接成功
	isConnected = true;

	__enable_irq();
}

//关闭连接
void AudioConnection::disconnect(void)
{
	AudioConnection *p;

	if (!isConnected) return; 
	if (dest_index > dst.num_inputs) return;
	__disable_irq();
	// Remove destination from source list
	p = src.destination_list;//源的目标连接列表为空说明没有进行连接
	if (p == NULL) {
//>>> PAH re-enable the IRQ
		__enable_irq();
		return;
	} else if (p == this) {
		//由于要释放该连接，如果该连接是连接列表中的第一个，则将连接列表的第一个设置为该链接的下一个
		//如果该连接没有下一个则连接列表设置为NULL
		if (p->next_dest) {
			src.destination_list = next_dest;
		} else {
			src.destination_list = NULL;
		}
	} else {
		//如果p不是连接列表中的第一个，则一直循环找到P,然后将其替换为下一个或者替换为NULL
		while (p) {
			if (p == this) {
				if (p->next_dest) {
					p = next_dest;
					break;
				} else {
					p = NULL;
					break;
				}
			}
			p = p->next_dest;
		}
	}
//>>> PAH release the audio buffer properly
	//Remove possible pending src block from destination
	//从目标中释放可能使用的音频块
	if(dst.inputQueue[dest_index] != NULL) {
		AudioStream::release(dst.inputQueue[dest_index]);
		// release() re-enables the IRQ. Need it to be disabled a little longer
		__disable_irq();
		dst.inputQueue[dest_index] = NULL;
	}

	//Check if the disconnected AudioStream objects should still be active
	//连接数--，如果没有连接了则设置状态为不活跃
	src.numConnections--;
	if (src.numConnections == 0) {
		src.active = false;
	}

	dst.numConnections--;
	if (dst.numConnections == 0) {
		dst.active = false;
	}

	isConnected = false;

	__enable_irq();
}


// When an object has taken responsibility for calling update_all()
// at each block interval (approx 2.9ms), this variable is set to
// true.  Objects that are capable of calling update_all(), typically
// input and output based on interrupts, must check this variable in
// their constructors.
//调用update_all时该变量被设置为true
bool AudioStream::update_scheduled = false;

bool AudioStream::update_setup(void)
{
	//如果已经调用update，则调用失败
	if (update_scheduled) return false;
	//设置NVIC优先级
	NVIC_SET_PRIORITY(IRQ_SOFTWARE, 208); // 255 = lowest priority
	//使能NVIC
	NVIC_ENABLE_IRQ(IRQ_SOFTWARE);
	update_scheduled = true;
	return true;
}

void AudioStream::update_stop(void)
{
	//停止NVIC
	NVIC_DISABLE_IRQ(IRQ_SOFTWARE);
	update_scheduled = false;
}

AudioStream * AudioStream::first_update = NULL;

//中断服务
void software_isr(void) // AudioStream::update_all()
{
	AudioStream *p;

	ARM_DEMCR |= ARM_DEMCR_TRCENA;
	ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
	uint32_t totalcycles = ARM_DWT_CYCCNT;
	//digitalWriteFast(2, HIGH);
	for (p = AudioStream::first_update; p; p = p->next_update) {
		if (p->active) {
			uint32_t cycles = ARM_DWT_CYCCNT;
			p->update();//虚函数，每个类的update不同
			// TODO: traverse inputQueueArray and release
			// any input blocks that weren't consumed?
			//检测CPU使用率，并重新设置CPU使用率最大值
			cycles = (ARM_DWT_CYCCNT - cycles) >> 4;
			p->cpu_cycles = cycles;
			if (cycles > p->cpu_cycles_max) p->cpu_cycles_max = cycles;
		}
	}
	//digitalWriteFast(2, LOW);
	//计算周期
	totalcycles = (ARM_DWT_CYCCNT - totalcycles) >> 4;;
	AudioStream::cpu_cycles_total = totalcycles;
	if (totalcycles > AudioStream::cpu_cycles_total_max)
		AudioStream::cpu_cycles_total_max = totalcycles;
}

