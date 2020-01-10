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

#ifndef AudioStream_h
#define AudioStream_h

#ifndef __ASSEMBLER__
#include <stdio.h>  // for NULL
#include <string.h> // for memcpy
#endif

//TODO: lets try to find an unused timer instead of hardcoding
#if defined(ADAFRUIT_PYBADGE_M4_EXPRESS) || defined(ADAFRUIT_PYGAMER_M4_EXPRESS) || defined(ADAFRUIT_PYGAMER_ADVANCE_M4_EXPRESS)
// TC2 is backlight, TC3 is Tone()
  #define AUDIO_TC TC5
  #define AUDIO_IRQn TC5_IRQn
  #define AUDIO_Handler TC5_Handler
  #define AUDIO_GCLK_ID TC5_GCLK_ID
  #define AUDIO_TC_DMAC_ID_OVF TC5_DMAC_ID_OVF
#else
  #define AUDIO_TC TC2
  #define AUDIO_IRQn TC2_IRQn
  #define AUDIO_Handler TC2_Handler
  #define AUDIO_GCLK_ID TC2_GCLK_ID
  #define AUDIO_TC_DMAC_ID_OVF TC2_DMAC_ID_OVF
#endif

#define WAIT_TC8_REGS_SYNC(x) while(x->COUNT8.SYNCBUSY.bit.ENABLE || x->COUNT8.SYNCBUSY.bit.SWRST);

// AUDIO_BLOCK_SAMPLES determines how many samples the audio library processes
// per update.  It may be reduced to achieve lower latency response to events,
// at the expense of higher interrupt and DMA setup overhead.
//
// Less than 32 may not work with some input & output objects.  Multiples of 16
// should be used, since some synthesis objects generate 16 samples per loop.

// Some parts of the audio library may have hard-coded dependency on 128 samples.
// Please report these on the forum with reproducible test cases.
//音频块样本
#ifndef AUDIO_BLOCK_SAMPLES
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
#define AUDIO_BLOCK_SAMPLES  128
#elif defined(__MKL26Z64__)
#define AUDIO_BLOCK_SAMPLES  64
#elif defined(__SAMD51__)
#define AUDIO_BLOCK_SAMPLES  128
#endif
#endif

//音频采样率
#ifndef AUDIO_SAMPLE_RATE_EXACT
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
#define AUDIO_SAMPLE_RATE_EXACT 44117.64706 // 48 MHz / 1088, or 96 MHz * 2 / 17 / 256
#elif defined(__MKL26Z64__)
#define AUDIO_SAMPLE_RATE_EXACT 22058.82353 // 48 MHz / 2176, or 96 MHz * 1 / 17 / 256
#elif defined(__SAMD51__)

#define AUDIO_CLKRATE (VARIANT_GCLK2_FREQ >> 4)
#define AUDIO_PRESCALER TC_CTRLA_PRESCALER_DIV16
#define AUDIO_SAMPLE_RATE_EXACT 44014.085 // 100 MHz / 16 / 142
#define AUDIO_TC_FREQ 44100
#endif
#endif
#define AUDIO_SAMPLE_RATE AUDIO_SAMPLE_RATE_EXACT


#define IRQ_SOFTWARE EVSYS_4_IRQn

//关中断
// #define __disable_irq() __asm__ volatile("CPSID i":::"memory");
// //开中断
// #define __enable_irq()	__asm__ volatile("CPSIE i":::"memory");
//NVIC设置优先级
#define NVIC_SET_PRIORITY(irqnum, priority)  (NVIC_SetPriority((IRQn_Type)irqnum,priority))
//查询NVIC优先级
#define NVIC_GET_PRIORITY(irqnum) (NVIC_GetPriorityGrouping((IRQn_Type)irqnum))
//设置NVIC等待时间
#define NVIC_SET_PENDING(n)	(NVIC_GetPendingIRQ((IRQn_Type)n))
//NVIC使能中断
#define NVIC_ENABLE_IRQ(n)	(NVIC_EnableIRQ((IRQn_Type)n))
//NVIC取消使能中断
#define NVIC_DISABLE_IRQ(n)	(NVIC_DisableIRQ((IRQn_Type)n))
//调试异常和监视控件
#define ARM_DEMCR		(CoreDebug->DEMCR) // Debug Exception and Monitor Control
//启动调试和监视块
#define ARM_DEMCR_TRCENA		(1 << 24)	 // Enable debugging & monitoring blocks
//DWT控制寄存器
#define ARM_DWT_CTRL		(DWT->CTRL) // DWT control register
//使能循环计数
#define ARM_DWT_CTRL_CYCCNTENA		(1 << 0)		// Enable cycle count
//循环计数寄存器
#define ARM_DWT_CYCCNT		(DWT->CYCCNT) // Cycle count register

#define DMAMEM __attribute__ ((section(".dmabuffers"), used))
#define FASTRUN __attribute__ ((section(".fastrun"), noinline, noclone ))




#ifndef __ASSEMBLER__
class AudioStream;
class AudioConnection;

typedef struct audio_block_struct {
	uint8_t  ref_count;
	uint8_t  reserved1;
	uint16_t memory_pool_index;
	int16_t  data[AUDIO_BLOCK_SAMPLES];
} audio_block_t;


class AudioConnection
{
public:
	////初始赋值 src源音频流	dst目标音频流	src_index 源端口 dest_index 目标端口
	AudioConnection(AudioStream &source, AudioStream &destination) :
		src(source), dst(destination), src_index(0), dest_index(0),
		next_dest(NULL)
		{ isConnected = false;//连接标志 初始设置为false
		  connect(); }			//进行连接
	AudioConnection(AudioStream &source, unsigned char sourceOutput,
		AudioStream &destination, unsigned char destinationInput) :
		src(source), dst(destination),
		src_index(sourceOutput), dest_index(destinationInput),
		next_dest(NULL)
		{ isConnected = false;
		  connect(); }
	friend class AudioStream;//这个类可以直接访问本类的public、private、protected
	~AudioConnection() {	//析构函数
		disconnect();		//断开连接
	}
	void disconnect(void);
	void connect(void);
protected:
	AudioStream &src;
	AudioStream &dst;
	unsigned char src_index;
	unsigned char dest_index;
	AudioConnection *next_dest;	//下一个连接？
	bool isConnected;			//连接标志
};

//分配音频块
#define AudioMemory(num) ({ \
	static DMAMEM audio_block_t data[num]; \
	AudioStream::initialize_memory(data, num); \
})

#define CYCLE_COUNTER_APPROX_PERCENT(n) (((n) + (F_CPU / 32 / AUDIO_SAMPLE_RATE * AUDIO_BLOCK_SAMPLES / 100)) / (F_CPU / 16 / AUDIO_SAMPLE_RATE * AUDIO_BLOCK_SAMPLES / 100))
//音频处理器使用情况
#define AudioProcessorUsage() (CYCLE_COUNTER_APPROX_PERCENT(AudioStream::cpu_cycles_total))
//音频处理器使用最大值
#define AudioProcessorUsageMax() (CYCLE_COUNTER_APPROX_PERCENT(AudioStream::cpu_cycles_total_max))
//音频处理器使用最大值重置
#define AudioProcessorUsageMaxReset() (AudioStream::cpu_cycles_total_max = AudioStream::cpu_cycles_total)
//音频内存使用
#define AudioMemoryUsage() (AudioStream::memory_used)
//音频内存使用最大值
#define AudioMemoryUsageMax() (AudioStream::memory_used_max)
//音频内存使用最大值重置
#define AudioMemoryUsageMaxReset() (AudioStream::memory_used_max = AudioStream::memory_used)

class AudioStream
{
public:
	//num_inputs 音频通道数	inputQueue 音频块输入队列
	AudioStream(unsigned char ninput, audio_block_t **iqueue) :
		num_inputs(ninput), inputQueue(iqueue) {
			active = false;
			destination_list = NULL;
			for (int i=0; i < num_inputs; i++) {
				inputQueue[i] = NULL;			//将两条音频通道的输入队列初始化为空
			}
			// add to a simple list, for update_all
			// TODO: replace with a proper data flow analysis in update_all
			if (first_update == NULL) {	//如果初始音频流为空，即第一次调用
				first_update = this;	//则让初始音频流该类
			} else {			//否则将该音频流放置到音频流的最后
				AudioStream *p;
				for (p=first_update; p->next_update; p = p->next_update) ;
				p->next_update = this;
			}
			next_update = NULL;	//将下一个音频流指向设置为空
			cpu_cycles = 0;		//CPU周期和周期最大值设置为0
			cpu_cycles_max = 0;
			numConnections = 0;	//连接数量为0
		}
	//初始化音频内存
	static void initialize_memory(audio_block_t *data, unsigned int num);
	//CPU使用
	int processorUsage(void) { return CYCLE_COUNTER_APPROX_PERCENT(cpu_cycles); }
	//COU使用最大值
	int processorUsageMax(void) { return CYCLE_COUNTER_APPROX_PERCENT(cpu_cycles_max); }
	//CPU使用最大值重置
	void processorUsageMaxReset(void) { cpu_cycles_max = cpu_cycles; }
	//返回活跃状态
	bool isActive(void) { return active; }
	//CPU使用以及音频块使用数据
	uint16_t cpu_cycles;
	uint16_t cpu_cycles_max;
	static uint16_t cpu_cycles_total;
	static uint16_t cpu_cycles_total_max;
	static uint16_t memory_used;
	static uint16_t memory_used_max;
protected:
	bool active;
	//输入通道
	unsigned char num_inputs;
	//分配音频块
	static audio_block_t * allocate(void);
	//释放音频块
	static void release(audio_block_t * block);
	//将音频块发送到目标
	void transmit(audio_block_t *block, unsigned char index = 0);
	//接收来自输入的块，不能写入
	audio_block_t * receiveReadOnly(unsigned int index = 0);
	//接收来自输入的块，可以写入
	audio_block_t * receiveWritable(unsigned int index = 0);

	static bool update_setup(void);
	static void update_stop(void);
	//等待IRQ_SOFTWARE中断
	static void update_all(void) { NVIC_SET_PENDING(IRQ_SOFTWARE); }

	friend void software_isr(void);
	friend class AudioConnection;
	uint8_t numConnections;
private:
	AudioConnection *destination_list;
	audio_block_t **inputQueue;
	static bool update_scheduled;
	virtual void update(void) = 0;
	static AudioStream *first_update; // for update_all
	AudioStream *next_update; // for update_all
	static audio_block_t *memory_pool;
	static uint32_t memory_pool_available_mask[];
	static uint16_t memory_pool_first_mask;
};

#endif
#endif
