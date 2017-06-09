// WalkieTalkie/Microphone.cpp
// based loosely on STM32F4-Discovery_FW_V1.1.0/Project/Audio_playback_and_record/src/waverecorder.c

#include <stdint.h>
#include <stm32f4xx.h>
#include <codec2_fifo.h>

extern "C"
{
// Modified by mikem for 4kHz cutoff:
#include <pdm_fir.h>
}

#define HTONS(A)  ((((u16)(A) & 0xff00) >> 8) | (((u16)(A) & 0x00ff) << 8))

/* SPI Configuration defines */
#define SPI_SCK_PIN                       GPIO_Pin_10
#define SPI_SCK_GPIO_PORT                 GPIOB
#define SPI_SCK_GPIO_CLK                  RCC_AHB1Periph_GPIOB
#define SPI_SCK_SOURCE                    GPIO_PinSource10
#define SPI_SCK_AF                        GPIO_AF_SPI2

#define SPI_MOSI_PIN                      GPIO_Pin_3
#define SPI_MOSI_GPIO_PORT                GPIOC
#define SPI_MOSI_GPIO_CLK                 RCC_AHB1Periph_GPIOC
#define SPI_MOSI_SOURCE                   GPIO_PinSource3
#define SPI_MOSI_AF                       GPIO_AF_SPI2

#define AUDIO_REC_SPI_IRQHANDLER          SPI2_IRQHandler

/* Audio recording Samples format (from 8 to 16 bits) */
uint32_t AudioRecBitRes = 16; 
/* Audio recording number of channels (1 for Mono or 2 for Stereo) */
uint32_t AudioRecChnlNbr = 1;

// PDM FIR filter for raw microhone data
// see pdm_fir for details on tuning etc
struct pdm_fir_filter filter;

// Destination FIFO for microphone data
static struct FIFO* fifo;

void Microphone_GPIO_Init(void);
void Microphone_SPI_Init(uint32_t Freq);
void Microphone_NVIC_Init(void);

uint8_t MicrophoneInit(uint32_t AudioFreq)
{
	/* Enable CRC module */
	RCC_AHB1PeriphClockCmd(RCC_AHB1ENR_CRCEN, ENABLE); /* REVISIT is this really required without PDMFilter? */

	/* Configure the GPIOs */
	Microphone_GPIO_Init();
	
	/* Configure the interrupts (for timer) */
	Microphone_NVIC_Init();
	
	/* Configure the SPI */
	Microphone_SPI_Init(AudioFreq);

	/* Init the PDM filter */
	pdm_fir_flt_init(&filter);

	/* Return 0 if all operations are OK */
	return 0;
}  

void Microphone_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIO clocks */
	RCC_AHB1PeriphClockCmd(SPI_SCK_GPIO_CLK | SPI_MOSI_GPIO_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	/* SPI SCK pin configuration */
	GPIO_InitStructure.GPIO_Pin = SPI_SCK_PIN;
	GPIO_Init(SPI_SCK_GPIO_PORT, &GPIO_InitStructure);
  
	/* Connect SPI pins to AF5 */  
	GPIO_PinAFConfig(SPI_SCK_GPIO_PORT, SPI_SCK_SOURCE, SPI_SCK_AF);
  
	/* SPI MOSI pin configuration */
	GPIO_InitStructure.GPIO_Pin =  SPI_MOSI_PIN;
	GPIO_Init(SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(SPI_MOSI_GPIO_PORT, SPI_MOSI_SOURCE, SPI_MOSI_AF);
}

// Freq = 32000 => I2S clock freq 328kHz.
// Freq = 64000 => I2S clock freq 656kHz.
// Freq = 100000 => I2S clock 1024kHz
// According to the data sheet,
// MEMS mic does not operate reliably below 1MHz
void Microphone_SPI_Init(uint32_t Freq)
{
	I2S_InitTypeDef I2S_InitStructure;

	/* Enable the SPI clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
  
	/* SPI configuration */
	SPI_I2S_DeInit(SPI2);
	I2S_InitStructure.I2S_AudioFreq = Freq;
	I2S_InitStructure.I2S_Standard = I2S_Standard_LSB;
	I2S_InitStructure.I2S_DataFormat = I2S_DataFormat_16b;
	I2S_InitStructure.I2S_CPOL = I2S_CPOL_High;
	I2S_InitStructure.I2S_Mode = I2S_Mode_MasterRx;
	I2S_InitStructure.I2S_MCLKOutput = I2S_MCLKOutput_Disable;
	/* Initialize the I2S peripheral with the structure above */
	I2S_Init(SPI2, &I2S_InitStructure);

	/* Enable the Rx buffer not empty interrupt */
	SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
}

void Microphone_NVIC_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3); 
	/* Configure the SPI interrupt priority */
	NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

uint8_t MicrophoneStart(struct FIFO* dest_fifo)
{
	/* This fifo will be filled with microphone data samples by the interrupt handler */
	fifo = dest_fifo;

	/* Enable the Rx buffer not empty interrupt */
	SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
	/* The Data transfer is performed in the SPI interrupt routine */
	/* Enable the SPI peripheral */
	I2S_Cmd(SPI2, ENABLE); 
	
	/* Return 0 if all operations are OK */
	return 0;
}

uint8_t MicrophoneStop(void)
{
	/* Stop conversion */
	I2S_Cmd(SPI2, DISABLE); 
	return 0;
}

extern "C"
{
	// Microphone interrupt handler.
	// Uses the free FIR filter from volkov.oleg, see ../pdm_fir
	// https://my.st.com/public/STe2ecommunities/mcu/Lists/STM32Discovery/Attachments/4449/pdm_fir.zip
	// consider also:
	// http://web.eece.maine.edu/~hummels/classes/ece486/docs/libece486_doc/config__mp45dt02_8c_source.html
	// In this interrupt handler, we have the PDM filter and decimation by 8
	void AUDIO_REC_SPI_IRQHANDLER(void)
	{  
		/* Check if data are available in SPI Data register */
		if (SPI_GetITStatus(SPI2, SPI_I2S_IT_RXNE) != RESET)
		{
			uint16_t app;
			static uint8_t count = 0;

			app = SPI_I2S_ReceiveData(SPI2);
			// FIR Filter
			pdm_fir_flt_put(&filter, app);
			// Decimation by 8
			// Every 8 samples we take an output sample from the filter as our next audio sample
			if ((++count % 8) == 0)
			{
				// Get the final sample and put it in our FIFO
				short sample = pdm_fir_flt_get(&filter, 16);
				fifo_write(fifo, &sample, 1);
				count = 0;
			}
		}
	}
}


