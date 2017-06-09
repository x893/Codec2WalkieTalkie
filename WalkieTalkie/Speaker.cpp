// WalkieTalkie/Speaker.cpp
// Uses stm32f4_discovery_audio_codec.c to provide audio output to speakers/headphones
// References to "Codec" here refer to the STM32 Discovery audio codec device, and not codec2

#include <stdint.h>
#include <string.h>

extern "C"
{
// stm32f4_discovery_audio_codec.h has been modified: configured for AUDIO_MAL_MODE_CIRCULAR
#include <stm32f4_discovery_audio_codec.h>
}
#include <codec2_fifo.h>

// REVISIT: should be configurable:
#define SPEAKER_VOLUME 100

// The FIFO we are playing from. It will be filled with samples
// to be output while we are not looking
static struct FIFO* fifo;

// Output buffer from where DMA will read output data.
// 160 x 16bit samples, stereo.
int16_t buffer[320];

// AudioFreq is in master clocks per second, looseley related to bits per second
uint32_t SpeakerInit(uint32_t AudioFreq)
{
	return EVAL_AUDIO_Init(OUTPUT_DEVICE_HEADPHONE, SPEAKER_VOLUME, AudioFreq);
}

// This is called in the DMA interrupt to fill the DMA buffer with the next 160 samples
void FillNextBuffer()
{
	if (fifo_read(fifo, buffer, 160) == 0)
	{
		// There are enough samples for the next buffer
		// Expand and duplicate the 160 sample mono buffer into a stereo buffer of 320 samples (160L + 160R)
		// They are interleaved LRLRLRLR......
		// We work backwards.
		int i;
		for (i = 160-1; i >= 0; i--)
			buffer[2 * i] = buffer[(2 * i) + 1] = buffer[i];
	}
	else
	{
		// Insufficient samples in the source fifo, play silence
		memset(buffer, 0, sizeof(buffer));
	}
}

// Starts playing from the FIFO
uint32_t SpeakerStart(struct FIFO* src_fifo)
{
	fifo = src_fifo;
	FillNextBuffer();
	// there is some very broken buffer size maths in EVAL_AUDIO_Play
	return EVAL_AUDIO_Play((uint16_t *)buffer, sizeof(buffer) * 4); // Only stereo is supported
}

// These callbacks are in C land, called by stm32f4_discovery_audio_codec.c
// They are called in audio output DMA interrupt context, so dont muck about.
uint32_t Codec_TIMEOUT_UserCallback(void)
{
	return 0;
}

uint16_t EVAL_AUDIO_GetSampleCallBack(void)
{
	return 0;
}

void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size)
{
	// More output data required
	FillNextBuffer();
}
