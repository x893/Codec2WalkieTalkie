// WalkieTalkie/main.cpp
//
// Implements a Codec2 encoded walkie-talkie
// on STM32F4 Discovery board, using any of the supported RadioHead Packet Radio or Serial port drivers.
// mikem@airspayce.com

#include <stdint.h>
extern "C"
{
	#include <gdb_stdio.h>
}
#include <codec2_fifo.h>
#include <codec2.h>
#include <wirish.h>

// Define this to enable internal testing from microphone direct to speakers
// No radio required
//#define LOOPBACK_TEST 1

// Define this to build using RF22 radio for comms
#define USE_DRIVER_RF22		1

// Or, define this to build using Serial2 for comms
//#define USE_DRIVER_SERIAL	1

// Microphone uses SPI2:
// PB10 is clock out to the mic
// PC3 is PDM dat in from the mic
extern uint8_t MicrophoneInit(uint32_t AudioFreq);
extern uint8_t MicrophoneStart(struct FIFO* dest_fifo);
extern uint8_t MicrophoneStop(void);

// Speaker uses I2C1 for transfer of audio data to the CS43L22:
// PB9 is SDA
// PB6 is SCL
// Also uses SPI3 as I2S for the CS43L22 control port:
// PC10 is SCK
// PC12 is SD
// Also uses PC7 for MCK and PA4 for LRCK and PD4 for /RESET
extern uint32_t SpeakerInit(uint32_t AudioFreq);
extern uint32_t SpeakerStart(struct FIFO* src_fifo);

struct FIFO* mic_fifo;
struct FIFO* spkr_fifo;

struct CODEC2 *c2;

// Pin numbers for input
#define USER_BUTTON		0

// Pin numbers for output LEDs on the Discovery board
#define LED3_ORANGE		61
#define LED4_GREEN		60
#define LED5_RED		62
#define LED6_BLUE		63

// Red LED while transmitting
#define TRANSMIT_LED	LED5_RED

// Green LED indicates driver initialised OK
#define INIT_OK_LED		LED4_GREEN

// This is the pin number of the accelerometer CS on the Discovery board
#define ACCELEROMETER_CS	67

#if defined(USE_DRIVER_RF22)
	// Radio uses SPI1:
	// PA5 is SCK
	// PA6 is MISO (SDO from radio)
	// PA7 is MOSI (SDI to radio)
	// PA10 is used for chip select NSEL
	// PA2 is input interrupt from NIRQ (if used)
	#include <RH_RF22.h>
	RH_RF22 driver;
#elif defined(USE_DRIVER_SERIAL)
	// Build using Serial port for comms
	// Serial port Serial2
	// Connect PA2 on this Discovery to PA3 on the other
	// Connect PA3 on this Discovery to PA2 on the other
	// Connect GND on both
	// 9600 baud is fast enough
	#include <HardwareSerial.h>
	#include <RH_Serial.h>
	RH_Serial driver(Serial2);
#endif

void setup()
{
	// Pin 0 (PA0) is the User button. 
	// We use it as PTT
	pinMode(USER_BUTTON, INPUT);

	// Pin 60 (PD12, LED4) is the Green 'initialised OK' LED
	pinMode(INIT_OK_LED, OUTPUT);
	digitalWrite(INIT_OK_LED, LOW);

	// Pin 62 (PD14, LED5) is the Red TRansmit LED
	pinMode(TRANSMIT_LED, OUTPUT);
	digitalWrite(TRANSMIT_LED, LOW);

	// Disable the accelerometer, set its CS (pin 67, PE3) high
	// otherwise the accelerometer output to PA6 will interfere with the Radio MISO
	pinMode(ACCELEROMETER_CS, OUTPUT);
	digitalWrite(ACCELEROMETER_CS, HIGH);

#ifndef LOOPBACK_TEST
	// Initialise and configure the driver
	if (driver.init())
		digitalWrite(INIT_OK_LED, HIGH);
	else
		printf("Driver init failed\n");

	#if defined(USE_DRIVER_RF22)

	// For RF22, Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36
	// but thats too slow: air time for each message must be < 20ms, else we cant keep up with the packet rate
	// Fast, wide modulations with short air times per packet 
	// may let us interleave multiple transmitters on the same frequency, perhaps with
	// different destination addresses configured into RadioHead driver.
	driver.setModemConfig(RH_RF22::GFSK_Rb125Fd125); // good, but overkill?
//	driver.setModemConfig(RH_RF22::GFSK_Rb9_6Fd45); // Very choppy
//	driver.setModemConfig(RH_RF22::GFSK_Rb19_2Fd9_6); // Choppy
	// Could also configure Radio frequency here
	// Could configure RadioHead driver addresses, etc here for private(ish) networks etc
	
	#elif defined(USE_DRIVER_SERIAL)

	Serial2.begin(9600);

	#endif

#endif

	// mic_fifo holds samples received by the microhone
	// spkr_fifo holds samples to be output to the speaker
	// We process 160 x 16 bit samples at a time (20ms), so 320 samples gives us double buffering
	mic_fifo = fifo_create(320);
	spkr_fifo = fifo_create(320);

	// Codec2 Mode 3200 encodes 160 shorts into 64 bits (8 bytes)
	c2 = codec2_create(CODEC2_MODE_3200);

	// Codec2 requires 8000 16 bit samples per sec
	// Minimum clock speed for MEMS mic is 1MHz => 1000000/16 = 62500 16 bit samples/per sec
	// so we choose AudioFreq of 100000 (=> 1024kHz I2S clock => 64k 16 bit samples per sec) and a decimation of 8 
	// to get 8k 16 bit samples per sec.
	if (MicrophoneInit(100000))
		printf("MicrophoneInit failed\n");

	// Output stream requires 8k x 16bits per second. EVAL_AUDIO_Play only supports stereo :-(
	// 8000*16*2 = 256kHz I2C clock
	// so we choose AudioFreq of 100000 (=> 256kHz I2C clock)
	if (SpeakerInit(25000)) // 8k x 16 bit samples per second, stereo = 256kHz
		printf("SpeakerInit failed\n");

	// Start the microphone input interrupts. It will insert samples into mic_fifo as they become available.
	MicrophoneStart(mic_fifo);

	// Start the speaker output DMA. It will read samples in batches of 160 from spkr_fifo as we push them in.
	SpeakerStart(spkr_fifo);
}

static short fifo_buffer[160];

void loop()
{
#if LOOPBACK_TEST
	// Internal testing code: take mic input, C2 encode it, C2 decode it and output to speaker
	// No RadioHead driver or radio hardware required
	if (digitalRead(USER_BUTTON))
	{
		digitalWrite(TRANSMIT_LED, HIGH); // Tx LED on
		// PTT is pressed: Transmit mode
		while (digitalRead(USER_BUTTON))
		{
			if (fifo_read(mic_fifo, fifo_buffer, 160) == 0)
			{
				// 160 shorts available from the microphone, encode them
				unsigned char bits[8];
				codec2_encode(c2, bits, fifo_buffer);
				// Now decode them
				codec2_decode(c2, fifo_buffer, bits);
				// And output what should be the same speech as the original input
				fifo_write(spkr_fifo, fifo_buffer, 160);
			}
		}
		digitalWrite(TRANSMIT_LED, LOW); // Tx LED off
	}
#else
	// Implement a walkie talkie with PTT over a RadioHead radio or Serial port driver
	if (digitalRead(USER_BUTTON))
	{
		digitalWrite(TRANSMIT_LED, HIGH); // Tx LED on
		// PTT is pressed: Transmit mode
		while (digitalRead(USER_BUTTON))
		{
			if (fifo_read(mic_fifo, fifo_buffer, 160) == 0)
			{
				// 160 shorts available from the microphone, encode them to 8 bytes
				uint8_t bits[8]; // big enough for any C2 encoded frame
				int bytes_per_frame = (codec2_bits_per_frame(c2) + 7) / 8;
				// This takes about 6.5ms:
				codec2_encode(c2, bits, fifo_buffer);
				// Wait for any previous packet to be sent
				driver.waitPacketSent();
				// Now transmit the 8 encoded bytes
				driver.send(bits, bytes_per_frame);
			}
		}
		digitalWrite(TRANSMIT_LED, LOW); // Tx LED off
	}
	else
	{
		// PTT is released: Receive mode
		// Wait for any previous packet to be sent
		driver.waitPacketSent();
		while ( ! digitalRead(USER_BUTTON))
		{
			uint8_t bits[8];// big enough for any C2 encoded frame
			uint8_t len = sizeof(bits);
			int bytes_per_frame = (codec2_bits_per_frame(c2) + 7) / 8;
			if (driver.recv(bits, &len) && len == bytes_per_frame)
			{
				// Got encoded data, decode it
				codec2_decode(c2, fifo_buffer, bits);
				// And output what should be the same speech as the original input on the transmitter side
				fifo_write(spkr_fifo, fifo_buffer, 160);
			}
	 	}
	}
#endif
}
