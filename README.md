This is the Codec2WalkieTalkie

It implements a Codec2 (http://www.rowetel.com/blog/codec2.html) encoded
walkie-talkie on an STM32F4 Discovery board
http://www.st.com/web/catalog/tools/FM116/SC959/SS1532/PF252419

Latest version can be found at
http://www.airspayce.com/mikem/Codec2WalkieTalkie

Codec2 is a very clever voice compression library by David Rowe that can
provide understandable voice communications over very narrow bandwidth
channels. Originally designed for SSB radio communications, here we use it to
provide voice communications over low duty cycle packet radio channels or slow
serial connections.

Each station has:
STM32F4 Discovery board (using onboard MEMS microphone and Audio out chip to 
headphone, and user button as PTT) and ST Firmware
Codec2 in CODEC2_MODE_3200
Optional HopeRF RFM22 radio module (connected to SPI1)
RadioHead drivers with some Arduino compatibility code
The free pdm_fir FIR filter from volkov.oleg

The MEMS Microphone is sampled at 1024kHz and FIR filtered to 4kHz bandwidth 
(this all happens in an interrupt handler).

When the PTT button is pressed, microphone samples (160x16bits at a time = 
20ms of voice) are compressed with Codec2 MODE_3200, producing 8 bytes of 
compresed voice. These 8 bytes are then sent by RadioHead::RH_RF22 driver 
(sent using the broadcast address).

When the PTT is not pressed, it listens for 8 byte messages received from 
RadioHead::RH_RF22. Each one is decompressed with Codec2 and the resulting 
160x16bit = 20ms samples are duplicated to stereo and sent to the output DAC 
by DMA, and can be herd through headphones.

The RF22 modem is configured for GFSK_Rb125Fd125 (125kHz bandwidth, 125kHz 
deviation GFSK), which gives good quality sound and plenty of extra air space 
for interleaved transmitters (possibly privately addressed).

Over the air, by analog radio, it just sounds like 50Hz buzz.

Lower data rates also work. Min workable RF22 modulation is prob about 9.6kbps 
(based on RadioHead addressing/preamble/header/crc overheads etc)

You can also use RadioHead's RH_Serial driver for transmitting 
quality voice over 9600 baud serial.

Should also (courtesy of RadioHead) be able to support the very interesting 
RFM95 LoRa family of radios, with long range and spread spectrum.

Or, using the RadioHead RH_RFM69 driver, you could also have AES encryption of 
the radio data for extra security.

Or, switch between all types of RadioHead driver.

See INSTALL for building instructions.

This is NOT a finished product. It is intended to be a proof-of-concept for
further experimentation by other amateurs.

Please dont ask me to support you in your experimentation. Over to you.....

Mike McCauley
mikem@airspayce.com
