#pragma once

/* PDM FIR low pass filter.
 * The source frequency is expected to be 1024kHz so we are receiving 16 bit words at 64kHz rate MSB first.
 * The filter cutoff frequency is 8kHz. Filter parameters may be easily customized by modifying
 * the pdm_fir.py script and regenerating tables in pdm_fir_.h header.
 */

#include <stdint.h>

#define PDM_FTL_TAPS 16

struct pdm_fir_filter {
	uint16_t buffer[PDM_FTL_TAPS];
	uint16_t next_tap;
};

/* Initialize filter */
void pdm_fir_flt_init(struct pdm_fir_filter* f);

/* Put 16 bits of input PDM signal (MSB first) */
void pdm_fir_flt_put(struct pdm_fir_filter* f, uint16_t bits);

/* Retrieve output value. May be called at any rate since it does not change the filter state.
 * The output ranges from -(2**(out_bits-1)) to +(2**(out_bits-1)). Those values correspond to
 * all 0 or all 1 input signal. Note that the output value may still exceed this range so caller
 * should truncate return value on its own if necessary.
 */
int pdm_fir_flt_get(struct pdm_fir_filter const* f, int out_bits);
