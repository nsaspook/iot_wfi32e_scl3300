/* 
 * File:   do_fft.h
 * Author: root
 *
 * Created on November 6, 2022, 1:53 PM
 */

#ifndef DO_FFT_H
#define	DO_FFT_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <xc.h>
#include <p32xxxx.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
	
#include "fft.h"

#define _XTAL_FREQ 100000000UL
	
#define DO_FFT_DRIVER	"V1.005" 
#define DO_FFT_ALIAS	"DO_FFT  "
	
#define fft_sample_rate	400.0f
#define fft_cutoff_freq	50.0f
		
void do_fft(bool);
void __delay_ms_fft(uint32_t);
void __delay_us_fft(uint32_t);
void do_fft_version(void);

FPU do_fft_dc_x(FPU input);
FPU do_fft_dc_y(FPU input);
FPU do_fft_dc_z(FPU input);

extern uint8_t inB[N_FFT];

#ifdef	__cplusplus
}
#endif

#endif	/* DO_FFT_H */

