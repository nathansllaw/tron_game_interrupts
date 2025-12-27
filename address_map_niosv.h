/*******************************************************************************
 * This file provides address values that exist in the DE10-Lite Computer
 * This file also works for DE1-SoC, except change #define DE10LITE to 0
 ******************************************************************************/

#ifndef __SYSTEM_INFO__
#define __SYSTEM_INFO__

#define DE10LITE 1 // change to 0 for CPUlator or DE1-SoC, 1 for DE10-Lite

/* do not change anything after this line */

#if DE10LITE
 #define BOARD				"DE10-Lite"
 #define MAX_X		160
 #define MAX_Y		120
 #define YSHIFT		  8
#else
 #define MAX_X		320
 #define MAX_Y		240
 #define YSHIFT		  9
#endif


/* Memory */
#define SDRAM_BASE			0x00000000
#define SDRAM_END			0x03FFFFFF
#define FPGA_PIXEL_BUF_BASE		0x08000000
#define FPGA_PIXEL_BUF_END		0x0800FFFF
#define FPGA_CHAR_BASE			0x09000000
#define FPGA_CHAR_END			0x09001FFF

/* Devices */
#define LED_BASE			0xFF200000
#define LEDR_BASE			0xFF200000
#define HEX3_HEX0_BASE			0xFF200020
#define HEX5_HEX4_BASE			0xFF200030
#define SW_BASE				0xFF200040
#define KEY_BASE			0xFF200050
#define JP1_BASE			0xFF200060
#define ARDUINO_GPIO			0xFF200100
#define ARDUINO_RESET_N			0xFF200110
#define JTAG_UART_BASE			0xFF201000
#define TIMER_BASE			0xFF202000
#define TIMER_2_BASE			0xFF202020
#define MTIMER_BASE			0xFF202100
#define RGB_RESAMPLER_BASE    		0xFF203010
#define PIXEL_BUF_CTRL_BASE		0xFF203020
#define CHAR_BUF_CTRL_BASE		0xFF203030
#define ADC_BASE			0xFF204000
#define ACCELEROMETER_BASE		0xFF204020

/* Nios V memory-mapped registers */
#define MTIME_BASE             		0xFF202100

#endif