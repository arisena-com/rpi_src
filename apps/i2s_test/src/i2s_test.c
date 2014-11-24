/*****************************************************************************
 * Copyright (C)  07 Nov 2014 Simon Derek Hughes
 *
 * i2s_test4.c
 *  Raspberry pi I2S bus test application with information and know-how
 *  addressing BCM2835-ARM-Peripherals.pdf documentation omissions and
 *  inaccuracies.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 *  For more information, contact Simon Hughes at:
 *    Email:    simon.d.hughes@arisena.com
 *    Address:  Arisena, 11 Gough Way, Cambridge, CB3 9LN, UK.
 *
 *****************************************************************************
 * Versions
 *  v0.1    20141103    First Working Version
 *  v0.2    20141103    Comments added.
 *  v0.21   20141104    Cmdline processing added
 *  v0.22   20141104    Annotation of functions, and starting to document main
 *                      clk configuration
 *  v0.23   20141105    Getting PCM_CLOCK registers CM_PCMCTRL and CM_PCMDIV
 *                      configured correctly
 *  v0.24   20141106    Code to help test frequencies as measured by logic
 *                      analyser.
 *  v0.25   20141107    Version generates 256kHz, 512kHz and 1.536MHZ clocks
 *                      correctly (approx) with OSC, PLLC & PLLD (there is some
 *                      error which suggests the xls is not quite right or the
 *                      precision is off, but I have to check whether the
 *                      measurements are within measurement error in detail).
 *  v1.00   20141107    Published to EJR for review.
 *  v1.01   20141107    256kHz, 512kHz and 1.536MHZ clocks now accurately
 *                      generated (see i2s_test4_test_vectors_v2.1.xls).
 *                      Corrected {SRC, MASH, DIVI, DIVF} test vectors
 *                      Corrected version of average clk output frequency
 *                      equation (see test_vector_table)
 *  v1.02   20141111    working register functions.
 *  v1.03   20141121    version sent to MB.
 *  v2.00   20141124    First publicly released version.
 *****************************************************************************
 *
 * Notes
 *  This is a test application which:
 *  - uses mmap() to map the GPIO registers into the virtual address space of
 *    this application so that they can be configured from user mode.
 *  - configures the GPIO pins to be mode ALT2 which uses GPIO28-31 for an
 *    I2S bus
 *  - writes the pattern 0xA0A0A0A0 out repeatedly on I2S_DOUT, which can
 *    be detected on the ZPLC, for example.
 *
 * See PROG_HELP define for the command line help.
 *
 * REFERENCES
 * REF1
 *   BCM2835 ARM Peripherals 6 Feb 2012 Broadcom Europe
 *   BCM2835-ARM-Peripherals.pdf
 *
 * REF2
 *   BCM2835_Audio_PWM_Clocks_errata_Geert_Van_Loo.doc which is images
 *   captured from http://www.scribd.com/doc/127599939/BCM2835-Audio-clocks
 *
 * REF3
 *  http://raspberrypi.stackexchange.com/questions/1153/what-are-the-different-
 *      clock-sources-for-the-general-purpose-clocks
 *
 *  which reports the following:
 *   0     0 Hz     Ground
 *   1     19.2 MHz oscillator
 *   2     0 Hz     testdebug0
 *   3     0 Hz     testdebug1
 *   4     0 Hz     PLLA
 *   5     1000 MHz PLLC (changes with overclock settings)
 *   6     500 MHz  PLLD
 *   7     216 MHz  HDMI auxiliary
 *   8-15  0 Hz     Ground
 *
 *  The REF1 table 6-34 doesn't report the clock frequencies
 *
 * REF4
 *  i2s_test4_test_vector_vy.yy.xls computes values for test vectors
 *  see docs dir for latest version
 *
 * Building on x86
 *  The code can be built and run to test those portions of the code not
 *  dependent on rpi hw. If run as a user on an ubuntu box, access to /dev/mem
 *  will not be allowed, and the app will exit when the file open fails.
 *
 ****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <dirent.h>
#include <fcntl.h>
#include <assert.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <inttypes.h>
#include <unistd.h>
#include <assert.h>

#include "debug.h"


/******************************************************************************
 * Global variables & constants
 *
 * Note on Regression Tests:
 *  regression tests invoke the application with the -q option so that no
 *  printf output is generated other than that relevant for regression testing.
 ******************************************************************************/
#ifdef _DEBUG
int rpi_optDebug = 1;               /* set if debug trace is enabled */
int rpi_optLogLevel = LOG_DEBUG;    /* verbosity level of debug trace */
#else
int rpi_optDebug = 0;
int rpi_optLogLevel = LOG_NOTICE;   /* set to LOG_NONE to disable tracing */
#endif /* _DEBUG */


/******************************************************************************
 * DEFINES
 *
 * BCM2708_PERI_BASE
 * This is the ARM physical address of the base of the BCM20708 SoC
 * configuration registers.
 *
 * See REF1, figure on P5 and section 1.2.3 ARM Physical addresses.
 *  - The IO peripherals appear at addresses starting 0x20000000 in the ARM
 *    physical address space. See (2) in Fig1 on page 5 with hand written
 *    notes.
 *  - The IO peripherals appear at addresses starting 0x7E000000 in the
 *    VideoCore peripheral bus address space. See (1) in Fig1 on page 5
 *    with hand written notes.
 *  - The IO peripherals are mapped with mmap() to virtual addresses between
 *    0x00000000 and 0xC0000000 in the ARM virtual address space. See (3) in
 *    Fig 1 on page 5 with hand written notes.
 *
 * The tables in REF1 use VideoCore peripheral bus addresses which begin
 * with 0x7eXXXXXX. This program uses /dev/mem which allows ARM physical
 * addresses beginning 0x20XXXXXX to be accessed. Replace the 0x7e in the
 * REF1 addresses with 0x20 to get the equivalent physical address for
 * this application.
 *
 * Note these symbol definitions:
 *      BCM2708_PERI_BASE
 *      GPIO_BASE
 *      I2S_BASE
 *      CLOCK_BASE
 * have been copied from the platform.h file here:
 *   <KERNEL_SRC_ROOT>/arch/arm/mach-bcm2708/include/mach/platform.h
 * I've included the option of defining KERNEL_SRC_ROOT and including
 * the header file.
 *
 * KERNEL_SRC_ROOT is defined in the ENV if source setup.sh has been run.
 * If this is defined then the BCM2708_PERI_BASE etc symbols can be
 * included from platform.h. Otherwise, the values copied here are used.
 *
 * GPIO_BASE
 * This is the start of the GPIO configuration registers. see REF1.
 *
 * I2S_BASE
 * This is the start of the I2S configuration registers. see REF1.
 *
 * CLOCK_BASE
 * This is the start of the clock configuration registers. see REF1.
 *
 ****************************************************************************/

/* command line processing option flags */
#define I2S_CMD_OPT_F_DIVF      1<<0
#define I2S_CMD_OPT_F_DIVI      1<<1
#define I2S_CMD_OPT_F_MASH      1<<2
#define I2S_CMD_OPT_F_SRC       1<<3

#define I2S_CMD_OPT_F_REQUIRED  (I2S_CMD_OPT_F_DIVF | I2S_CMD_OPT_F_DIVI | I2S_CMD_OPT_F_MASH | I2S_CMD_OPT_F_SRC)

#ifdef KERNEL_SRC_ROOT
/* it would be better to create a symlink in the kernel tree at
 *  $KERNEL_SRC_ROOT/include/mach -> ../arch/arm/mach-bcm2708/include/mach
 * but this states it explicitly for reference */
#include <arch/arm/mach-bcm2708/include/mach/platform.h>

#else
/* values copied from platform.h */
#define BCM2708_PERI_BASE        0x20000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */
#define I2S_BASE                 (BCM2708_PERI_BASE + 0x203000) /* GPIO controller */

#endif /* KERNEL_SRC_ROOT */

/* defines not copied from platform.h */
#define CLOCK_BASE               (BCM2708_PERI_BASE + 0x101000) /* Clocks */

#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)


/* GPIO Registers for I2S BUS*/
#define GPI018_ALT0_PCM_CLK     18
#define GPI019_ALT0_PCM_FS      19
#define GPI020_ALT0_PCM_DIN     20
#define GPI021_ALT0_PCM_DOUT    21
#define GPI028_ALT2_PCM_CLK     28
#define GPI029_ALT2_PCM_FS      29
#define GPI030_ALT2_PCM_DIN     30
#define GPI031_ALT2_PCM_DOUT    31

/* PCM/I2S Registers */
#define CS_A     0
#define FIFO_A   1
#define MODE_A   2
#define RXC_A    3
#define TXC_A    4
#define DREQ_A   5
#define INTEN_A  6
#define INTSTC_A 7
#define GRAY     8

const char *i2s_register_name[] = {"CS_A", "FIFO_A", "MODE_A", "RXC_A", "TXC_A", "DREQ_A", "INTEN_A", "INTSTC_A", "GRAY"};

#define PCM_CS_A_OFFSET     0x00000000
#define PCM_FIFO_A_OFFSET   0x00000004
#define PCM_MODE_A_OFFSET   0x00000008
#define PCM_RXC_A_OFFSET    0x0000000c
#define PCM_TXC_A_OFFSET    0x00000010
#define PCM_DREQ_A_OFFSET   0x00000014
#define PCM_INTEN_A_OFFSET  0x00000018
#define PCM_INTSTC_A_OFFSET 0x0000001c
#define PCM_GRAY_OFFSET     0x00000020

/* PCM/I2S Register Bitfield settings & flags */
#define PCM_CS_A_F_EN               (1<<0)          /* enable PCM interface */
#define PCM_CS_A_F_RXON             (1<<1)          /* enable Rx interface */
#define PCM_CS_A_F_TXON             (1<<2)          /* enable Tx interface */
#define PCM_CS_A_F_TXCLR            (1<<3)          /* Clear the TX FIFO */
#define PCM_CS_A_F_RXCLR            (1<<4)          /* Clear the RX FIFO */
#define PCM_CS_A_TXTHR              (0x3<<5)        /* Tx fifo threshold */
#define PCM_CS_A_RXTHR              (0x2<<7)        /* Rx fifo threshold */
#define PCM_CS_A_F_TXD              (1<<19)         /* indicates TX FIFO can accept data */
#define PCM_CS_A_F_SYNC             (1<<24)         /* PCM Clock sync helper */
#define PCM_CS_A_F_STBY             (1<<25)         /* RAM Standby */

#define PCM_TXC_A_CH2WID            (0x8<<0)       /* => 16 bits wide */
#define PCM_TXC_A_CH2POS            (33<<4)        /* 33rd clock of frame for channel 2 first data bit */
#define PCM_TXC_A_F_CH2EN           (1<<14)        /* enable channel 1 */
#define PCM_TXC_A_F_CH2WEX_RESET    (0<<15)        /* channel 2 not using width extension (L channel) */
#define PCM_TXC_A_CH1WID            (0x8<<16)      /* => 16 bits wide */
#define PCM_TXC_A_CH1POS            (1<<20)        /* 2nd clock of frame for channel 1 first data bit */
#define PCM_TXC_A_F_CH1WEX_RESET    (0<<30)        /* channel 1 not using width extension (R channel) */
#define PCM_TXC_A_F_CH1EN           (1<<30)        /* enable channel 1 */

#define PCM_MODE_A_FSLEN            (32<<0)        /* PCM_FS is held active (hig) for first 32 clocks in frame */
#define PCM_MODE_A_FLEN             (63<<10)       /* frame len 63 => there will be 64 clocks in a frame */
#define PCM_MODE_A_F_FTXP           (1<<24)        /* tx frame packet mode: tx fifo split into 2 16 bit words */

/* REF1 Sec 6.3 & REF32 Sec 1.1 specify PCM/PWM max operating frequency as
 * 25MHz */
#define RPI_MAX_FREQ_HZ         25000000

/*  CM_PCMCTRL  0x73101098          0x20101098
 *  CM_PCMDIV   0x73101098          0x2010109c */
#define CM_PCMCTRL_OFFSET       0x00000098
#define CM_PCMDIV_OFFSET        0x0000009c
#define CM_PCM_REG_SIZE_BYTES   4

/* CM_PCMCTRL register offsets for bit fields */
#define CM_PCMCTRL_SRC_LSB_OFFSET   0           /* bits 0:3 */
#define CM_PCMCTRL_ENAB_LSB_OFFSET  4           /* bit 4 */
#define CM_PCMCTRL_BUSY_OFFSET      7           /* bit 7 */
#define CM_PCMCTRL_MASH_LSB_OFFSET  9           /* MASH bits 9:10 */

#define CM_PCMCTRL_BUSY             (1<<CM_PCMCTRL_BUSY_OFFSET)

/* CM_PCMDIV register offsets for bit fields */
#define CM_PCMDIV_DIVF_LSB_OFFSET   0           /* DIVI bits 0:11 */
#define CM_PCMDIV_DIVI_LSB_OFFSET   12          /* DIVI bits 12:23 */

/* default values */
#define CM_PCMCTRL_SRC_DEF      1   /* CM_PCMCTRL clock src setting, default to using the 19.2MHz osc (shown on schmatics as 19M2) */
#define CM_PCMCTRL_MASH_DEF     0   /* CM_PCMCTRL clock mash setting *default to no mash, so just using integer divider */
#define CM_PCMDIV_DIVI_DEF      10; /* CM_PCMDIV DIVI setting, note, frequency on module should not exceed 25MHz, so dont let the PLLs driver high frequencies as it might damage the module */
#define CM_PCMDIV_DIVF_DEF      1;  /* CM_PCMDIV DIVF setting */

/* defines for supported clock sources src */
#define CM_PCMCTRL_SRC_OSC          1
#define CM_PCMCTRL_SRC_PLLA         4
#define CM_PCMCTRL_SRC_PLLC         5
#define CM_PCMCTRL_SRC_PLLD         6
#define CM_PCMCTRL_SRC_HDMI_AUX     7
#define CM_PCMCTRL_SRC_MAX          0xffffffff
#define CM_PCMCTRL_MASH_MAX         3

/* clock source frequencies (documentation is wrong)
 * REF3 deduced these values by experiment
 * */
#define CM_PCMCTRL_SRC_GND_FREQ_0MHZ        0
#define CM_PCMCTRL_SRC_OSC_FREQ_19_2MHZ     19200000
#define CM_PCMCTRL_SRC_PLLA_FREQ_0_HZ       0               /* untested*/
#define CM_PCMCTRL_SRC_PLLC_FREQ_500MHZ     1000000000
#define CM_PCMCTRL_SRC_PLLD_FREQ_1GHZ       500000000
#define CM_PCMCTRL_SRC_HDMI_AUX_FREQ_0HZ    0               /* untested */
#define CM_PCMCTRL_SRC_MAX_FREQ_HZ          0xffffffff


static unsigned int cm_pcmctrl_src_supported[] =
{
    CM_PCMCTRL_SRC_OSC,
    CM_PCMCTRL_SRC_PLLA,
    CM_PCMCTRL_SRC_PLLC,
    CM_PCMCTRL_SRC_PLLD,
    CM_PCMCTRL_SRC_HDMI_AUX,
    CM_PCMCTRL_SRC_MAX
};

static unsigned int cm_pcmctrl_src_freq_ref[] =
{
    CM_PCMCTRL_SRC_GND_FREQ_0MHZ,
    CM_PCMCTRL_SRC_OSC_FREQ_19_2MHZ,
    CM_PCMCTRL_SRC_MAX_FREQ_HZ,
    CM_PCMCTRL_SRC_MAX_FREQ_HZ,
    CM_PCMCTRL_SRC_PLLA_FREQ_0_HZ,
    CM_PCMCTRL_SRC_PLLC_FREQ_500MHZ,
    CM_PCMCTRL_SRC_PLLD_FREQ_1GHZ,
    CM_PCMCTRL_SRC_HDMI_AUX_FREQ_0HZ,
    CM_PCMCTRL_SRC_MAX_FREQ_HZ
};

/*CM_PCMDIV register bit fields max values */
#define CM_PCMDIV_DIVI_MAX        (1<<12)
#define CM_PCMDIV_DIVF_MAX        (1<<12)

/* test vector number defines */
#define IS2_CMD_OPT_TEST_VECTOR_OSC_M1_256KBS       0
#define IS2_CMD_OPT_TEST_VECTOR_OSC_M1_512KBS       1
#define IS2_CMD_OPT_TEST_VECTOR_OSC_M1_536MBS       2
#define IS2_CMD_OPT_TEST_VECTOR_PLLC_M1_256KBS      3
#define IS2_CMD_OPT_TEST_VECTOR_PLLC_M1_512KBS      4
#define IS2_CMD_OPT_TEST_VECTOR_PLLC_M1_536MBS      5
#define IS2_CMD_OPT_TEST_VECTOR_PLLD_M1_256KBS      6
#define IS2_CMD_OPT_TEST_VECTOR_PLLD_M1_512KBS      7
#define IS2_CMD_OPT_TEST_VECTOR_PLLD_M1_536MBS      8
#define IS2_CMD_OPT_TEST_VECTOR_MAX                 9

typedef struct test_vector_table_entry_t
{
    unsigned int src;   /* CM_PCMCTRL clock src setting */
    unsigned int mash;  /* CM_PCMCTRL clock mash setting */
    unsigned int divi;  /* CM_PCMDIV DIVI setting */
    unsigned int divf;  /* CM_PCMDIV DIVF setting */
} test_vector_table_entry_t;

/* values taken from REF4 using ave_output_freq = freq_src/(DIVI + DIVF/4096)
 * REF1 Sec 6.3 incorrectly documents 1024 as the divisor to DIVF */
test_vector_table_entry_t test_vector_table[] =
{
        /* src              mash DIVI DIVF */
        {CM_PCMCTRL_SRC_OSC,  1, 75,   0},      /* 256KHz*/
        {CM_PCMCTRL_SRC_OSC,  1, 37,   2048},   /* 512KHz*/
        {CM_PCMCTRL_SRC_OSC,  1, 12,   2048},   /* 1.536MBs */
        {CM_PCMCTRL_SRC_PLLC, 1, 3906, 1024},   /* 256KHz*/
        {CM_PCMCTRL_SRC_PLLC, 1, 1953, 512},    /* 512KHz*/
        {CM_PCMCTRL_SRC_PLLC, 1, 651,  170},    /* 1.536MBs */
        {CM_PCMCTRL_SRC_PLLD, 1, 1953, 512},    /* 256KHz*/
        {CM_PCMCTRL_SRC_PLLD, 1, 976,  2304},   /* 512KHz*/
        {CM_PCMCTRL_SRC_PLLD, 1, 325,  2133},   /* 1.536MBs */
        {CM_PCMCTRL_SRC_MAX,  0, 0,    0},      /* sentinel */
};

/* globals for command line options */
unsigned int cm_pcmctrl_src = CM_PCMCTRL_SRC_DEF;   /* CM_PCMCTRL clock src setting */
unsigned int cm_pcmctrl_mash = CM_PCMCTRL_MASH_DEF; /* CM_PCMCTRL clock mash setting */
unsigned int cm_pcmdiv_divi = CM_PCMDIV_DIVI_DEF;   /* CM_PCMDIV DIVI setting, note, frequency on module should not exceed 25MHz, so dont let the PLLs driver high frequencies as it might damage the module */
unsigned int cm_pcmdiv_divf = CM_PCMDIV_DIVF_DEF;   /* CM_PCMDIV DIVF setting */

typedef struct bcm2835_map_t
{
    char* mem;                      /* malloc memory block page boundary aligned address*/
    char* mem_na;                   /* malloc memory as returned from malloc (Not Aligned to block page boundary); needed for free() */
    volatile char* mmap_addr;       /* mmap()  */
} bcm2835_map_t;


typedef struct bcm2835_i2s_t
{
    int  mem_fd;                /* file descriptor for /dev/mem, the file object to be mapped */
    bcm2835_map_t gpio_base;    /* gpio configuration area*/
    bcm2835_map_t i2s_base;     /* i2s configuration area*/
    bcm2835_map_t clk_base;     /* clk configuration area*/
} bcm2835_i2s_t;

/* **the** device context */
static bcm2835_i2s_t bcm2835_i2s;

static inline unsigned int i2st_gpio_reg_get(bcm2835_i2s_t* ctx, unsigned int num)
{
    return *(volatile unsigned *)(ctx->gpio_base.mmap_addr+(sizeof(unsigned int) * num));
}

static inline void i2st_gpio_reg_set(bcm2835_i2s_t* ctx, unsigned int num, unsigned int val)
{
    *(volatile unsigned *)(ctx->gpio_base.mmap_addr+(sizeof(unsigned int) * num)) = val;
    return;
}

static inline unsigned int i2st_pcm_reg_get(bcm2835_i2s_t* ctx, unsigned int num)
{
    return *(volatile unsigned *)(ctx->i2s_base.mmap_addr+(sizeof(unsigned int) * num));
}

static inline void i2st_cm_pcmctrl_set(bcm2835_i2s_t* ctx, unsigned int val)
{
    *(volatile unsigned *)(ctx->clk_base.mmap_addr+CM_PCMCTRL_OFFSET) = val;
    return;
}

static inline int i2st_cm_pcmctrl_wait_not_busy(bcm2835_i2s_t* ctx)
{
    int i = 100;
    /* wait for the busy flag to be cleared */
    while( (*(volatile unsigned *)(ctx->clk_base.mmap_addr+CM_PCMCTRL_OFFSET) & CM_PCMCTRL_BUSY) && i > 0)
    {
        usleep(100);
        i--;
    }
    return i;
}

static inline int i2st_cm_pcmctrl_wait_busy(bcm2835_i2s_t* ctx)
{
    int i = 100;
    /* wait for the busy flag to be set */
    while( !(*(volatile unsigned *)(ctx->clk_base.mmap_addr+CM_PCMCTRL_OFFSET) & CM_PCMCTRL_BUSY) && i > 0)
    {
        usleep(100);
        i--;
    }
    return i;
}

static inline void i2st_cm_pcmdiv_set(bcm2835_i2s_t* ctx, unsigned int val)
{
    *(volatile unsigned *)(ctx->clk_base.mmap_addr+CM_PCMDIV_OFFSET) = val;
    return;
}

static inline unsigned int i2st_pcm_cs_a_get(bcm2835_i2s_t* ctx)
{
    return *(volatile unsigned *)(ctx->i2s_base.mmap_addr+PCM_CS_A_OFFSET);
}

static inline void i2st_pcm_cs_a_set(bcm2835_i2s_t* ctx, unsigned int val)
{
    *(volatile unsigned *)(ctx->i2s_base.mmap_addr+PCM_CS_A_OFFSET) = val;
    return;
}

static inline void i2st_pcm_fifo_a_set(bcm2835_i2s_t* ctx, unsigned int val)
{
    *(volatile unsigned *)(ctx->i2s_base.mmap_addr+PCM_FIFO_A_OFFSET) = val;
    return;
}

static inline void i2st_pcm_mode_a_set(bcm2835_i2s_t* ctx, unsigned int val)
{
    *(volatile unsigned *)(ctx->i2s_base.mmap_addr+PCM_MODE_A_OFFSET) = val;
    return;
}

static inline unsigned int i2st_pcm_mode_a_get(bcm2835_i2s_t* ctx)
{
    return *(volatile unsigned *)(ctx->i2s_base.mmap_addr+PCM_MODE_A_OFFSET);
}

static inline void i2st_pcm_txc_a_set(bcm2835_i2s_t* ctx, unsigned int val)
{
    *(volatile unsigned *)(ctx->i2s_base.mmap_addr+PCM_TXC_A_OFFSET) = val;
    return;
}



/* MASH table callback functions and structs to go in table entries */
static int i2s_calc_output_freq_core(unsigned int ref_freq, double val, double* result)
{
    int ret = -1;
    if(val > 0 && result != NULL)
    {
        *result = ref_freq / val;
        ret = 0;
    }
    return ret;
}

static int i2s_calc_min_output_freq_mash0(unsigned int ref_freq, unsigned int divi, double* result)
{
    return i2s_calc_output_freq_core(ref_freq, (double) divi, result);
}

static int i2s_calc_min_output_freq_mash1(unsigned int ref_freq, unsigned int divi, double* result)
{
    return i2s_calc_output_freq_core(ref_freq, (double) divi+1, result);
}

static int i2s_calc_min_output_freq_mash2(unsigned int ref_freq, unsigned int divi, double* result)
{
    return i2s_calc_output_freq_core(ref_freq, (double) divi+2, result);
}

static int i2s_calc_min_output_freq_mash3(unsigned int ref_freq, unsigned int divi, double* result)
{
    return i2s_calc_output_freq_core(ref_freq, (double) divi+4, result);
}

static int i2s_calc_max_output_freq_mash0(unsigned int ref_freq, unsigned int divi, double* result)
{
    return i2s_calc_output_freq_core(ref_freq, (double) divi, result);
}

static int i2s_calc_max_output_freq_mash1(unsigned int ref_freq, unsigned int divi, double* result)
{
    return i2s_calc_output_freq_core(ref_freq, (double) divi, result);
}

static int i2s_calc_max_output_freq_mash2(unsigned int ref_freq, unsigned int divi, double* result)
{
    return i2s_calc_output_freq_core(ref_freq, (double) divi-1, result);
}

static int i2s_calc_max_output_freq_mash3(unsigned int ref_freq, unsigned int divi, double* result)
{
    return i2s_calc_output_freq_core(ref_freq, (double) divi-3, result);
}

static int i2s_calc_ave_output_freq_mash0(unsigned int ref_freq, unsigned int divi, unsigned int divf, double* result)
{
    return i2s_calc_output_freq_core(ref_freq, (double) divi, result);
}

static int i2s_calc_ave_output_freq_mash(unsigned int ref_freq, unsigned int divi, unsigned int divf, double* result)
{
    return i2s_calc_output_freq_core(ref_freq, (double) (divi + (divf/1024)), result);
}

typedef struct mash_tentry_t
{
    unsigned int num_stages;
    unsigned int min_divi;
    int (*calc_max_output_freq)(unsigned int ref_freq, unsigned int divi, double* result);
    int (*calc_ave_output_freq)(unsigned int ref_freq, unsigned int divi, unsigned int divf, double* result);
    int (*calc_min_output_freq)(unsigned int ref_freq, unsigned int divi, double* result);
} mash_tentry_t;

/* MASH table corresponding to REF1 table 6-32 Sec 6.3 P107 */
mash_tentry_t mash_table_6_32[] =
{
        {0, 1, i2s_calc_max_output_freq_mash0, i2s_calc_ave_output_freq_mash0, i2s_calc_min_output_freq_mash0},
        {1, 2, i2s_calc_max_output_freq_mash1, i2s_calc_ave_output_freq_mash,  i2s_calc_min_output_freq_mash0},
        {2, 3, i2s_calc_max_output_freq_mash2, i2s_calc_ave_output_freq_mash,  i2s_calc_min_output_freq_mash0},
        {3, 5, i2s_calc_max_output_freq_mash3, i2s_calc_ave_output_freq_mash,  i2s_calc_min_output_freq_mash0},
        {0xffffffff, 0xffffffff, NULL, NULL,  NULL}
};


#define REG_GPFSEL_FSELN_BITFIELD_SZ    3       /* number of configuration bits per port */
#define REG_GPFSEL_NUM_FSELN            10      /* number of GPIO ports per GPFSEL register */

/*****************************************************************************
 * FUNCTION: i2st_gpio_pin_set_input
 ****************************************************************************
 * ARGS
 *  ctx
 *  gpio_pin_num     number of the gpio pin to configure as an input
 *
 * Function to configure a particular GPIO port number gpio_pin_num to
 * be INPUT. It sets the GPIO configuration bits for the GPIO port
 * number gpio_pin_num corresponding to the FSELn field i.e.
 *      FSEL(gpio_pin_num % 10)
 * bitfield in the GPFSEL(gpio_pin_num/10) register to 0b000,
 * thus configuring the GPIO port to be an input. GPIO ports reset state is
 * input.
 *
 * If we look at the table REF1 Sec 6.1 P90-91 it shows
 * - to configure the 54 GPIO ports, 3 configuration bits are allocated to
 *   each port.
 * - GPIO ports 0-9   (10 in total) are configured in GPFSEL0.
 * - GPIO ports 10-19 (10 in total) are configured in GPFSEL1.
 * - GPIO ports 20-29 (10 in total) are configured in GPFSEL2.
 * - GPIO ports 30-39 (10 in total) are configured in GPFSEL3.
 * - GPIO ports 40-49 (10 in total) are configured in GPFSEL4.
 * - GPIO ports 50-53 (4 in total)  are configured in GPFSEL5.
 * - GPFSELn stands for GPIO Function Selection Register because
 *   they are used to select the function (ALT mode) of each of the GPIO
 *   ports. There are up to 6 alternative modes.
 * - The configuration bit field for GPIO port m within a GPFSELn is labelled
 *   FSELm e.g. for GPFSEL0 the
 *    - GPIO port 0 configuration field is FSEL0 i.e. bits 0:2
 *    - GPIO port 5 configuration field is FSEL5 i.e. bits 5*3 : 5*3+2
 *      i.e. 15:17, because each bitfield is 3 bits wide.
 *
 * The function works like this:
 * - for a GPIO port n = gpio_pin_num = {0..53}, the n in GPFSELn which
 *   contains the configuration bits is gpio_pin_num/10. Hence the function
 *   part:
 *      gpio_pin_num/REG_GPFSEL_NUM_FSELN
 *   finds the address of the GPFSELn register which contains GPIO port
 *   gpio_pin_num.
 * - for a GPIO port gpio_pin_num = {0..53} in GPFSELn, the configuration bit
 *   field is (gpio_pin_num % 10) * 3 i.e. gpio_pin_num % 10 is the remainder
 *   after dividing by 10 to give the number of 3 bit fields that have to be
 *   skipped to access the correct one
 * - E.g.
 *       (7 << ( (gpio_pin_num % REG_GPFSEL_NUM_FSELN) * REG_GPFSEL_FSELN_BITFIELD_SZ) )
 *       7<<( ( gpio_pin_num % 10) * 3)
 *   for gpio_pin_num=5 produces the bit pattern
 *
 *      0000 0000 0000 0011 1000 0000 0000 0000
 *                       ----
 *                       3 bits corresponding to FSEL5
 *
 * - ~(7<<((gpio_pin_num)%10)*3) for gpio_pin_num=5 produces the bit pattern
 *   (inverting the pattern above
 *
 *      1111 1111 1111 1100 0111 1111 1111 1111
 *                       ----
 *                       3 bits corresponding to FSEL5
 *   so the FSEL5 bits are set to 0
 * - for gpio_pin_num=5
 *       i2st_gpio_reg_set(ctx, n, gpfseln)
 *   sets the FSEL5 bits to 0 in the GPFSEL0 register
 *
 *
 *****************************************************************************/
static inline void i2st_gpio_pin_set_input(bcm2835_i2s_t* ctx, unsigned int gpio_pin_num)
{
    unsigned int n = gpio_pin_num/REG_GPFSEL_NUM_FSELN;
    unsigned int gpfseln = 0x00000000;

    gpfseln = i2st_gpio_reg_get(ctx, n);
    gpfseln &= ~(7 << ( (gpio_pin_num % REG_GPFSEL_NUM_FSELN) * REG_GPFSEL_FSELN_BITFIELD_SZ) );
    i2st_gpio_reg_set(ctx, n, gpfseln);
    return;
}

/*****************************************************************************
 * FUNCTION: i2st_gpio_pin_set_output
 ****************************************************************************
 * ARGS
 *  ctx
 *  gpio_pin_num     number of the gpio pin to configure as an output
 *
 * This function sets the GPIO configuration bits for GPIO port gpio_pin_num
 * corresponding to the FSEL(gpio_pin_num%10) bitfield in the
 * GPFSEL(gpio_pin_num/10) register to 0b001, thus configuring the GPIO
 * port to be an output.
 *
 * Note this function wont work when there has been a bit pattern other than
 * b000 previously programmed into the register because the funct uses bitwise
 * or |= with whatever is already there. See the comment later for
 * i2st_gpio_pin_set_alt_mode()
 *
 *****************************************************************************/
static inline void i2st_gpio_pin_set_output(bcm2835_i2s_t* ctx, unsigned int gpio_pin_num)
{
    unsigned int n = gpio_pin_num/REG_GPFSEL_NUM_FSELN;
    unsigned int gpfseln = 0x00000000;

    gpfseln = i2st_gpio_reg_get(ctx, n);
    gpfseln |= (1 << ( (gpio_pin_num % REG_GPFSEL_NUM_FSELN) * REG_GPFSEL_FSELN_BITFIELD_SZ) );
    i2st_gpio_reg_set(ctx, n, gpfseln);
    return;
}

/*****************************************************************************
 * FUNCTION: i2st_gpio_pin_set_alt_mode
 ****************************************************************************
 * ARGS
 *  ctx
 *  gpio_pin_num     number of the gpio pin to set the ALT mode
 *  alt_mode         the alt mode to set, = {0..5) corresponding to ALT0..ALT5
 *
 * This funct sets the GPIO configuration bits for GPIO port gpio_pin_num
 * corresponding to the FSEL(gpio_pin_num%10) bitfield in the
 * GPFSEL(gpio_pin_num/10) register to the bit pattern (see REF1 P92)
 *        100 = GPIO Pin g takes alternate function 0 i.e. ALT0
 *        101 = GPIO Pin g takes alternate function 1 i.e. ALT1
 *        110 = GPIO Pin g takes alternate function 2 i.e. ALT2
 *        111 = GPIO Pin g takes alternate function 3 i.e. ALT3
 *        011 = GPIO Pin g takes alternate function 4 i.e. ALT4
 *        010 = GPIO Pin g takes alternate function 5 i.e. ALT5
 *
 * Note the funct doesn't generate the following bit patterns for setting the
 * port to be an input or output
 *        000 = GPIO Pin gpio_pin_num is an input
 *        001 = GPIO Pin gpio_pin_num is an output
 *
 * so this is what happens in the part of the funct which determines the
 * bit pattern to program:
 *
 *     a          X=(((a)<=3?(a)+4:(a)==4?3:2)          ALT Mode    Correct?
 *     =          ============================          ========    ========
 *     0          a <= 3 yes => X=a+4 = 4 = b100 ==      ALT0        yes
 *     1          a <= 3 yes => X=a+4 = 5 = b101 ==      ALT1        yes
 *     2          a <= 3 yes => X=a+4 = 6 = b110 ==      ALT2        yes
 *     3          a <= 3 yes => X=a+4 = 7 = b111 ==      ALT3        yes
 *     4          a <= 3 no => a==4 ? yes => 3 = b011 == ALT4        yes
 *     5          a <= 3 no => a==4 ? no => 2 =  b010 == ALT5        yes
 *
 * Note on Using i2st_gpio_pin_set_output() or i2st_gpio_pin_set_alt_mode()
 *
 * Always use i2st_gpio_pin_set_input() before using
 *      i2st_gpio_pin_set_output() or
 *      i2st_gpio_pin_set_alt_mode()
 * As these 2 latter functions are implemented using the bitwise or |=, they
 * will only work properly if the FSELn field that is being set is b000.
 * Hence, i2st_gpio_pin_set_input() is always called first (setting the
 * appropriate FSELn bitfield to b000) before the other functions are
 * is called.
 *****************************************************************************/
static inline void i2st_gpio_pin_set_alt_mode(bcm2835_i2s_t* ctx, unsigned int gpio_pin_num, unsigned int alt_mode)
{
    unsigned int n = gpio_pin_num/REG_GPFSEL_NUM_FSELN;
    unsigned int gpfseln = 0x00000000;

    gpfseln = i2st_gpio_reg_get(ctx, n);
    gpfseln |= (alt_mode <= 3 ? alt_mode+4 :alt_mode == 4 ? 3 : 2) << ( (gpio_pin_num%10) * REG_GPFSEL_FSELN_BITFIELD_SZ);
    i2st_gpio_reg_set(ctx, n, gpfseln);
    return;
}

/*****************************************************************************
 * FUNCTION: desetup_io
 ****************************************************************************
 * unwind initialisation performed in setup_io() i.e. unmap the mapped memroy.
 *
 *****************************************************************************/
static void desetup_io(bcm2835_i2s_t* ctx)
{
    assert(ctx != 0);

    RPII2S_DBGLOG("%s: enter\n", __FUNCTION__);
    if(ctx->clk_base.mmap_addr != NULL)
    {
        /* Note munmap() uses the map address returned from the mmap() call.
         * The man page confusing calls the first arg to munmap(addr, len)
         * which is also supplied as an arg to mmap(addr, ...), but they
         * are different. */
        munmap((void*) ctx->clk_base.mmap_addr, BLOCK_SIZE);
        ctx->clk_base.mmap_addr = NULL;
    }
    if(ctx->i2s_base.mmap_addr != NULL)
    {
        munmap((void*) ctx->i2s_base.mmap_addr, BLOCK_SIZE);
        ctx->i2s_base.mmap_addr = NULL;
    }
    if(ctx->gpio_base.mmap_addr != NULL)
    {
        munmap((void*) ctx->gpio_base.mmap_addr, BLOCK_SIZE);
        ctx->gpio_base.mmap_addr = NULL;
    }
    if(ctx->clk_base.mem_na != NULL)
    {
        free(ctx->clk_base.mem_na);
        ctx->clk_base.mem_na = NULL;
        ctx->clk_base.mem = NULL;
    }
    if(ctx->i2s_base.mem_na != NULL)
    {
        free(ctx->i2s_base.mem_na);
        ctx->i2s_base.mem_na = NULL;
        ctx->i2s_base.mem = NULL;
    }
    if(ctx->gpio_base.mem_na != NULL)
    {
        free(ctx->gpio_base.mem_na);
        ctx->gpio_base.mem_na = NULL;
        ctx->gpio_base.mem = NULL;
    }
    /* we also have to close the file object */
    if(ctx->mem_fd > 0)
    {
        close(ctx->mem_fd);
        ctx->mem_fd = 0;
    }
    return;
}

/*****************************************************************************
 * FUNCTION: setup_io
 ****************************************************************************
 * Set up a memory regions to access GPIO using mmap
 *
 *****************************************************************************/
static void setup_io(bcm2835_i2s_t* ctx)
{
    assert(ctx != 0);

    /* see notes from 20141103-04 hand notes */
    RPII2S_DBGLOG("%s: enter\n", __FUNCTION__);

    /* /dev/mem is a special linux file allowing accesses to physical
     * memory addresses and so can be used to access soc registers */
    if ((ctx->mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0)
    {
        RPII2S_ERRLOG("can't open /dev/mem \n");
        goto error;
    }

    /* mmap GPIO: Allocate MAP block for GPIO configuration area
     * malloc() can potentially allocated anywhere in memory. This is
     * handled by rounding to a page boundary later. */
    if ((ctx->gpio_base.mem = malloc(BLOCK_SIZE + (PAGE_SIZE-1))) == NULL)
    {
        RPII2S_ERRLOG("allocation error \n");
        goto error;
    }
    /* store address for clean up later */
    ctx->gpio_base.mem_na = ctx->gpio_base.mem;

    /* mmap I2S: Allocate MAP block for I2S configuration area */
    if ((ctx->i2s_base.mem = malloc(BLOCK_SIZE + (PAGE_SIZE-1))) == NULL)
    {
        RPII2S_ERRLOG("allocation error \n");
        goto error;
    }
    /* store address for clean up later */
    ctx->i2s_base.mem_na = ctx->i2s_base.mem;

    /* mmap CLK: Allocate MAP block for CLK configuration area */
    if ((ctx->clk_base.mem = malloc(BLOCK_SIZE + (PAGE_SIZE-1))) == NULL)
    {
        RPII2S_ERRLOG("allocation error \n");
        goto error;
    }
    /* store address for clean up later */
    ctx->clk_base.mem_na = ctx->clk_base.mem;

    /* trace the addresses to see what has happened and the effect of malloc() */
    RPII2S_DBGLOG("addresses of interest after malloc() before rounding to page boundary\n");
    RPII2S_DBGLOG("gpio_base.mem=0x%p gpio_base.mmap_addr=0x%p\n", ctx->gpio_base.mem, ctx->gpio_base.mmap_addr);
    RPII2S_DBGLOG("i2s.mem=0x%p  i2s.mmap_addr=0x%p\n", ctx->i2s_base.mem, ctx->i2s_base.mmap_addr);
    RPII2S_DBGLOG("clk.mem=0x%p  clk.mmap_addr=0x%p\n", ctx->clk_base.mem, ctx->clk_base.mmap_addr);

    // Make sure pointer is on 4K boundary
    if ((unsigned long) ctx->gpio_base.mem % PAGE_SIZE)
    {
        ctx->gpio_base.mem += PAGE_SIZE - ((unsigned long) ctx->gpio_base.mem % PAGE_SIZE);
    }
    // Make sure pointer is on 4K boundary
    if ((unsigned long) ctx->i2s_base.mem % PAGE_SIZE)
    {
        ctx->i2s_base.mem += PAGE_SIZE - ((unsigned long) ctx->i2s_base.mem % PAGE_SIZE);
    }
    // Make sure pointer is on 4K boundary
    if ((unsigned long) ctx->clk_base.mem % PAGE_SIZE)
    {
        ctx->clk_base.mem += PAGE_SIZE - ((unsigned long)ctx->clk_base.mem % PAGE_SIZE);
    }
    /* trace the addresses to see what has happened after page boundary rounding */
    RPII2S_DBGLOG("addresses of interest after rounding to page boundary\n");
    RPII2S_DBGLOG("gpio_base.mem=0x%p gpio_base.mmap_addr=0x%p\n", ctx->gpio_base.mem, ctx->gpio_base.mmap_addr);
    RPII2S_DBGLOG("i2s.mem=0x%p  i2s.mmap_addr=0x%p\n", ctx->i2s_base.mem, ctx->i2s_base.mmap_addr);
    RPII2S_DBGLOG("clk.mem=0x%p  clk.mmap_addr=0x%p\n", ctx->clk_base.mem, ctx->clk_base.mmap_addr);

    /* map device */
    ctx->gpio_base.mmap_addr = (unsigned char *)mmap(
                               (caddr_t)ctx->gpio_base.mem,
                                BLOCK_SIZE,
                                PROT_READ|PROT_WRITE,
                                MAP_SHARED|MAP_FIXED,
                                ctx->mem_fd,
                                GPIO_BASE);

    ctx->i2s_base.mmap_addr = (unsigned char *)mmap(
                                 (caddr_t)ctx->i2s_base.mem,
                                  BLOCK_SIZE,
                                  PROT_READ|PROT_WRITE,
                                  MAP_SHARED|MAP_FIXED,
                                  ctx->mem_fd,
                                  I2S_BASE);

    ctx->clk_base.mmap_addr = (unsigned char *)mmap(
                                 (caddr_t)ctx->clk_base.mem,
                                  BLOCK_SIZE,
                                  PROT_READ|PROT_WRITE,
                                  MAP_SHARED|MAP_FIXED,
                                  ctx->mem_fd,
                                  CLOCK_BASE);

    if ((long)ctx->gpio_base.mmap_addr < 0)
    {
        RPII2S_ERRLOG("ctx->gpio_base.mmap_addr mmap error %d\n", (int)ctx->gpio_base.mmap_addr);
        goto error;
    }
    if ((long)ctx->i2s_base.mmap_addr < 0)
    {
        RPII2S_ERRLOG("ctx->i2s_base.mmap_addr mmap error %d\n", (int)ctx->i2s_base.mmap_addr);
        goto error;
    }
    if ((long)ctx->clk_base.mmap_addr < 0)
    {
        RPII2S_ERRLOG("ctx->clk_base.mmap_addr mmap error %d\n", (int)ctx->clk_base.mmap_addr);
        goto error;
    }

    /* trace the addresses to see what has happened and the effect of malloc() and MAP_FIXED flag */
    RPII2S_DBGLOG("addresses of interest after call to mmap()\n");
    RPII2S_DBGLOG("ctx->gpio_base.mem=0x%p ctx->gpio_base.mmap_addr=0x%p\n", ctx->gpio_base.mem, ctx->gpio_base.mmap_addr);
    RPII2S_DBGLOG("ctx->i2s_base.mem=0x%p  ctx->i2s_base.mmap_addr=0x%p\n", ctx->i2s_base.mem, ctx->i2s_base.mmap_addr);
    RPII2S_DBGLOG("ctx->clk_base.mem=0x%p  ctx->clk_base.mmap_addr=0x%p\n", ctx->clk_base.mem, ctx->clk_base.mmap_addr);
    return;
error:
    /* something went wrong with the setup */
    desetup_io(ctx);
    exit(-1);
}



#define PROG_HELP \
"\n"\
"  i2s_test version 1.01, Copyright (C) 2014 Simon Derek Hughes\n"\
"  i2s_test comes with ABSOLUTELY NO WARRANTY; for details type `i2s_test -h'.\n"\
"  This is free software, and you are welcome to redistribute it\n"\
"  under GPL V2.0 as described in the LICENSE file accompanying the source file\n"\
"  used to generate this binary.\n"\
"\n"\
"  Program to cm_pcm clk config().\n"\
"   First form on command line options (-s, -m, -i, -f all required):\n"\
"      i2s_test4 -s <SRC> -m <MASH> -i <DIVI> -f <DIVF>\n"\
"          -s <SRC>     clock src, 1=OSC(19.2MHz)(default), 4=PLLA(0Hz), \n"\
"                       5=PLLC(1GHz), 6=PLLD(500MHz), 7=HDMI Aux(216MHz)\n"\
"          -m <MASH>    number of mash stages {0..3}\n"\
"          -i <DIVI>    integer divider {0..4095}\n"\
"          -f <DIVF>    fractional divider {0..4095}\n"\
"\n"\
"   Second form on command line options (-t required):\n"\
"      i2s_test4 -t <TV> \n"\
"          -t <TV>      Test Vector value {0..2} \n"\
"                       0 => {src=OSC,f=256kHz}\n"\
"                       1 => {src=OSC,f=512kHz}\n"\
"                       2 => {src=OSC,f=1.536MHz}\n"\
"                       3 => {src=PLLC,f=256kHz}\n"\
"                       4 => {src=PLLC,f=512kHz}\n"\
"                       5 => {src=PLLC,f=1.536MHz}\n"\
"                       6 => {src=PLLD,f=256kHz}\n"\
"                       7 => {src=PLLD,f=512kHz}\n"\
"                       8 => {src=PLLD,f=1.536MHz}\n"\


static int cmdline_opt(int argc, char** argv)
{
    int ret = -1;
    int i = 0;
    int c = 0;
    unsigned int temp = 0;
    unsigned int cmd_opt_f = 0;

    RPII2S_DBGLOG("%s: enter\n", __FUNCTION__);

    /* see help message for description of options
     * note the option string to getopt must contain the options in the order they must appear on the cmdline*/
    while ( -1 != (c = getopt(argc,argv,"s:m:i:f:t:h")) )
    {
        switch (c)
        {
        case 'f': /* divf */
            temp = (int) strtoul(optarg, NULL, 0);
            if(temp < CM_PCMDIV_DIVF_MAX)
            {
                cm_pcmdiv_divf = temp;
                printf("fractional divisor DIVF = %d\n", cm_pcmdiv_divf);
                cmd_opt_f |= I2S_CMD_OPT_F_DIVF;
            }
            else
            {
                printf("fractional divisor DIVF = %d out of range {0..4095}\n", temp);
                ret = -1;
                goto error;
            }
            break;
        case 'h': /* help */
            printf("%s\n", (char*) PROG_HELP);
            return -1;
            break;
        case 'i': /* divi */
            temp = (int) strtoul(optarg, NULL, 0);
            if(temp < CM_PCMDIV_DIVF_MAX)
            {
                cm_pcmdiv_divi = temp;
                printf("integer divisor DIVI = %d\n", cm_pcmdiv_divi);
                cmd_opt_f |= I2S_CMD_OPT_F_DIVI;
            }
            else
            {
                printf("integer divisor DIVI = %d out of range {0..4095}\n", temp);
                ret = -1;
                goto error;
            }
            break;
        case 'm': /* mash */
            temp = (size_t) strtoul(optarg, NULL, 0);
            if(temp <= CM_PCMCTRL_MASH_MAX)
            {
                cm_pcmctrl_mash = temp;
                printf("number of mash stages = %d\n", cm_pcmctrl_mash);
                cmd_opt_f |= I2S_CMD_OPT_F_MASH;
            }
            else
            {
                printf("number of mash stages = %d unsupported\n", temp);
                ret = -1;
                goto error;
            }
            break;
        case 's': /* src */
            temp = (size_t) strtoul(optarg, NULL, 0);
            for(i=0; cm_pcmctrl_src_supported[i] != CM_PCMCTRL_SRC_MAX; i++)
            {
                if(temp == cm_pcmctrl_src_supported[i])
                {
                    cm_pcmctrl_src = temp;
                    cmd_opt_f |= I2S_CMD_OPT_F_SRC;
                    printf("clock source src = %d supported\n", cm_pcmctrl_src);
                    break;
                }
            }
            if(cm_pcmctrl_src_supported[i] == CM_PCMCTRL_SRC_MAX)
            {
                printf("clock source src = %d unsupported\n", temp);
                ret = -1;
                goto error;
            }
            break;
        case 't': /* test vector */
            temp = (int) strtoul(optarg, NULL, 0);
            if(temp < IS2_CMD_OPT_TEST_VECTOR_MAX)
            {

                cm_pcmctrl_src = test_vector_table[temp].src;
                cm_pcmctrl_mash = test_vector_table[temp].mash;
                cm_pcmdiv_divi = test_vector_table[temp].divi;
                cm_pcmdiv_divf = test_vector_table[temp].divf;
                printf("clock source src        = %d supported\n", cm_pcmctrl_src);
                printf("number of mash stages   = %d\n", cm_pcmctrl_mash);
                printf("integer divisor DIVI    = %d\n", cm_pcmdiv_divi);
                printf("fractional divisor DIVF = %d\n", cm_pcmdiv_divf);
                cmd_opt_f |= I2S_CMD_OPT_F_REQUIRED;
            }
            else
            {
                printf("test vector out of range {0..%d}\n", IS2_CMD_OPT_TEST_VECTOR_MAX-1);
                ret = -1;
                goto error;
            }
            break;
            break;
        default: /* unknown or missing argument */
            ret = -1;
            goto error;
        } /* switch */
    } /* while */

    if (optind < argc)
    {
        printf("error, unrecognised option (%s)", argv[optind]);
        ret = -1;
        goto error;
    }

    if(cmd_opt_f != I2S_CMD_OPT_F_REQUIRED)
    {
        printf("All the required options have not been supplied.\n");
        printf("%s\n", (char*) PROG_HELP);
        ret = -1;
        goto error;
    }

    /* check the validity of the args */
    unsigned int clk_src_freq_ref = cm_pcmctrl_src_freq_ref[cm_pcmctrl_src];
    mash_tentry_t* entry = &mash_table_6_32[cm_pcmctrl_mash];
    double max_freq = 0, ave_freq = 0, min_freq = 0;

    RPII2S_DBGLOG("cm_pcmctrl_src=%d, clk_src_freq_ref=%d\n", cm_pcmctrl_src, clk_src_freq_ref);
    ret = entry->calc_max_output_freq(clk_src_freq_ref, cm_pcmdiv_divi, &max_freq);
    if(ret < 0)
    {
        printf("unable to determine max frequency output\n");
        goto error;
    }
    ret = entry->calc_ave_output_freq(clk_src_freq_ref, cm_pcmdiv_divi, cm_pcmdiv_divf, &ave_freq);
    if(ret < 0)
    {
        printf("unable to determine average frequency output\n");
        goto error;
    }
    ret = entry->calc_min_output_freq(clk_src_freq_ref, cm_pcmdiv_divi, &min_freq);
    if(ret < 0)
    {
        printf("unable to determine min frequency output\n");
        goto error;
    }

    RPII2S_DBGLOG("output frequency min     = %15.6E\n", min_freq);
    RPII2S_DBGLOG("output frequency max     = %15.6E\n", max_freq);
    RPII2S_DBGLOG("output frequency average = %15.6E\n", ave_freq);

    /* check we're not using clock source that we haven't defined a frequency for */
    if(min_freq == 0.0 || max_freq == 0.0 || ave_freq == 0.0)
    {
        printf("Clock Source Definition Error, min, max or ave Frequency == 0\n");
        ret = -1;
        goto error;
    }

    /* check the max freq is less than board permissible max of 25MHz*/
    if(max_freq > RPI_MAX_FREQ_HZ)
    {
        printf("Input Error, Max Operating Frequency exceeded (%.3E > %.3E) \n", max_freq, (double)RPI_MAX_FREQ_HZ);
        ret = -1;
        goto error;
    }
    ret = 0;
error:
    return ret;
}

/*****************************************************************************
 * FUNCTION: i2st_check_pcm_cs_a_strobe_sync
 ****************************************************************************
 this is just setting the sync bit which takes 2 clocks cycles to clear
 * ARGS
 *  ctx     i2s device context
 *****************************************************************************/
static int i2st_check_pcm_cs_a_strobe_sync(bcm2835_i2s_t* ctx)
{
    unsigned int pcm_cs_a = 0x00000000;

    assert(ctx != NULL);

    pcm_cs_a = i2st_pcm_cs_a_get(ctx);
    //*(ctx->i2s+CS_A) &= ~(1<<24);
    i2st_pcm_cs_a_set(ctx, PCM_CS_A_F_SYNC);
    usleep(100);
    return 0;
}

/*****************************************************************************
 * FUNCTION: i2st_cm_pcm_clk_init
 ****************************************************************************
 * Initialise the CM_PCMCTRL and CM_PCMDIV registers for the PCM clock
 * configuration
 * ARGS
 *  ctx     i2s device context
 *****************************************************************************/
static int i2st_cm_pcm_clk_init(bcm2835_i2s_t* ctx)
{
    int g;
    int ret = 0;

    unsigned int cm_pcmctrl = 0x5A000000;       /* default setting, just contains password, used to turn clock off and reset */
    unsigned int cm_pcmdiv = 0x5A000000;        /* default setting, just contains password */
    unsigned int const cm_pcmctrl_enab = 0x1;   /* enable setting */

    RPII2S_DBGLOG("%s: enter\n", __FUNCTION__);
    assert(ctx != NULL);

    /* This code is not at all clear and the REF1 is incomplete so its necessary
     * to use REF2 (errata for clocks) to understand whats going on here.
     *
     * GPIO general purpose clocks have a configuration base starting at
     *  CLOCK_BASE = 0x20101000
     * This is the value of clk. Although not documented main REF1, there is
     * an errata document for clocks (see REF2) that documents the PCM clock
     * (aka I2S clock) which is configurable using 2 registers:
     *
     *  Name        VideoCore           ARM Physical
     *              Peripheral Bus      Address
     *              Address
     *  ============================================
     *  CM_PCMCTRL  0x73101098          0x20101098
     *  CM_PCMDIV   0x73101098          0x2010109c
     *
     * 0x26 x sizeof(unsigned int) = 0x98
     * hence *(clk+0x26) references 0x20101098
     */
    RPII2S_DBGLOG("Disabling I2S clock\n");
    i2st_cm_pcmctrl_set(ctx, cm_pcmctrl);
    ret = i2st_cm_pcmctrl_wait_not_busy(ctx);
    if(ret < 0)
    {
        RPII2S_ERRLOG("error: gave up waiting for busy flag to clear\n");
        goto error;
    }

    i2st_cm_pcmdiv_set(ctx, cm_pcmdiv);
    cm_pcmctrl |= cm_pcmctrl_mash << CM_PCMCTRL_MASH_LSB_OFFSET | cm_pcmctrl_src << CM_PCMCTRL_SRC_LSB_OFFSET;
    cm_pcmdiv |= cm_pcmdiv_divi << CM_PCMDIV_DIVI_LSB_OFFSET | cm_pcmdiv_divf << CM_PCMDIV_DIVF_LSB_OFFSET;

    /* set up the cm_pcm registers without enabling the clock */
    RPII2S_DBGLOG("Configure I2S clock\n");
    i2st_cm_pcmctrl_set(ctx, cm_pcmctrl);
    i2st_cm_pcmdiv_set(ctx, cm_pcmdiv);

    /* let change take effect */
    usleep(10);

    /* now enable the clock*/
    RPII2S_DBGLOG("Enabling I2S clock\n");
    cm_pcmctrl |= cm_pcmctrl_mash << CM_PCMCTRL_MASH_LSB_OFFSET | cm_pcmctrl_src << CM_PCMCTRL_SRC_LSB_OFFSET | cm_pcmctrl_enab << CM_PCMCTRL_ENAB_LSB_OFFSET;
    i2st_cm_pcmctrl_set(ctx, cm_pcmctrl);
    ret = i2st_cm_pcmctrl_wait_busy(ctx);
    if(ret < 0)
    {
        RPII2S_ERRLOG("error: gave up waiting for busy flag to set\n");
        goto error;
    }

    ret = 0;
error:
    return ret;
}

/*****************************************************************************
 * FUNCTION: i2st_check_pcm_cs_sync_bit
 ****************************************************************************
 * check that the PCM_CS sync bit behaves as expected
 * ARGS
 *  ctx     i2s device context
 *****************************************************************************/
static int i2st_check_pcm_cs_sync_bit(bcm2835_i2s_t* ctx)
{
    unsigned int pcm_cs_a = 0x00000000;

    assert(ctx != NULL);
    pcm_cs_a = i2st_pcm_cs_a_get(ctx);

    RPII2S_DBGLOG("setting sync bit high\n");
    pcm_cs_a |= PCM_CS_A_F_SYNC;
    i2st_pcm_cs_a_set(ctx, pcm_cs_a);
    if(i2st_pcm_cs_a_get(ctx) & PCM_CS_A_F_SYNC)
    {
        RPII2S_DBGLOG("SYNC bit high, strange.\n");
    }
    else
    {
        RPII2S_DBGLOG("SYNC bit low, as expected.\n");
    }

    usleep(1);

    if(i2st_pcm_cs_a_get(ctx) & PCM_CS_A_F_SYNC)
    {
        RPII2S_DBGLOG("SYNC bit high, as expected.\n");
    }
    else
    {
        RPII2S_DBGLOG("SYNC bit low, strange.\n");
    }
    return 0;
}

/*****************************************************************************
 * FUNCTION: i2st_check_pcm_dump_registers
 ****************************************************************************
 * output current value of pcm_xx registers
 * ARGS
 *  ctx     i2s device context
 *****************************************************************************/
static void i2st_check_pcm_dump_registers(bcm2835_i2s_t* ctx)
{
    int i = 0;

    assert(ctx != NULL);
    RPII2S_DBGLOG("Dump I2S Registers\n");
    for(i = 0; i < 9; i++)
    {
        RPII2S_DBGLOG("PCM/I2S Register Num=%d: 0x%08x    %s\n", i, i2st_pcm_reg_get(ctx, i), i2s_register_name[i]);
    }
    return;
}

/*****************************************************************************
 * FUNCTION: i2st_check_gpio_dump_registers
 ****************************************************************************
 * output current value of gpio_xx registers
 * ARGS
 *  ctx     i2s device context
 *****************************************************************************/
static void i2st_check_gpio_dump_registers(bcm2835_i2s_t* ctx)
{
    int i = 0;

    assert(ctx != NULL);
    RPII2S_DBGLOG("Dump GPIO Registers\n");
    for(i = 0; i < 10; i++)
    {
        RPII2S_DBGLOG("GPIO Reg Number=%d: 0x%08x\n", i, i2st_gpio_reg_get(ctx, i));
    }
    return;
}

/*****************************************************************************
 * FUNCTION: i2st_check_pcm_i2s_send_forever
 ****************************************************************************
 * output test bit pattern on i2s_dout forever
 * ARGS
 *  ctx     i2s device context
 *****************************************************************************/
static int i2st_check_pcm_i2s_send_forever(bcm2835_i2s_t* ctx)
{
    int count = 0;
    unsigned int i2s_dout_data = 0xA0A0A0A0;    /* I2S_DOUT bit pattern */

    printf("Sending data\n");
    printf("\n");
    for(;;)
    {
        /* if the tx fifo is full then wait for some space to become available */
        while (! (i2st_pcm_cs_a_get(&bcm2835_i2s) & PCM_CS_A_F_TXD) )
        {
            usleep(1);
        }
        count++;
        i2st_pcm_fifo_a_set(&bcm2835_i2s, i2s_dout_data);
        if(++count % 10000 == 0)
        {
            printf("Tx FIFO, count=%i data %x\n", count, i2s_dout_data);
        }
    }
    return 0;
}

/*****************************************************************************
 * FUNCTION: i2st_cm_pcm_i2s_init
 ****************************************************************************
 * Initialise the i2s config
 * ARGS
 *  ctx     i2s device context
 *****************************************************************************/
static int i2st_cm_pcm_i2s_init(bcm2835_i2s_t* ctx)
{
    int i;
    unsigned int pcm_cs_a = 0x00000000;
    unsigned int pcm_txc_a = 0x00000000;
    unsigned int pcm_mode_a = 0x00000000;

    assert(ctx != NULL);
    /* disable I2S so we can modify the regs */
    RPII2S_DBGLOG("Disabling I2S Bus\n");
    i2st_pcm_cs_a_set(ctx, pcm_cs_a);
    usleep(100);

    /* 1<<3 => TXCLR i.e. clear the tx fifo. takes 2 PCM_CLK to take effect
     * 1<<4 => RXCLR i.e. clear the rx fifo. takes 2 PCM_CLK to take effect
     * 11 << 5 => 11 decimal == b1011 =>
     *  RXTHR = 0b10 (RX fifo threshold for setting RXR flag)
     *          0b10 => RXR flag will be set when rx fifo is less than full
     *  TXTHR = 0b11 (TX fifo threshold for setting TXW flag)
     *          0b11 => TXW flag will be set when tx fifo full except for 1 sample
     */
    RPII2S_DBGLOG("Clearing Rx & Tx FIFOs\n");
    pcm_cs_a |= PCM_CS_A_F_TXCLR | PCM_CS_A_F_RXCLR | PCM_CS_A_TXTHR | PCM_CS_A_RXTHR;
    i2st_pcm_cs_a_set(ctx, pcm_cs_a);
    usleep(10);

    /* ch1 (assumed R channel, check) 32 clocks i.e. bits long carrying 16 bits of data (=> TXC_A_CH1WID = 0x8)
     * ch2 (assume L channel) 32 clocks i.e. bits long carrying 16 bits of data (=> TXC_A_CH2WID = 0x8)
     * frame (LRCLK length) is therefore 64 clocks in length (=> MODE_A_FLEN=64)
     * LRCLK negedge/posedge each after 32 clocks (=> MODE_A_FSLEN=32)
     * TXC_A_CH1POS set to 1 so that the 2nd neg edge is the first clock edge for data in the R frame
     * TXC_A_CH2POS set to 33 so that the 33rd neg edge is the first clock edge for data in the L frame
     * TX fifo takes R and L channel 16bit data in 1 32 word (=> MODE_A_FTXP=1) */

    RPII2S_DBGLOG("Setting TXC_A R/L channel settings\n");
    /* ch1 config */
    pcm_txc_a |= PCM_TXC_A_F_CH1WEX_RESET;      /* CH1WEX = 0b0 => not using field extension */
    pcm_txc_a |= PCM_TXC_A_F_CH1EN;             /* CH1EN = 0b1 => enable channel 1 in the frame */
    pcm_txc_a |= PCM_TXC_A_CH1POS;              /* CH1POS = 1 => ch1 data on 2nd clock of frame */
    pcm_txc_a |= PCM_TXC_A_CH1WID;              /* CH1WID = 0x8 = 0b1000 => ch1 data 16 bits wide */

    /* ch2 config */
    pcm_txc_a |= PCM_TXC_A_F_CH2WEX_RESET;      /* CH2WEX = 0b0 => not using field extension */
    pcm_txc_a |= PCM_TXC_A_F_CH2EN;             /* CH2EN = 0b1 => enable channel 2 in the frame */
    pcm_txc_a |= PCM_TXC_A_CH2POS;              /* CH2POS = 33 => ch2 data on 2nd clock of frame */
    pcm_txc_a |= PCM_TXC_A_CH2WID;              /* CH2WID = 0x8 = 0b1000 => ch2 data 16 bits wide */

    i2st_pcm_txc_a_set(ctx, pcm_txc_a);

    RPII2S_DBGLOG("Setting MODE_A Fifo, Frame Len and Frame Sync (LRCLK) Len settings\n");
    pcm_mode_a |= PCM_MODE_A_F_FTXP;            /* tx fifo split into 2 16 bit words */
    pcm_mode_a |= PCM_MODE_A_FLEN;              /* 64 clocks in a frame */
    pcm_mode_a |= PCM_MODE_A_FSLEN;             /* L/R i.e ch1/ch2 both have 32 clocks */

    i2st_pcm_mode_a_set(ctx, pcm_mode_a);

    /* must wait for 4 pcm clocks after releasing from standby */
    RPII2S_DBGLOG("Release from PCM RAMs from standby prior to Rx/Tx operations\n");
    pcm_cs_a |= PCM_CS_A_F_STBY;
    i2st_pcm_cs_a_set(ctx, pcm_cs_a);

    /* todo: make this sleep time clock speed dependent */
    usleep(50);     // is this 4 pcm clocks?

    /* enable PCM/I2S tx/rx operations */
    pcm_cs_a |= PCM_CS_A_F_EN;
    i2st_pcm_cs_a_set(ctx, pcm_cs_a);

    /* enable transmission */
    pcm_cs_a |= PCM_CS_A_F_TXON;
    i2st_pcm_cs_a_set(ctx, pcm_cs_a);

    /* enable transmission */
    /* pcm_cs_a |= PCM_CS_A_F_RXON;
     * i2st_pcm_cs_a_set(ctx, pcm_cs_a);
     */

    i2st_check_pcm_cs_sync_bit(ctx);
    if(rpi_optDebug)
    {
        i2st_check_pcm_dump_registers(ctx);
    }
    return 0;
}

/*****************************************************************************
 * FUNCTION: main
 ****************************************************************************
 * main entry point to application
 * see PROG_HELP for documentation of the command line arguments and format
 *****************************************************************************/
int main(int argc, char **argv)
{
    int i;
    int gpio_port_num = 0;
    int ret = -1;

    bcm2835_i2s_t* ctx =&bcm2835_i2s;

    RPII2S_DBGLOG("%s: enter\n", __FUNCTION__);
    memset(&bcm2835_i2s, 0, sizeof(bcm2835_i2s));

    ret = cmdline_opt(argc, argv);
    if(ret < 0)
    {
        RPII2S_ERRLOG("Error processing command line\n");
        goto out;
    }

    setup_io(&bcm2835_i2s);

    /* Set the GPIO18-21 on P1 header to I2S mode (ALT0)
     * (I think there is some problem here which I forget).
     *
     * REF1 P101 Sec 6.2 Alternative Function Assignments
     * The following can be seen for ALT0 (alternative function mode 0)

     *  GPIO Pin Num    ALT0 Function   I2S Equivalent
     *  ============    =============   ==============
     *  GPIO18          PCM_CLK         I2S_BCLK
     *  GPIO19          PCM_FS          I2S_LRCLK
     *  GPIO20          PCM_DIN         I2S_DIN
     *  GPIO21          PCM_DOUT        I2S_DOUT
     *
     * On the RPI Rev 2.0 board the above pins are on the P5 header next to the P1 header
     * On the RPI Rev 2.1 board the above pins are on the P6 header next to the P1 header
     */
    for (gpio_port_num = GPI018_ALT0_PCM_CLK; gpio_port_num <= GPI021_ALT0_PCM_DOUT; gpio_port_num++)
    {
        /* set the GPIO pin config to b000 i.e. to input
         * note there is no explicit config of the PCM_CLK,
         * PCM_FS, PCM_DOUT pins to output. This must be
         * implicit by setting the alt mode for the pin */
        i2st_gpio_pin_set_input(&bcm2835_i2s, gpio_port_num);
        i2st_gpio_pin_set_alt_mode(&bcm2835_i2s, gpio_port_num, 0);
    }

    /* Set the GPIO28-31 on P5(Rev2.0)/P6(Rev2.1) header to I2S mode (ALT2)
     *
     * REF1 P101 Sec 6.2 Alternative Function Assignments
     * The following can be seen for ALT2 (alternative function mode 2)

     *  GPIO Pin Num    ALT2 Function   I2S Equivalent
     *  ============    =============   ==============
     *  GPIO28          PCM_CLK         I2S_BCLK
     *  GPIO29          PCM_FS          I2S_LRCLK
     *  GPIO30          PCM_DIN         I2S_DIN
     *  GPIO31          PCM_DOUT        I2S_DOUT
     *
     * On the RPI Rev 2.0 board the above pins are on the P5 header next to the P1 header
     * On the RPI Rev 2.1 board the above pins are on the P6 header next to the P1 header
     */
    for (gpio_port_num = GPI028_ALT2_PCM_CLK; gpio_port_num <= GPI031_ALT2_PCM_DOUT; gpio_port_num++)
    {
        i2st_gpio_pin_set_input(&bcm2835_i2s, gpio_port_num);        /* required to set FSELg to b000 before using i2st_gpio_pin_set_alt_mode() */
        i2st_gpio_pin_set_alt_mode(&bcm2835_i2s, gpio_port_num, 2);  /* set gpio pin functionality to ALT2 i.e. i2s mode */
    }

    if(rpi_optDebug)
    {
        i2st_check_gpio_dump_registers(&bcm2835_i2s);
    }

    ret = i2st_cm_pcm_clk_init(&bcm2835_i2s);
    if(ret < 0)
    {
        RPII2S_ERRLOG("error: failed to initialise pcm clk\n");
        goto out;
    }

    ret = i2st_cm_pcm_i2s_init(&bcm2835_i2s);
    if(ret < 0)
    {
        RPII2S_ERRLOG("error: failed to initialise i2s bus\n");
        goto out;
    }

    ret = i2st_check_pcm_i2s_send_forever(&bcm2835_i2s);
out:
    desetup_io(&bcm2835_i2s);
    return ret;
} /* main() */


