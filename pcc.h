#ifndef NTP_PCC_HEADER_H
#define NTP_PCC_HEADER_H

#include <stdint.h>

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

/* there might be mild dependencies on sizeof(pcc_t) being >= sizeof(long) */
#if defined(__i386__) && defined(USE_RDTSC)

#define PCC_WIDTH 64
typedef uint64_t pcc_t;

#else

#define PCC_WIDTH 32
typedef unsigned long pcc_t;

#endif

/* There are two methods for implementing nanosecond
 * clock resolution:
 *
 * A) the timer responsible for generating system clock
 *    interrupts itself is readable with a high resolution and
 *    its value at the time of interrupt generation is known.
 *    This is the preferred method.
 *
 * B) system clock ISR reads a PCC ("processor cycle counter")
 *    which is possibly asynchronous to the system clock.
 *    At each interrupt, the ISR saves the PCC to a global variable.
 *    It is essential to minimize jitter of this reading with
 *    respect to interrupt time. Therefore, reading PCC should
 *    be performed ASAP in the clock ISR.
 *
 * Then there is method C) which means that there is no
 * high resolution clock support at all. The kernel clock
 * just has a 'tick'-level resolution but is nevertheless
 * synchronized to NTP.
 *
 */


#ifndef USE_NO_HIGH_RESOLUTION_CLOCK

#ifdef __PPC__

#ifdef USE_METHOD_B_FOR_DEMO

/* we use the free running timebase register as a PCC
 * (which is actually also locked to the decrementer but
 * this is for demo purposes only)
 */
static volatile pcc_t pcc_base;
static volatile pcc_t lastIrqTime;

static inline pcc_t getPcc()
{
unsigned pcc;
	asm volatile("mftb %0":"=r"(pcc));
	return pcc - pcc_base;
}

/* The clock driver should call this directly from the ISR
 * for demo purposes we use a timer, however.
 */
static inline void rtems_ntp_isr_snippet()
{
	asm volatile("mftb %0":"=r"(lastIrqTime));
}

static inline pcc_t setPccBase()
{
pcc_t oldbase = pcc_base;
	pcc_base = lastIrqTime;
	return pcc_base - oldbase;
}

#elif !defined(USE_PICTIMER) /* ifdef method_B */

/* PowerPC 'Method A' implementation -- use for production */

extern unsigned Clock_Decrementer_value;

#define HRC_PERIOD Clock_Decrementer_value
static inline unsigned PPC_HRC_READ()
{
unsigned	val;
	PPC_Get_decrementer(val);
	return val;
}
#define HRC_READ()	PPC_HRC_READ()

#else /* if method_B elif !pictimer */
/* pictimer.c provides an implementation for method A */
extern pcc_t getPcc();
extern pcc_t setPccBase();
#endif

#elif defined(__mcf528x__)

#include <bsp.h>
#include <mcf5282/mcf5282.h>

#warning "High-resolution clock implementation for the uC5282 BSP only - but I can't check for BSP in header"

static inline unsigned UC5282_HRC_READ()
{
unsigned rval = MCF5282_PIT3_PCNTR;
	return rval;
}

#define HRC_READ() UC5282_HRC_READ()

#ifdef DECL_SRAM_PITC_PER_TICK /* in config.h */
#define HRC_PERIOD  (__SRAMBASE.pitc_per_tick)
#else
#if RTEMS_VERSION_AT_LEAST(4,8,99)
#define HRC_PERIOD	rtems_configuration_get_microseconds_per_tick()
#else
#define HRC_PERIOD	BSP_Configuration.microseconds_per_tick
#endif
#endif

#elif /* ifdef __PPC__ */ defined(__i386__) && defined(USE_RDTSC)

static pcc_t last_tick = 0;

static inline pcc_t rdtsc()
{
uint32_t hi, lo;
	__asm__ __volatile__ ("rdtsc" : "=a" (lo), "=d" (hi));
	return ((((pcc_t)hi)<<32) | lo);
}

static pcc_t getPcc()
{
	return rdtsc() - last_tick;
}

static inline pcc_t setPccBase()
{
pcc_t old = last_tick;
	last_tick = rdtsc();
	return old ? (last_tick - old) : 0;
}

#else /* ifdef __PPC__ */

#warning No High Resolution Clock Implementation for this CPU, please add to pcc.h (DISABLED)
#define  USE_NO_HIGH_RESOLUTION_CLOCK

#endif

#if defined(HRC_READ)
/* this routine returns the difference between the current PCC
 * and the PCC base time (==IRQ time).
 * In Method A, we know that the base time is actually always 0.
 * However, we allow the clock adjustment task to run at a subharmonic
 * of the system clock. Therefore, we must account for the number
 * of ticks expired since the (subharmonic) time was updated.
 *
 * --> subharmonic task calls setPccBase() and remembers the current
 *     system clock tick as the 'base'
 * --> getPcc reads the PCC and adds (ticks-base) * clock_period
 */

static uint32_t tick_base;

#include <rtems/clockdrv.h>

static inline pcc_t getPcc()
{
unsigned        flags;
pcc_t           pcc;
unsigned		rtemsTicks;

	/* reading the PCC (the decrementer register) and
	 * the current system tick counter must be
	 * atomical...
	 */
	rtems_interrupt_disable( flags );
	pcc = HRC_READ();
	rtemsTicks   = Clock_driver_ticks;
	rtems_interrupt_enable( flags );

	/* even correct if the decrementer has underflown */
	pcc = HRC_PERIOD - pcc;

	/* account for the number of ticks expired since setPccBase()
	 * was called for the last time
	 */
	rtemsTicks = rtemsTicks - tick_base;

	return pcc + HRC_PERIOD * rtemsTicks;
}

static inline pcc_t setPccBase()
{
unsigned oldbase = tick_base;
	tick_base = Clock_driver_ticks;
	return HRC_PERIOD * (tick_base - oldbase);
}
#endif


#endif /* ifdef USE_NO_HIGH_RESOLUTION_CLOCK */

/* Test again; USE_NO_xxx could be defined since the last test... */
#ifdef USE_NO_HIGH_RESOLUTION_CLOCK

static inline pcc_t getPcc() 		{ return 0; }
static inline pcc_t setPccBase() 	{ return 0; }

#endif


#endif
