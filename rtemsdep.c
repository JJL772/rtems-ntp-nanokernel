/* $Id$ */

/* RTEMS support for Dave Mills' ktime and a very simple NTP synchronization daemon */

/* Author: Till Straumann <strauman@slac.stanford.edu>, 2004 */

#ifndef _USED_FROM_SIMULATOR_

#include <rtems.h>
#include <bsp.h>
/*
#include <bsp/bspExt.h>
#include <bsp/irq.h>
*/
#include <rtems/rtems_bsdnet.h>
#include <rtems/bspIo.h>
#include <netinet/in.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <assert.h>
#include <math.h>

#include <sys/socket.h>

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#ifdef HAVE_CEXP
#include <cexp.h>
#endif

#include "kern.h"
#include "rtemsdep.h"
#include "timex.h"
#include "pcc.h"

#ifdef USE_PICTIMER
#include "pictimer.h"
#endif

#endif


/* =========== CONFIG PARAMETERS ===================== */
#undef USE_PROFILER

#define NTP_DEBUG (0)
/* define to OR or the following: */
#define		NTP_DEBUG_PACKSTATS     1   /* gather NTP packet timing info */
#define		NTP_DEBUG_MISC          2   /* misc utils (beware of symbol clashes) */
#define     NTP_DEBUG_FILTER		4   /* print info about trivial filtering algorithm */

#define DAEMON_SYNC_INTERVAL_SECS	64	/* default sync interval */

#define KILL_DAEMON					RTEMS_EVENT_1

#ifdef USE_PICTIMER
#if KILL_DAEMON == PICTIMER_SYNC_EVENT
#error PICTIMER_SYNC_EVENT collides with KILL_DAEMON event
#endif
#endif

#define PPM_SCALE					(1<<16)
#define PPM_SCALED					((double)PPM_SCALE)

/* If no server can be contacted after 'MAX_FAILED_SYNCS' poll intervals
 * then the clock goes into STA_UNSYNC (unsynchronized status).
 */
#define MAX_FAILED_SYNCS			10


/* =========== PUBLIC GLOBALS ======================== */
volatile unsigned      rtems_ntp_debug = 0;
FILE		  		   *rtems_ntp_debug_file = 0;

/* =========== GLOBAL VARIABLES ====================== */

#ifdef NTP_NANO
struct timespec TIMEVAR = {0, 0};	/* kernel nanosecond clock */
#else
struct timeval TIMEVAR  = {0, 0};	/* kernel microsecond clock */
#endif

int microset_flag[NCPUS] = {0,};	/* microset() initialization filag */
int hz                   = 0;

#ifndef _USED_FROM_SIMULATOR_
       rtems_id rtems_ntp_ticker_id = 0;
       rtems_id rtems_ntp_daemon_id = 0;
static rtems_id kill_sem;
static rtems_id mutex_id  = 0;
#ifndef USE_PICTIMER
static volatile int tickerRunning = 0;
#endif
#ifdef USE_METHOD_B_FOR_DEMO
static rtems_id sysclk_irq_id = 0;
#endif

int    rtems_ntp_daemon_sd          = 0;
static int our_sd                   = 0;

/* Mutex Primitives (compat with ktime.c / micro.c) */

int
splclock()
{
	rtems_semaphore_obtain( mutex_id, RTEMS_WAIT, RTEMS_NO_TIMEOUT );
	return 1;
}

int
splx(int level)
{
	if ( level )	/* this test exists, so splclock() can be e.g., #define 0 */
		rtems_semaphore_release( mutex_id );
	return 0;
}

/*
 * RTEMS base: 1988, January 1
 *  UNIX base: 1970, January 1
 *   NTP base: 1900, January 1
 */
#define UNIX_BASE_TO_NTP_BASE (((70UL*365UL)+17UL) * (24*60*60))

#define PARANOIA(something)	assert( RTEMS_SUCCESSFUL == (something) )

#endif

#if NTP_DEBUG & NTP_DEBUG_PACKSTATS
long long rtems_ntp_max_t1;
long long rtems_ntp_max_t2;
long long rtems_ntp_max_t3;
long long rtems_ntp_max_diff   = 0;
#ifdef __PPC__
long      rtems_ntp_max_tbdiff = 0;
#endif

#if 0
static inline long long llabs(long long n)
{
	return n < 0 ? -n : n;
}
#endif
#endif

static unsigned long pcc_numerator;
static unsigned long pcc_denominator = 0;
#ifdef NTP_NANO
static struct timespec	nanobase;
#else
static struct timeval	nanobase;
#endif

/* Convert poll seconds to PLL time constant. According to the
 * documentation the polling interval tracks the time-constant 
 * and a constant of 0 corresponds to a 16s polling interval.
 */
static unsigned
secs2tcld(int secs)
{
unsigned rval,probe;
	secs >>= 4;	/* divide by 16; const == 2^0  ~ poll_interval == 16s */
	/* find closest power of two */
	for ( rval = 0, probe=1; probe < secs; rval++ )
		probe <<= 1;

	if ( 0 == rval )
		return 0;

	return ( probe - secs < secs - (probe>>1) ) ? rval : rval - 1;
}

unsigned long long lasttime = 0;

long
nano_time(struct timespec *tp)
{
unsigned long long pccl, thistime;

	pccl = getPcc();

#ifdef NTP_NANO
	*tp = nanobase;
#else
	tp->tv_sec   = nanobase.tv_sec;
	tp->tv_nsec  = nanobase.tv_usec * 1000;
#endif

	/* convert to nanoseconds */
	if ( pcc_denominator ) {
		pccl *= pcc_numerator;
		pccl /= pcc_denominator;
		
		thistime  = (unsigned long)tp->tv_sec;
		thistime  = thistime * NANOSECOND + tp->tv_nsec + pccl;

		/* prevent the clock from running backwards
		 * (small backjumps may appear if a clock tick
		 * adjustment is smaller than what the last nanoclock
		 * prediction was...)
		 */
		if ( thistime <= lasttime )
			thistime = lasttime + 1;
		lasttime = thistime;

		tp->tv_sec  = thistime / NANOSECOND;
		tp->tv_nsec = thistime % NANOSECOND;
	}

	return (long)pccl;
}

unsigned long tsillticks=0;

static inline void
ticker_body()
{
int s;
unsigned flags;

	s = splclock();

	ntp_tick_adjust(&TIMEVAR, 0);
	second_overflow(&TIMEVAR);

	rtems_interrupt_disable(flags);
tsillticks++;

	pcc_denominator = setPccBase();
	pcc_numerator   = 
#ifdef NTP_NANO
		TIMEVAR.tv_nsec - nanobase.tv_nsec 
#else
		(TIMEVAR.tv_usec - nanobase.tv_usec) * 1000
#endif
		+ (TIMEVAR.tv_sec - nanobase.tv_sec) * NANOSECOND
		;
	nanobase = TIMEVAR;
	rtems_interrupt_enable(flags);

	splx(s);
}


#ifndef _USED_FROM_SIMULATOR_

#ifdef USE_METHOD_B_FOR_DEMO
static rtems_timer_service_routine sysclkIrqHook( rtems_id me, void *uarg )
{
	rtems_ntp_isr_snippet();
	rtems_timer_fire_after( me, 1, sysclkIrqHook, uarg );
}
#endif


/* could use ntp_gettime() for this - avoid the overhead */
static inline void locked_nano_time(struct timespec *pt)
{
int s;
	s = splclock();
	nano_time(pt);
	splx(s);
}

static inline void locked_hardupdate(long nsecs)
{
int s;
	s = splclock();
	hardupdate(&TIMEVAR, nsecs );
	splx(s);
}

static inline long long nts2ll(struct timestamp *pt)
{
	return (((long long)ntohl(pt->integer))<<32) + (unsigned long)ntohl(pt->fraction);
}

static inline long long nsec2frac(unsigned long nsec)
{
	return (((long long)nsec)<<32)/NANOSECOND;
}

/* assume the result can represent 'f' in nanoseconds ! */
static inline long frac2nsec(long long f)
{
	return (long) ((((long long)NANOSECOND) * f) >> 32);
}

/* convert integral part to seconds; NOTE: fractional part
 * still may be bigger than 1s
 */
static inline long int2sec(long long f)
{
long rval = f>>32;
	return rval < 0 ? rval + 1 : rval;
}

#define LEAP_MSK	0xc0
#define LEAP_INS	0x40
#define LEAP_DEL	0x80
#define LEAP_NO     0x00

typedef struct DiffTimeCbData_ {
	long long			diff;
	unsigned long		tripns;
	unsigned char   	li_vn_mode;
	uint32_t 			srv_rcvts;
	uint32_t            rootdelay; /* this is a single-precision 'fixed-point' number */
	uint32_t            rootdisp;  /* this is a single-precision 'fixed-point' number */
} DiffTimeCbDataRec, *DiffTimeCbData;

static int diffTimeCb(struct ntpPacketSmall *p, int state, void *usr_data)
{
DiffTimeCbData	udat = usr_data;
struct timespec nowts;
long long       now, diff, org, rcv = 0;
#if (NTP_DEBUG & NTP_DEBUG_PACKSTATS) && defined(__PPC__)
static long		tbthen;
long			tbnow;
#endif
	
	if ( state >= 0 ) {
		locked_nano_time(&nowts);
		now = nsec2frac(nowts.tv_nsec);
		/* convert RTEMS to NTP seconds */
		nowts.tv_sec += rtems_bsdnet_timeoffset + UNIX_BASE_TO_NTP_BASE;
		if ( 1 == state ) {
			/* first pass; record our current time */
			p->transmit_timestamp.integer  = htonl( nowts.tv_sec );
			p->transmit_timestamp.fraction = htonl( (unsigned long)now ); 
#if (NTP_DEBUG & NTP_DEBUG_PACKSTATS) && defined(__PPC__)
			asm volatile("mftb %0":"=r"(tbthen));
#endif
		} else {
			now  += ((long long)nowts.tv_sec)<<32;
			diff  = nts2ll( &p->transmit_timestamp ) - now;
			if ( ( org  = nts2ll( &p->originate_timestamp ) ) && 
			     ( rcv  = nts2ll( &p->receive_timestamp   ) ) ) {
				/* correct for delays */
				diff += (rcv - org);
				diff >>=1;
				udat->tripns = frac2nsec(now-org);
			} else {
				udat->tripns = 0;
			}
			udat->diff       = diff;
			udat->li_vn_mode = p->li_vn_mode;
			udat->srv_rcvts  = ntohl(p->receive_timestamp.integer);
			udat->rootdelay  = ntohl(p->root_delay);
			udat->rootdisp   = ntohl(p->root_dispersion);
#if (NTP_DEBUG & NTP_DEBUG_PACKSTATS)
			if ( llabs(diff) > llabs(rtems_ntp_max_diff) ) {
#ifdef __PPC__
				asm volatile("mftb %0":"=r"(tbnow));
#endif
				rtems_ntp_max_t1 = org;
				rtems_ntp_max_t2 = org ? rcv : 0;
				rtems_ntp_max_t3 = nts2ll( &p->transmit_timestamp );
				rtems_ntp_max_diff = diff;
#ifdef __PPC__
				rtems_ntp_max_tbdiff = tbnow-tbthen;
#endif
			}
#endif
		}
	}
	return 0;
}

#if (NTP_DEBUG & NTP_DEBUG_PACKSTATS)
static inline void ufrac2ts(unsigned long long f, struct timespec *pts)
{
	pts->tv_sec = f>>32;
	f = ((f & ((1ULL<<32)-1)) * NANOSECOND ) >> 32;
	pts->tv_sec += f/NANOSECOND;
	pts->tv_nsec = f % NANOSECOND;
}

long
rtemsNtpPrintMaxdiff(int reset)
{
time_t    t;
long long t4;
struct timespec ts;

	printf("Max adjust %llins", (rtems_ntp_max_diff * NANOSECOND)>>32);
#ifdef __PPC__
	printf(" TB diff 0x%08lx", rtems_ntp_max_tbdiff);
#endif
		fputc('\n',stdout);

	ufrac2ts( rtems_ntp_max_t1, &ts );
	printf("    req. sent at (local time) %lu.%lu\n", ts.tv_sec, ts.tv_nsec );
	ufrac2ts( rtems_ntp_max_t2, &ts );
	printf("    received at (remote time) %lu.%lu\n", ts.tv_sec, ts.tv_nsec );
	ufrac2ts( rtems_ntp_max_t3, &ts );
	printf("    reply sent  (remote time) %lu.%lu\n", ts.tv_sec, ts.tv_nsec );
	t4 =  rtems_ntp_max_t3 - rtems_ntp_max_diff;

	if ( rtems_ntp_max_t1 && rtems_ntp_max_t2 )
		t4 +=  rtems_ntp_max_t2 - rtems_ntp_max_diff - rtems_ntp_max_t1;
	ufrac2ts( t4, &ts );
	printf("    reply received (lcl time) %lu.%lu\n", ts.tv_sec, ts.tv_nsec );

	t = ts.tv_sec - rtems_bsdnet_timeoffset - UNIX_BASE_TO_NTP_BASE;

	printf("Happened around %s\n", ctime(&t));
	if ( reset )
		rtems_ntp_max_diff = 0;
	return frac2nsec(rtems_ntp_max_diff);
}
#endif

#if NTP_DEBUG & NTP_DEBUG_MORE
static void printNtpTimestamp(struct timestamp *p)
{
	printf("%i.%i\n",p->integer,p->fraction);
}

unsigned
rtemsNtpDiff()
{
DiffTimeCbDataRec d;
	if ( 0 == rtems_bsdnet_get_ntp(rtems_ntp_daemon_sd,diffTimeCb, &d) ) {
		printf("%lli; %lis; %lins\n",d.diff, int2sec(d.diff), frac2nsec(d.diff));
		printf("%lli; %lis; %lins\n",-d.diff, -int2sec(-d.diff), -frac2nsec(-d.diff));
		printf("total roundtrip time was %luns\n", d.tripns);
	}
	return (unsigned)diff;
}

static int copyPacketCb(struct ntpPacketSmall *p, int state, void *usr_data)
{
	if ( 0 == state )
		memcpy(usr_data, p, sizeof(*p));
	return 0;
}

unsigned
rtemsNtpPacketGet(void *p)
{
	return rtems_bsdnet_get_ntp(rtems_ntp_daemon_sd,copyPacketCb,p);
}
#endif

/* Convenience routine to set poll interval */
int
rtemsNtpSetPollInterval(int poll_seconds)
{
struct timex ntv;

	if ( poll_seconds < 16 )
		poll_seconds = 16;

	ntv.modes    = 0;
	if ( ntp_adjtime(&ntv) )
		return -1;

	ntv.status  |= STA_PLL;
	ntv.constant = secs2tcld(poll_seconds);
	ntv.modes    = MOD_TIMECONST | MOD_STATUS;
	return ntp_adjtime(&ntv);
}

static rtems_interval
get_poll_interval()
{
struct timex   ntv;
rtems_interval rate;

	rate = rtems_clock_get_ticks_per_second();
	ntv.modes = 0;
	if ( ntp_adjtime(&ntv) ) {
		printk("NTP: warning; unable to determine poll interval; using 600s\n");
		return rate * 600;
	}
	return rate * (1<<(ntv.constant+4));
}

static int
acceptFiltered(unsigned long delay)
{
static unsigned long avgDelay = 0;
int rval;

	if ( 0 == avgDelay ) {
		avgDelay = delay;
		return 1;
	}

	rval = delay < avgDelay;

	/* avgDelay/delay = .5/(1-.75z) */
	avgDelay = ((3*avgDelay) >> 2) + (delay >> 1);
#if NTP_DEBUG & NTP_DEBUG_FILTER
	printf("Filter: delay %lu, 2*avg %lu %s\n", delay, avgDelay, rval ? "PASS":"DROP");
#endif
	return rval;
}


static rtems_task
ntpDaemon(rtems_task_argument unused)
{
rtems_status_code     rc;
rtems_event_set       got;
long                  nsecs;
float                 last_fusecs=0., jitter=0., ftmp, ftmp1, fusecs, maxerr=0.;
int                   retry, shots;
DiffTimeCbDataRec     data[2];
unsigned char         best, try;
struct timex          ntv;
int                   failedsyncs;
unsigned char         leap;
unsigned              r_s;

	ntv.modes = 0;
	ntp_adjtime(&ntv);

	/* initial lookup suceeded (during init); claim we're synced */
	ntv.status &= ~ STA_UNSYNC;
	ntv.modes   = MOD_STATUS;

	ntp_adjtime(&ntv);

	ntv.modes  |= MOD_MAXERROR | MOD_ESTERROR;

	failedsyncs = 0;

	while ( RTEMS_TIMEOUT == (rc = rtems_event_receive(
									KILL_DAEMON,
									RTEMS_WAIT | RTEMS_EVENT_ANY,
									get_poll_interval(),
									&got )) ) {

		for ( retry = 3; retry > 0; retry--  ) {

			/* Do a few shots and remember the best one */
			shots = 0;
			try   = best = 0;
			while ( retry > 0 ) {
				if ( rtems_bsdnet_get_ntp(rtems_ntp_daemon_sd, diffTimeCb, &data[try]) ) {
					--retry;	
					continue;
				}

				/* Use <= so that we switch after the first try */
				if ( data[try].tripns <= data[best].tripns ) {
					best = try;
					try  = (best+1)%2;
				}

				if ( ++shots >= 3 )
					break;
				
				/* wait for some randomized delay */
				rtems_task_wake_after( (rtems_interval)(rand_r(&r_s) & 63) );
			}

			/* If no shot succeeded we give up for this time */
			if ( !shots )
				break;

			/* drop answers with excessive roundtrip time */
			if ( !acceptFiltered(data[best].tripns) )
				continue;

			if ( data[best].diff > nsec2frac(MAXPHASE) )
				nsecs =  MAXPHASE;
			else if ( data[best].diff < -nsec2frac(MAXPHASE) )
				nsecs = -MAXPHASE;
			else
				nsecs = frac2nsec(data[best].diff);

#ifndef USE_PROFILER_RAW
			locked_hardupdate( nsecs ); 

			{
				/* statistics; basic algorithm stolen from ntpd */
				fusecs = (float)nsecs/1000.;

				ftmp   = jitter*jitter;
				ftmp1  = last_fusecs - fusecs;
				ftmp1  = ftmp1*ftmp1;

				jitter = sqrtf( ftmp + (ftmp1 - ftmp)/8. );

				last_fusecs = fusecs;

				maxerr = (float)data[best].rootdelay/65536. + 2.*(float)data[best].rootdisp/65536.;
				maxerr = 1.0E6*maxerr; /* in uS */
			}

			if ( rtems_ntp_debug ) {
				rtems_task_priority old_p;
				/* Lower priority for printing */
				rtems_task_set_priority(RTEMS_SELF, 180, &old_p);
				if ( rtems_ntp_debug_file ) {
					/* log difference in microseconds */
					fprintf(rtems_ntp_debug_file,"Diff: %.5g us (0x%016llx; %ld ns)\n", 1000000.*(double)data[best].diff/4./(double)(1<<30), data[best].diff, nsecs);
					fflush(rtems_ntp_debug_file);
				} else {
					long secs = int2sec(data[best].diff);
					printf("Update diff %li %sseconds\n", secs ? secs : nsecs, secs ? "" : "nano");
				}
				/* Restore priority */
				rtems_task_set_priority(RTEMS_SELF, old_p, &old_p);
			}
#endif
			break;

		}

		if ( retry <= 0 ) {
			failedsyncs++;
		} else  {
			failedsyncs = 0;

			/* Check for leap seconds */
			switch ( leap = (data[best].li_vn_mode & LEAP_MSK) ) {
				case LEAP_NO:
					ntv.status &= ~(STA_INS | STA_DEL);
				break;

				default:	/* ignore '3' for now */
				break;

				case LEAP_INS:
				case LEAP_DEL:
				{
				struct tm tm;
				time_t    tmp = data[best].srv_rcvts;

		            tmp -= rtems_bsdnet_timeoffset + UNIX_BASE_TO_NTP_BASE;
					/* Only announce to the kernel clock on the last day
					 * of June or December.
					 */
					if (    gmtime_r(&tmp, &tm) 
					    &&  (  (tm.tm_mon + 1 ==  6 && tm.tm_mday == 30)
						     ||
					           (tm.tm_mon + 1 == 12 && tm.tm_mday == 31) 
							) ) {
						if ( LEAP_INS == leap )
							ntv.status |= STA_INS;
						else
							ntv.status |= STA_DEL;
					}
				}
				break;
			}
		}

		if ( failedsyncs > MAX_FAILED_SYNCS ) {
			ntv.status |= STA_UNSYNC;
			/* prevent from overflowing */
			failedsyncs = MAX_FAILED_SYNCS + 1;
		} else {
			ntv.status &= ~STA_UNSYNC;
		}

		if ( retry > 0 ) {
			ntv.maxerror = maxerr;
			ntv.esterror = jitter;
			ntv.modes  |= MOD_MAXERROR | MOD_ESTERROR;
		} else {
			ntv.modes  &= ~ (MOD_MAXERROR | MOD_ESTERROR);
		}

		ntp_adjtime(&ntv);

		/* TODO: sync / calibrate hwclock hook */
	}

	ntv.modes  &= ~ (MOD_MAXERROR | MOD_ESTERROR);
	ntv.status |= STA_UNSYNC;
	ntp_adjtime(&ntv);

	if ( RTEMS_SUCCESSFUL == rc && (KILL_DAEMON==got) ) {
		rtems_semaphore_release(kill_sem);
		/* DONT delete ourselves; note the race condition - our creator
		 * who sent the KILL event might get the CPU and remove us from
		 * memory before we get a chance to delete!
		 */
	}
	rtems_task_suspend( RTEMS_SELF );
}


#ifndef USE_PICTIMER

unsigned rtems_ntp_ticker_misses = 0;

static rtems_task
tickerDaemon(rtems_task_argument unused)
{
rtems_id			pid;
rtems_status_code	rc;

	PARANOIA( rtems_rate_monotonic_create( rtems_build_name('n','t','p','T'), &pid ) );

	tickerRunning = 1;

	while ( tickerRunning ) {

		rc = rtems_rate_monotonic_period( pid, RATE_DIVISOR );

		if ( RTEMS_TIMEOUT == rc )
			rtems_ntp_ticker_misses++;

		ticker_body();

	}
	PARANOIA( rtems_rate_monotonic_delete( pid ) );
	PARANOIA( rtems_semaphore_release( kill_sem ) );
	rtems_task_suspend( RTEMS_SELF );
}

#else

static rtems_task
tickerDaemon(rtems_task_argument unused)
{
rtems_event_set		got;

	while ( 1 ) {

		/* if they want to kill this daemon, they send a zero sized message
		 */
		PARANOIA ( rtems_event_receive(
									KILL_DAEMON | PICTIMER_SYNC_EVENT,
									RTEMS_WAIT | RTEMS_EVENT_ANY,
									RTEMS_NO_TIMEOUT,
									&got ) );

		if ( KILL_DAEMON & got ) {
			break;
		}
		
		ticker_body();
	}

	/* they killed us */
	PARANOIA( rtems_semaphore_release( kill_sem ) );
	rtems_task_suspend( RTEMS_SELF );
}
#endif

static void
closeSd()
{
	if ( our_sd ) {
		close(rtems_ntp_daemon_sd);
		rtems_ntp_daemon_sd = 0;
		our_sd = 0;
	}
}

int
rtemsNtpInitialize(unsigned tickerPri, unsigned daemonPri)
{
struct timex    ntv;
struct timespec initime;
struct sockaddr me;

	if ( ! tickerPri ) {
		tickerPri = 35;
	}
	if ( ! daemonPri ) {
		daemonPri = rtems_bsdnet_config.network_task_priority;
		if ( ! daemonPri )
			daemonPri = 30;

		if ( daemonPri > 2 )
			daemonPri -= 2;
	}
	/* TODO: recover frequency from NVRAM; read time from hwclock */

	ntv.offset = 0;
	ntv.freq   = 0;
	ntv.status = STA_PLL | STA_UNSYNC;
	ntv.constant = secs2tcld(DAEMON_SYNC_INTERVAL_SECS);
	ntv.modes = MOD_STATUS | MOD_NANO |
	            MOD_TIMECONST |
	            MOD_OFFSET | MOD_FREQUENCY;

	rtems_ntp_debug_file = stdout;

#ifdef USE_PICTIMER
	hz = TIMER_FREQ;
#else
	{
	rtems_interval rate;
	rate = rtems_clock_get_ticks_per_second();
	hz = rate / RATE_DIVISOR;
	}
#endif

	ntp_init();
	ntp_adjtime(&ntv);

#ifdef USE_PICTIMER
	if ( pictimerInstallClock( TIMER_NO ) ) {
		return -1;
	}
#endif

	if ( rtems_ntp_daemon_sd ) {
		if ( rtems_ntp_daemon_sd < 0 ) {
			/* let NTP code create/delete socket */
			rtems_ntp_daemon_sd = -1;
		}   /* else use pre-opened socked */
	} else {
		if ( (rtems_ntp_daemon_sd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
			fprintf(stderr, "rtemsNtpInitialize(): unable to create socket: %s\n", strerror(errno));
			goto bail;
		}
		our_sd = 1;
		memset(&me, 0, sizeof(me));
		me.sa_family = AF_INET;
		if ( bind(rtems_ntp_daemon_sd, &me, sizeof(me)) ) {
			fprintf(stderr, "rtemsNtpInitialize(): unable to bind socket: %s\n", strerror(errno));
			goto bail;
		}
	}

	fprintf(stderr,"Trying to contact NTP server; (timeout ~1min.)... ");
	fflush(stderr);

	/* initialize time */
	if ( rtems_bsdnet_get_ntp(rtems_ntp_daemon_sd, 0, &initime) ) {
		fprintf(stderr,"FAILED: check networking setup and try again\n");
		goto bail;
	}
	fprintf(stderr,"OK\n");

#ifdef NTP_NANO
	TIMEVAR = initime;
#else
	TIMEVAR.tv_sec  = initime.tv_sec;
	TIMEVAR.tv_usec = initime.tv_nsec/1000;
#endif

	if ( RTEMS_SUCCESSFUL != rtems_semaphore_create(
								rtems_build_name('N','T','P','m'),
								1,
								RTEMS_LOCAL | RTEMS_BINARY_SEMAPHORE |
								RTEMS_PRIORITY | RTEMS_INHERIT_PRIORITY,
								0,
								&mutex_id ) ) {
		printf("Unable to create mutex\n");
		goto bail;
	}

	if ( RTEMS_SUCCESSFUL != rtems_task_create(
								rtems_build_name('C','L','K','d'),
								tickerPri,
								RTEMS_MINIMUM_STACK_SIZE,
								RTEMS_DEFAULT_MODES,
								RTEMS_DEFAULT_ATTRIBUTES,
								&rtems_ntp_ticker_id) ||
	     RTEMS_SUCCESSFUL != rtems_task_start( rtems_ntp_ticker_id, tickerDaemon, 0) ) {
		printf("Clock Ticker daemon couldn't be started :-(\n");
		goto bail;
	}

	if ( RTEMS_SUCCESSFUL != rtems_task_create(
								rtems_build_name('N','T','P','d'),
								daemonPri,
								RTEMS_MINIMUM_STACK_SIZE,
								RTEMS_DEFAULT_MODES,
								RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT,
								&rtems_ntp_daemon_id) ||
	     RTEMS_SUCCESSFUL != rtems_task_start( rtems_ntp_daemon_id, ntpDaemon, 0) ) {
		printf("NTP daemon couldn't be started :-(\n");
		/* PICTIMER might still be in use; */
		goto bail1;
	}

#ifdef USE_METHOD_B_FOR_DEMO
	PARANOIA( rtems_timer_create(
					rtems_build_name('N','T','P','t'),
					&sysclk_irq_id) );
	PARANOIA( rtems_timer_fire_after( sysclk_irq_id, 1, sysclkIrqHook, 0 ) );
#endif

#ifdef USE_PICTIMER
	/* start timer */
	pictimerEnable(TIMER_NO, 1);
#endif
#ifdef USE_PROFILER
	pictimerProfileInstall();
#endif

	printf("NTP synchro code initialized; this is EXPERIMENTAL\n");
#ifdef USE_NO_HIGH_RESOLUTION_CLOCK
	fprintf(stderr,"WARNING: High resolution clock not implemented for this CPU\n");
	fprintf(stderr,"         please contribute to <ntpNanoclock/pcc.h>\n");
#endif
	return 0;
bail:
#ifdef USE_PICTIMER
	pictimerCleanup(TIMER_NO);
#endif
bail1:
	closeSd();
	return -1;
}

int rtemsNtpCleanup()
{

#ifdef USE_PICTIMER
	if ( pictimerCleanup(TIMER_NO) ) {
		return -1;
	}
#endif

	if ( RTEMS_SUCCESSFUL == rtems_semaphore_create(
								rtems_build_name('k','i','l','l'),
								0,
								RTEMS_LOCAL | RTEMS_SIMPLE_BINARY_SEMAPHORE,
								0,
								&kill_sem) ) {
			if ( rtems_ntp_daemon_id ) {
				PARANOIA( rtems_event_send( rtems_ntp_daemon_id, KILL_DAEMON ) );
				PARANOIA( rtems_semaphore_obtain( kill_sem, RTEMS_WAIT, RTEMS_NO_TIMEOUT ) );
				PARANOIA( rtems_task_delete( rtems_ntp_daemon_id ));
			}
			if ( rtems_ntp_ticker_id ) {
#ifdef USE_PICTIMER
				PARANOIA( rtems_event_send( rtems_ntp_ticker_id, KILL_DAEMON ) );
#else
				tickerRunning = 0;
#endif
				/* end of aborting sequence */
				PARANOIA( rtems_semaphore_obtain( kill_sem, RTEMS_WAIT, RTEMS_NO_TIMEOUT ) );
				PARANOIA( rtems_task_delete( rtems_ntp_ticker_id ) );
			}
			PARANOIA( rtems_semaphore_release( kill_sem ) );
			PARANOIA( rtems_semaphore_delete( kill_sem ) );	
	} else {
		return -1;
	}

#ifdef USE_METHOD_B_FOR_DEMO
	if ( sysclk_irq_id )
		PARANOIA( rtems_timer_delete( sysclk_irq_id ) );
#endif

	if ( mutex_id )
		PARANOIA( rtems_semaphore_delete( mutex_id ) );

	closeSd();

#ifdef USE_PROFILER
	return pictimerProfileCleanup();
#else
	return 0;
#endif
}

#ifdef HAVE_CEXP
int
_cexpModuleFinalize(void *handle)
{
	return rtemsNtpCleanup();
}
#endif


#ifdef NTP_NANO
#define UNITS	"nS"
#else
#define UNITS	"uS"
#endif

static char *yesno(struct timex *p, unsigned mask)
{
	return p->status & mask ? "YES" : "NO";
}


long rtemsNtpDumpStats(FILE *f)
{
struct timex ntp;

	if ( !f )
		f = stdout;

	memset( &ntp, 0, sizeof(ntp) );
	if ( 0 == ntp_adjtime( &ntp ) ) {
		fprintf(stderr,"Current Timex Values:\n");
		fprintf(stderr,"            time offset %11li "UNITS"\n",  ntp.offset);
		fprintf(stderr,"       frequency offset %11.3f ""ppm""\n", (double)ntp.freq/PPM_SCALED);
		fprintf(stderr,"             max  error %11li ""uS""\n",   ntp.maxerror);
		fprintf(stderr,"        estimated error %11li ""uS""\n",   ntp.esterror);
		fprintf(stderr,"          poll interval %11i  ""S""\n",    1<<(ntp.constant+4));
		fprintf(stderr,"              precision %11li "UNITS"\n",  ntp.precision);
		fprintf(stderr,"              tolerance %11.3f ""ppm""\n", (double)ntp.tolerance/PPM_SCALED);
		fprintf(stderr,"    PLL updates enabled %s\n", yesno(&ntp, STA_PLL));
		fprintf(stderr,"       FLL mode enabled %s\n", yesno(&ntp, STA_FLL));
		fprintf(stderr,"            insert leap %s\n", yesno(&ntp, STA_INS));
		fprintf(stderr,"            delete leap %s\n", yesno(&ntp, STA_DEL));
		fprintf(stderr,"   clock unsynchronized %s\n", yesno(&ntp, STA_UNSYNC));
		fprintf(stderr,"         hold frequency %s\n", yesno(&ntp, STA_FREQHOLD));
		fprintf(stderr,"   clock hardware fault %s\n", yesno(&ntp, STA_CLOCKERR));
		fprintf(stderr,     "          %ssecond resolution\n", ntp.status & STA_NANO ? " nano" : "micro");
		fprintf(stderr,"         operation mode %s\n", ntp.status & STA_MODE ? "FLL" : "PLL");
		fprintf(stderr,"           clock source %s\n", ntp.status & STA_CLK  ? "B"   : "A");
	}
		fprintf(stderr,"Estimated Nanoclock Frequency:\n");
		fprintf(stderr,"   %lu clicks/%lu ns = %.10g MHz\n",
							pcc_denominator,
							pcc_numerator,
							pcc_numerator ? (double)pcc_denominator/(double)pcc_numerator*1000. : (double)-1.);
	return 0;
}

int
rtemsNtpPrintTime(FILE *fp)
{
struct ntptimeval ntv;
char              buf[30];

	if ( ntp_gettime(&ntv) )
		return -1;

	if ( !fp )
		fp = stdout;

	if ( fp != (FILE*)-1 ) {
		ctime_r(&ntv.time.tv_sec, buf);
		fprintf(fp,"%s",buf);
	}
	
	return ntv.time.tv_sec;
}

#endif
