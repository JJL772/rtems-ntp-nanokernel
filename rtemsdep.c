#include <rtems.h>
#include <bsp.h>
#include <bsp/bspExt.h>
#include <bsp/irq.h>
#include <rtems/rtems_bsdnet.h>
#include <netinet/in.h>
#include <string.h>
#include <time.h>
#include <assert.h>

#include <cexp.h>

#include "kern.h"
#include "rtemsdep.h"
#include "timex.h"
#include "pcc.h"

#ifdef USE_PICTIMER
#include "pictimer.h"
#endif

#define NTP_DEBUG

#define DAEMON_SYNC_INTERVAL_SECS	1
#define KILL_DAEMON					RTEMS_EVENT_1

#ifdef USE_PICTIMER
#if KILL_DAEMON == PICTIMER_SYNC_EVENT
#error PICTIMER_SYNC_EVENT collides with KILL_DAEMON event
#endif
#endif

#define PPM_SCALE					(1<<16)
#define PPM_SCALED					((double)PPM_SCALE)

#ifdef NTP_NANO
struct timespec TIMEVAR;	/* kernel nanosecond clock */
#else
struct timeval TIMEVAR;		/* kernel microsecond clock */
#endif

int microset_flag[NCPUS];	/* microset() initialization filag */
int hz;

static rtems_id daemon_id = 0;
       rtems_id rtems_ntp_ticker_id = 0;
static rtems_id kill_sem;
static rtems_id mutex_id  = 0;
#ifndef USE_PICTIMER
static volatile int tickerRunning = 0;
#endif
#ifdef USE_METHOD_B_FOR_DEMO
static rtems_id sysclk_irq_id = 0;
#endif

volatile unsigned      rtems_ntp_debug = 1;
volatile unsigned      rtems_ntp_daemon_sync_interval_secs = DAEMON_SYNC_INTERVAL_SECS;
volatile FILE		  *rtems_ntp_debug_file;

int
splx(int level)
{
	if ( level )
		rtems_semaphore_release( mutex_id );
	return 0;
}

int
splclock()
{
	rtems_semaphore_obtain( mutex_id, RTEMS_WAIT, RTEMS_NO_TIMEOUT );
	return 1;
}

/*
 * RTEMS base: 1988, January 1
 *  UNIX base: 1970, January 1
 *   NTP base: 1900, January 1
 */
#define UNIX_BASE_TO_NTP_BASE (((70UL*365UL)+17UL) * (24*60*60))

#define PARANOIA(something)	assert( RTEMS_SUCCESSFUL == (something) )

#ifdef NTP_DEBUG
long long rtems_ntp_max_t1;
long long rtems_ntp_max_t2;
long long rtems_ntp_max_t3;
long long rtems_ntp_max_diff   = 0;
#ifdef __PPC__
long      rtems_ntp_max_tbdiff = 0;
#endif

static inline long long llabs(n)
{
	return n < 0 ? -n : n;
}
#endif

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

static int copyPacketCb(struct ntpPacketSmall *p, int state, void *usr_data)
{
	if ( 0 == state )
		memcpy(usr_data, p, sizeof(*p));
	return 0;
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

static void pp(struct timestamp *p)
{
	printf("%i.%i\n",p->integer,p->fraction);
}

static int diffTimeCb(struct ntpPacketSmall *p, int state, void *usr_data)
{
struct timespec nowts;
long long       now, diff, org, rcv;
#if defined(NTP_DEBUG) && defined(__PPC__)
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
#if defined(NTP_DEBUG) && defined(__PPC__)
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
			}
			*(long long*)usr_data = diff;
#ifdef NTP_DEBUG
			if ( llabs(diff) > llabs(rtems_ntp_max_diff) ) {
				asm volatile("mftb %0":"=r"(tbnow));
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

#ifdef NTP_DEBUG
static inline void ufrac2ts(unsigned long long f, struct timespec *pts)
{
	pts->tv_sec = f>>32;
	f = ((f & ((1ULL<<32)-1)) * NANOSECOND ) >> 32;
	pts->tv_sec += f/NANOSECOND;
	pts->tv_nsec = f % NANOSECOND;
}

long
rtems_ntp_print_maxdiff(int reset)
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

unsigned
ntpdiff()
{
long long diff;
	if ( 0 == rtems_bsdnet_get_ntp(-1,diffTimeCb, &diff) ) {
		printf("%lli; %lis; %lins\n",diff, int2sec(diff), frac2nsec(diff));
		printf("%lli; %lis; %lins\n",-diff, -int2sec(-diff), -frac2nsec(-diff));
	}
	return (unsigned)diff;
}

unsigned
ntppack(void *p)
{
	return rtems_bsdnet_get_ntp(-1,copyPacketCb,p);
}

static unsigned
secs2tcld(int secs)
{
unsigned rval,probe;
	for ( rval = 0, probe=1; probe < secs; rval++ )
		probe <<= 1;
	return rval;
}

static rtems_task
ntpDaemon(rtems_task_argument unused)
{
rtems_status_code     rc;
rtems_interval        rate;
rtems_event_set       got;
long                  nsecs;
int                   pollsecs = 0;

	rtems_clock_get( RTEMS_CLOCK_GET_TICKS_PER_SECOND , &rate );


goto firsttime;

	while ( RTEMS_TIMEOUT == (rc = rtems_event_receive(
									KILL_DAEMON,
									RTEMS_WAIT | RTEMS_EVENT_ANY,
									rate * pollsecs,
									&got )) ) {
		long long diff;
		if ( 0 == rtems_bsdnet_get_ntp(-1, diffTimeCb, &diff) ) {
			if ( diff > nsec2frac(MAXPHASE) )
				nsecs =  MAXPHASE;
			else if ( diff < -nsec2frac(MAXPHASE) )
				nsecs = -MAXPHASE;
			else
				nsecs = frac2nsec(diff);

			locked_hardupdate( nsecs ); 

			if ( rtems_ntp_debug ) {
				if ( rtems_ntp_debug_file ) {
					/* log difference in microseconds */
					fprintf(rtems_ntp_debug_file,"%.5g\n", 1000000.*(double)diff/4./(double)(1<<30));
					fflush(rtems_ntp_debug_file);
				} else {
					long secs = int2sec(diff);
					printf("Update diff %li %sseconds\n", secs ? secs : nsecs, secs ? "" : "nano");
				}
			}
		}

firsttime:
		if ( rtems_ntp_daemon_sync_interval_secs != pollsecs ) {
			struct timex ntv;
			pollsecs = rtems_ntp_daemon_sync_interval_secs;
			ntv.constant = secs2tcld(pollsecs);
			ntv.modes    = MOD_TIMECONST;
			ntp_adjtime(&ntv);
		}
	}
	if ( RTEMS_SUCCESSFUL == rc && (KILL_DAEMON==got) ) {
		rtems_semaphore_release(kill_sem);
		/* DONT delete ourselves; note the race condition - our creator
		 * who sent the KILL event might get the CPU and remove us from
		 * memory before we get a chance to delete!
		 */
	}
	rtems_task_suspend( RTEMS_SELF );
}

static unsigned long pcc_numerator;
static unsigned long pcc_denominator = 0;

long
nano_time(struct timespec *tp)
{
unsigned long long pccl;

#ifdef NTP_NANO
	*tp = TIMEVAR;
#else
	tp->tv_sec   = TIMEVAR.tv_sec;
	tp->tv_nsec  = TIMEVAR.tv_usec * 1000;
#endif

	pccl = getPcc();

	/* convert to nanoseconds */
	if ( pcc_denominator ) {
		pccl *= pcc_numerator;
		pccl /= pcc_denominator;

		tp->tv_sec  += pccl / NANOSECOND;
		tp->tv_nsec += pccl % NANOSECOND;

		if ( tp->tv_nsec >= NANOSECOND ) {
			tp->tv_nsec -= NANOSECOND;
			tp->tv_sec++;
		}
	}

	return 0;
}

static inline void
ticker_body()
{
int s;
#ifdef NTP_NANO
struct timespec ts;
#else
struct timeval  ts;
#endif

	s = splclock();

	ts = TIMEVAR;

	ntp_tick_adjust(&TIMEVAR, 0);
	second_overflow(&TIMEVAR);

	pcc_denominator = setPccBase();
	pcc_numerator   = 
#ifdef NTP_NANO
		TIMEVAR.tv_nsec - ts.tv_nsec 
#else
		(TIMEVAR.tv_usec - ts.tv_usec) * 1000
#endif
		+ (TIMEVAR.tv_sec - ts.tv_sec) * NANOSECOND
		;

	splx(s);
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

void
_cexpModuleInitialize(void *handle)
{
struct timex          ntv;
struct timespec       initime;

	ntv.offset = 0;
	ntv.freq = 0;
	ntv.status = STA_PLL;
	ntv.constant = secs2tcld(rtems_ntp_daemon_sync_interval_secs);
	ntv.modes = MOD_STATUS | MOD_NANO |
	            MOD_TIMECONST |
	            MOD_OFFSET | MOD_FREQUENCY;

	rtems_ntp_debug_file = stdout;

#ifdef USE_PICTIMER
	hz = TIMER_FREQ;
#else
	{
	rtems_interval rate;
	rtems_clock_get( RTEMS_CLOCK_GET_TICKS_PER_SECOND , &rate );
	hz = rate / RATE_DIVISOR;
	}
#endif

	ntp_init();
	ntp_adjtime(&ntv);

#ifdef USE_PICTIMER
	if ( pictimerInstall( &rtems_ntp_ticker_id ) ) {
		return;
	}
#endif

	/* initialize time */
	if ( 0 == rtems_bsdnet_get_ntp(-1, 0, &initime) ) {
#ifdef NTP_NANO
		TIMEVAR = initime;
#else
		TIMEVAR.tv_sec  = initime.tv_sec;
		TIMEVAR.tv_usec = initime.tv_nsec/1000;
#endif
	}

	if ( RTEMS_SUCCESSFUL != rtems_semaphore_create(
								rtems_build_name('N','T','P','m'),
								1,
								RTEMS_LOCAL | RTEMS_BINARY_SEMAPHORE |
								RTEMS_PRIORITY | RTEMS_INHERIT_PRIORITY,
								0,
								&mutex_id ) ) {
		printf("Unable to create mutex\n");
		return;
	}

	if ( RTEMS_SUCCESSFUL != rtems_task_create(
								rtems_build_name('C','L','K','d'),
								80,
								1000,
								RTEMS_DEFAULT_MODES,
								RTEMS_DEFAULT_ATTRIBUTES,
								&rtems_ntp_ticker_id) ||
	     RTEMS_SUCCESSFUL != rtems_task_start( rtems_ntp_ticker_id, tickerDaemon, 0) ) {
		printf("Clock Ticker daemon couldn't be started :-(\n");
		return;
	}

	if ( RTEMS_SUCCESSFUL != rtems_task_create(
								rtems_build_name('N','T','P','d'),
								120,
								10000,
								RTEMS_DEFAULT_MODES,
								RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT,
								&daemon_id) ||
	     RTEMS_SUCCESSFUL != rtems_task_start( daemon_id, ntpDaemon, 0) ) {
		printf("NTP daemon couldn't be started :-(\n");
		return;
	}

#ifdef USE_METHOD_B_FOR_DEMO
	PARANOIA( rtems_timer_create(
					rtems_build_name('N','T','P','t'),
					&sysclk_irq_id) );
	PARANOIA( rtems_timer_fire_after( sysclk_irq_id, 1, sysclkIrqHook, 0 ) );
#endif

#ifdef USE_PICTIMER
	/* start timer */
	pictimerEnable(1);
#endif
}

int
_cexpModuleFinalize(void *handle)
{

#ifdef USE_PICTIMER
	if ( pictimerCleanup() ) {
		return -1;
	}
#endif

	if ( RTEMS_SUCCESSFUL == rtems_semaphore_create(
								rtems_build_name('k','i','l','l'),
								0,
								RTEMS_LOCAL | RTEMS_SIMPLE_BINARY_SEMAPHORE,
								0,
								&kill_sem) ) {
			if ( daemon_id ) {
				PARANOIA( rtems_event_send( daemon_id, KILL_DAEMON ) );
				PARANOIA( rtems_semaphore_obtain( kill_sem, RTEMS_WAIT, RTEMS_NO_TIMEOUT ) );
				PARANOIA( rtems_task_delete( daemon_id ));
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
	return 0;
}

#ifdef NTP_NANO
#define UNITS	"nS"
#else
#define UNITS	"uS"
#endif

static char *yesno(struct timex *p, unsigned mask)
{
	return p->status & mask ? "YES" : "NO";
}


long dumpNtpStats(FILE *f)
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
		fprintf(stderr,"          poll interval %11i  ""S""\n",    1<<ntp.constant);
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
							(double)pcc_denominator/(double)pcc_numerator*1000.);
	return 0;
}
