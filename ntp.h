
#ifndef _NTP_H_
#define _NTP_H_

#include <stdint.h>

struct timestamp {
	uint32_t	integer;
	uint32_t	fraction;
};

/* Data is passed in network byte order */
struct ntpPacketSmall {
	uint8_t				li_vn_mode;
	uint8_t				stratum;
	int8_t				poll_interval;
	int8_t				precision;
	int32_t				root_delay;
	int32_t				root_dispersion;
	char				reference_identifier[4];
	struct timestamp	reference_timestamp;
	struct timestamp	originate_timestamp;
	struct timestamp	receive_timestamp;
	struct timestamp	transmit_timestamp;
};

#endif // _NTP_H_
