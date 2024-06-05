#ifdef ESP8266

#include <sys/types.h>
#include <sys/times.h>
#include <unistd.h>

clock_t _times_r(struct _reent *r, struct tms *tm) {
    return (clock_t)-1;
}

#endif