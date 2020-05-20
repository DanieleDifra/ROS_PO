/* Copyright 2015-2018 The MathWorks, Inc. */

#include "roundtolong.h"
#include "csleepfun.h"

/* Make functionality from the 2001 edition of the POSIX.1b standard available to
   get clock_nanosleep definition */
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200112L
#endif

#include <stdio.h>
#include <sys/time.h>
#include <time.h>

struct timespec timespecAdd(struct timespec time1, struct timespec time2)
{
    struct timespec result;

    /* Add the two times together. */

    result.tv_sec = time1.tv_sec + time2.tv_sec ;
    result.tv_nsec = time1.tv_nsec + time2.tv_nsec ;
    if (result.tv_nsec >= 1000000000L) {		/* Carry? */
        result.tv_sec++ ;  result.tv_nsec = result.tv_nsec - 1000000000L ;
    }

    return (result) ;
}

void csleepfun(double seconds)
{
    struct timespec currentTime;
    struct timespec sleepTime;
    struct timespec absSleepEndTime;

    /* Get current time */
    clock_gettime(CLOCK_MONOTONIC, &currentTime);

    /* Construct sleep time */
    sleepTime.tv_sec = (time_t) seconds;
    sleepTime.tv_nsec = roundToLong((seconds - sleepTime.tv_sec) * 1e9);

    /* Add sleep time to current time */
    absSleepEndTime = timespecAdd(currentTime, sleepTime);

    /* Use CLOCK_MONOTONIC, so that sleep time is not affected by
     discontinuous jumps in the system time, e.g. on manual time changes or
     during Daylight Savings Time */

    /* User TIMER_ABSTIME to allow an acccurate sleep interval if the sleep
       is interrupted by signals.
       Call clock_nanosleep in a while loop to ensure that the sleep is
       continued after a signal interruption (the return value will be positive
       if a signal interrupt occured). */

    while(clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &absSleepEndTime, NULL));

    return;
}

