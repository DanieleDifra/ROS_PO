/* Copyright 2015-2018 The MathWorks, Inc. */
#include "ctimefun.h"

/* Make functionality from the 1993 edition of the POSIX.1b standard available to
 * get CLOCK_MONOTONIC definition */
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 199309L
#endif

#include <stdio.h>
#include <sys/time.h>
#include <time.h>

double ctimefun()
{
    double t = 0.0;
    
    /* Use CLOCK_MONOTONIC, so that the returned time is not affected by
     discontinuous jumps in the system time, e.g. on manual time changes or
     during Daylight Savings Time */
    
    struct timespec currentTime;
    clock_gettime(CLOCK_MONOTONIC, &currentTime);
    t = (double)currentTime.tv_sec + (double)currentTime.tv_nsec*1e-9;
    
    return t;
}
