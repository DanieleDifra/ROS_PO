/* C++ function for getting system time. */

/* Copyright 2015-2018 The MathWorks, Inc. */

#ifndef CTIMEFUN_H
#define CTIMEFUN_H

/** 
 * Get current system time in seconds.
 * The returned time is monotonically increasing and is not affected 
 * by discontinuous jumps in the system time, for example on manual time 
 * changes or during Daylight Savings Time.
 * Note that the system time is measured relative to some baseline, but has 
 * no relation to the calendar time.
 *
 * @return Current system time (in seconds)
 */
#ifdef __cplusplus
extern "C"
{
#endif
double ctimefun();
#ifdef __cplusplus
}
#endif
#endif /* CTIMEFUN_H */