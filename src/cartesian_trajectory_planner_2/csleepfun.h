/* C++ function for sleep.*/

/* Copyright 2015-2018 The MathWorks, Inc. */

#ifndef CSLEEPFUN_H
#define CSLEEPFUN_H

/** 
 * Sleep for a given number of seconds.
 * The sleep time is not affected by discontinuous jumps in the system time, 
 * for example on manual time changes or during Daylight Savings Time.
 *
 * @param seconds Seconds to sleep
 */
#ifdef __cplusplus
extern "C"
{
#endif
void csleepfun(double seconds);
#ifdef __cplusplus
}
#endif
#endif /* CSLEEPFUN_H */