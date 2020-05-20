/* C function for rounding double to long. */

/* Copyright 2015-2018 The MathWorks, Inc. */

#ifndef ROUNDTOLONG_H
#define ROUNDTOLONG_H

/** 
 * Round floating-point value to nearest long integer
 *
 * @param input The input floating-point value
 * @return Rounded long integer.
 */
#ifdef __cplusplus
extern "C"
{
#endif
long roundToLong(double input);
#ifdef __cplusplus
}
#endif
#endif /* ROUNDTOLONG_H */