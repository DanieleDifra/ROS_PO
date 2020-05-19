/* Copyright 2015-2018 The MathWorks, Inc. */
#include "roundtolong.h"

long roundToLong(double input)
{
    long output = 0;
    long longInput = (long)input;
    if (input - longInput >= 0.5)
        output = longInput + 1;
    else
        output = longInput;
    return output;
}
