#ifndef INC_LUT_H_
#define INC_LUT_H_

#include <stdbool.h>

/* Bilinear interpolation into the 51x51 PAA5163 resolution-variation LUT.
 * inX, inY: measured velocities in inches/second (convert from m/s: * 39.37f)
 * outX, outY: scaling factors to multiply measured velocity by
 * Returns false if input is out of the ±100 in/s table bounds (scalars unchanged).
 * Ported from SFE/Inc/otosLut.h (SparkFun OTOS, MIT License). */
bool bilinearInterp(float inX, float inY, float *outX, float *outY);

#endif /* INC_LUT_H_ */
