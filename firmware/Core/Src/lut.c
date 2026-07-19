#include "lut.h"
#include "otosLut.h"

bool bilinearInterp(float inX, float inY, float *outX, float *outY)
{
    /* Floor to integer: truncation works for positive, subtract 1 for negative */
    int8_t inXInt = (int8_t)inX;
    int8_t inYInt = (int8_t)inY;
    if (inX < 0.0f) inXInt--;
    if (inY < 0.0f) inYInt--;

    /* Bounds check against table range */
    if (inXInt < lutIndices[0] || inXInt >= lutIndices[numLutIndices - 1] ||
        inYInt < lutIndices[0] || inYInt >= lutIndices[numLutIndices - 1])
    {
        return false;
    }

    /* Binary search for X bracket index i */
    uint8_t low = 0;
    uint8_t high = numLutIndices - 1u;
    uint8_t i = 0;
    while (low <= high)
    {
        i = (uint8_t)((low + high) / 2u);
        if (lutIndices[i] < inXInt)       low  = i + 1u;
        else if (lutIndices[i] > inXInt)  high = i - 1u;
        else                              break;
    }

    /* Binary search for Y bracket index j */
    low  = 0;
    high = numLutIndices - 1u;
    uint8_t j = 0;
    while (low <= high)
    {
        j = (uint8_t)((low + high) / 2u);
        if (lutIndices[j] < inYInt)       low  = j + 1u;
        else if (lutIndices[j] > inYInt)  high = j - 1u;
        else                              break;
    }

    /* Four corners of the bilinear cell */
    float x1 = (float)lutIndices[i];
    float y1 = (float)lutIndices[j];
    float x2 = (float)lutIndices[i + 1u];
    float y2 = (float)lutIndices[j + 1u];

    float denomInv = 1.0f / ((x2 - x1) * (y2 - y1));
    float dx1 = inX - x1;
    float dy1 = inY - y1;
    float dx2 = x2 - inX;
    float dy2 = y2 - inY;
    float dx1dy1 = dx1 * dy1;
    float dx2dy1 = dx2 * dy1;
    float dx1dy2 = dx1 * dy2;
    float dx2dy2 = dx2 * dy2;

    /* Interpolate X table */
    float q11 = (float)xResVarLut[j][i];
    float q12 = (float)xResVarLut[j][i + 1u];
    float q21 = (float)xResVarLut[j + 1u][i];
    float q22 = (float)xResVarLut[j + 1u][i + 1u];
    *outX = (q11 * dx2dy2 + q21 * dx1dy2 + q12 * dx2dy1 + q22 * dx1dy1) * denomInv;

    /* Interpolate Y table */
    q11 = (float)yResVarLut[j][i];
    q12 = (float)yResVarLut[j][i + 1u];
    q21 = (float)yResVarLut[j + 1u][i];
    q22 = (float)yResVarLut[j + 1u][i + 1u];
    *outY = (q11 * dx2dy2 + q21 * dx1dy2 + q12 * dx2dy1 + q22 * dx1dy1) * denomInv;

    /* Convert byte-encoded value to scaling factor */
    *outX = (*outX * lutResolution) + 1.0f;
    *outY = (*outY * lutResolution) + 1.0f;

    return true;
}
