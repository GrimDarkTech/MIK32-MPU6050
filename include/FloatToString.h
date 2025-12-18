#ifndef FLOAT_TO_STRING_H
#define FLOAT_TO_STRING_H

#include <stdint.h>

void float2string(float value, char* string, uint32_t stringSize, uint32_t position)
{
    if (position + 4 > stringSize) 
    {
        return; 
    }

    float multipliedValue = value * 100.0f;
    float offset;

    if (value >= 0) 
    {
        offset = 0.5f;
    } 
    else 
    {
        offset = -0.5f;
    }

    int intValue = (int)(multipliedValue + offset);

    string[position]     = (char)(intValue & 0xFF);
    string[position + 1] = (char)((intValue >> 8) & 0xFF);
    string[position + 2] = (char)((intValue >> 16) & 0xFF);
    string[position + 3] = (char)((intValue >> 24) & 0xFF);
}

#endif // FLOAT_TO_STRING_H