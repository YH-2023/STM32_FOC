#include "filter.h"

float low_pass_filter(float input, float last_output, float alpha)
{
    return alpha * input + (1.0 - alpha) * last_output;
}
