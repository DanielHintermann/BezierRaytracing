#include "graphics_formulas.h"

double shade(const v3& normale, const v3& light_direction)
{
    return (1 + normalize(normale) * normalize(light_direction))/2;
}

