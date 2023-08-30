#ifndef graphics_formulas_h
#define graphics_formulas_h

#include <geometry/types/vector.h>

double shade(const v3& normale, const v3& light_direction);

v3 random_unit_vector(v3 normale);

#endif
