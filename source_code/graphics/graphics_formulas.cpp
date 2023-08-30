#include "graphics_formulas.h"

#include <random>

double shade(const v3& normale, const v3& light_direction)
{
    return (1 + normalize(normale) * normalize(light_direction)) / 2;
}

v3 random_unit_vector(v3 normale)
{
    static std::random_device rd;  // Will be used to obtain a seed for the random number engine
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<> dis(-1, 1);
    while (true)
    {
        v3 candidate{ dis(gen), dis(gen), dis(gen) };
        auto l2 = length2(candidate);
        if (l2 < 1)
        {
            if (normale * candidate < 0)
            {
                candidate = -candidate;
            }
            return normalize(candidate);
        }
    }
}
