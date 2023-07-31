#include <cmath>

#include "matrix.h"

matrix<4, 4> rotation_in_e1(double rad)
{
    return matrix<4, 4>{{
        std::array<double, 4>{{1, 0, 0, 0}},
        std::array<double, 4>{{0, cos(rad), -sin(rad), 0}},
        std::array<double, 4>{{0, sin(rad), cos(rad), 0}},
        std::array<double, 4>{{0, 0, 0, 1}}
    }};
}

matrix<4, 4> rotation_in_e2(double rad)
{
    return matrix<4, 4>{{
        std::array<double, 4>{{cos(rad), 0, sin(rad), 0}},
        std::array<double, 4>{{0, 1, 0, 0}},
        std::array<double, 4>{{-sin(rad), 0, cos(rad), 0}},
        std::array<double, 4>{{0, 0, 0, 1}}
    }};
}

matrix<4, 4> rotation_in_e3(double rad)
{
    return matrix<4, 4>{{
        std::array<double, 4>{{cos(rad), -sin(rad), 0, 0}},
        std::array<double, 4>{{sin(rad), cos(rad), 0, 0}},
        std::array<double, 4>{{0, 0, 1, 0}},
        std::array<double, 4>{{0, 0, 0, 1}}
    }};
}
