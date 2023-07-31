#ifndef matrix_h
#define matrix_h

#include <array>

#include "vector.h"

template <size_t ROW, size_t COL>
using matrix = std::array<std::array<double, COL>, ROW>;

matrix<4, 4> rotation_in_e1(double rad);
matrix<4, 4> rotation_in_e2(double rad);
matrix<4, 4> rotation_in_e3(double rad);

template <size_t r, size_t c> v<r> operator*(const matrix<r, c> &m, const v<c> &v)
{
    std::array<double, r> result;
    
    for (int i = 0; i < r; i++)
    {
        result[i] = 0;
        for (int j = 0; j < c; j++)
        {
            result[i] += m[i][j] * v[j];
        }
    }
    
    return result;
}

template <size_t r, size_t rc, size_t c> matrix<r, c> operator*(const matrix<r, rc> &m, const matrix<rc, c> &n)
{
    matrix<r, c> result;
    
    for (int i = 0; i < r; i++)
    {
        for (int j = 0; j < c; j++)
        {
            result[i][j] = 0;
            for (int k = 0; k < rc; k++)
            {
                result[i][j] += m[i][k] * n[k][j];
            }
        }
    }
    
    return result;
}


#endif /* matrix_h */
