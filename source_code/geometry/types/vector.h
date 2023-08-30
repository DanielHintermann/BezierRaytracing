#ifndef vector_h
#define vector_h

#include <array>
#include <vector>
#include <cmath>

template <size_t N> using v = std::array<double, N>;
typedef v<1> v1;
typedef v<2> v2;
typedef v<3> v3;
typedef v<4> v4;

template<std::size_t N> v<N> operator+(const v<N>& p, const v<N>& q)
{
    v<N> result;
    
    for (int i = 0; i < N; i++)
    {
        result[i] = p[i] + q[i];
    }
    
    return result;
}

template<std::size_t N> double operator*(const v<N>& p, const v<N>& q)
{
    double result = 0;
    
    for (int i = 0; i < N; i++)
    {
        result += p[i] * q[i];
    }
    
    return result;
}

template<std::size_t N> v<N> operator-(const v<N>& p, const v<N>& q)
{
    std::array<double, N> result;
    
    for (int i = 0; i < N; i++)
    {
        result[i] = p[i] - q[i];
    }
    
    return result;
}

template<std::size_t N> v<N> operator-(const v<N>& p)
{
    std::array<double, N> result;
    
    for (int i = 0; i < N; i++)
    {
        result[i] = -p[i];
    }
    
    return result;
}

template<std::size_t N> v<N> operator*(double scale, const v<N>& q)
{
    std::array<double, N> result;
    
    for (int i = 0; i < N; i++)
    {
        result[i] = scale*q[i];
    }
    
    return result;
}

template<std::size_t N> v<N> component_mul(const v<N>& p, const v<N>& q)
{
    std::array<double, N> result;

    for (int i = 0; i < N; i++)
    {
        result[i] = p[i] * q[i];
    }

    return result;
}

template<std::size_t N> double length2(const v<N>& p)
{
    return p * p;
}

template<std::size_t N> double length(const v<N>& p)
{
    return std::sqrt(length2(p));
}

template<std::size_t N> double l_inf(const v<N>& p)
{
    double max = std::abs(p[0]);
    
    for (int i = 1; i < N; i++)
    {
        if (max < std::abs(p[i]))
        {
            max = std::abs(p[i]);
        }
    }
    
    return max;
}

template<std::size_t N> v<N+1> add_dimension(const v<N>& p)
{
    v<N + 1> result;
    
    for (int i = 0; i < N; i++)
    {
        result[i] = p[i];
    }
    
    result[N] = 1;
    
    return result;
}

template<std::size_t N> v<N-1> remove_dimension(const v<N>& p)
{
    v<N - 1> result;
    
    double scale = (0 == p[N-1] ? 0 : 1./p[N-1]);
    
    for (int i = 0; i < N - 1; i++)
    {
        result[i] = scale * p[i];
    }
    
    return result;
}

template<std::size_t N> v<N - 1> remove_last_component(const v<N>& p)
{
    v<N - 1> result;

    for (int i = 0; i < N - 1; i++)
    {
        result[i] = p[i];
    }

    return result;
}

template<std::size_t N> v<N> normalize(const v<N>& p)
{
    double l = length(p);
    
    double scale = (0 == l ? 0 : 1./l);
    
    return scale * p;
}

#endif /* vector_h */
