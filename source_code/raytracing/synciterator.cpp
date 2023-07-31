#include "synciterator.h"

synciterator::synciterator(int px, int py) : i{-1}, j{0}, x{px}, y{py}
{
    
}

std::pair<int, int> synciterator::next()
{
    std::lock_guard<std::mutex> guard(m);
    
    if (i >= x - 1 && j >= y - 1)
    {
        return std::make_pair(-1, -1);
    }
    
    i++;
    
    if (i == x)
    {
        i = 0;
        j++;
    }
    
    return std::make_pair(i, j);
}
