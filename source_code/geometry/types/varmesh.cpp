#include "varmesh.h"
#include <geometry/linear_algebra/formulas.h>

v3 normale_of_varmesh(const varmesh<4>& m, size_t r, size_t c)
{
    if (r == m.row_size() - 1)
    {
        r--;
    }

    if (c == m.col_size() - 1)
    {
        c--;
    }

    const v3& origin = remove_dimension(m.element(r, c));
    const v3& e01 = remove_dimension(m.element(r + 1, c));
    const v3& e10 = remove_dimension(m.element(r, c + 1));

    return cross_product(e01 - origin, e10 - origin);
}
