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

varmesh<4> move(varmesh<4> mesh, v<3> translate)
{
    v4 translate4{ translate[0], translate[1], translate[2], 0 };

    for (int i = 0; i < mesh.row_size(); i++)
        for (int j = 0; j < mesh.col_size(); j++)
            mesh.element(i, j) = mesh.element(i, j) + mesh.element(i, j)[3] * translate4;

    return mesh;
}

varmesh<4> scale(varmesh<4> mesh, double scale)
{
    for (int i = 0; i < mesh.row_size(); i++)
        for (int j = 0; j < mesh.col_size(); j++)
            for (int k = 0; k < 3; k++)
                mesh.element(i, j)[k] = scale * mesh.element(i, j)[k];

    return mesh;
}