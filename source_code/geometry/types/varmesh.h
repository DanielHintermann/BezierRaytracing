#ifndef varmesh_h
#define varmesh_h

#include "vector.h"

template<size_t DIM> class varmesh
{
public:
    class row_index {
    public:
        row_index(varmesh<DIM>& ref, int row) : referenced{ ref }, referenced_row{ row } {}
        v<DIM>& operator[](int col)
        {
            return referenced.element(referenced_row, col);
        }
    private:
        varmesh<DIM>& referenced;
        int referenced_row;
    };

    class const_row_index {
    public:
        const_row_index(const varmesh<DIM>& ref, int row) : referenced{ ref }, referenced_row{ row } {}
        const v<DIM>& operator[](int col) const
        {
            return referenced.element(referenced_row, col);
        }
    private:
        const varmesh<DIM>& referenced;
        int referenced_row;
    };

    varmesh(size_t r, size_t c): rows{r}, cols{c}, points{r * c}
    {
    }

    inline v<DIM>& element(size_t r, size_t c)
    {
        return points[r * cols + c];
    }
    
    inline const v<DIM>& element(size_t r, size_t c) const
    {
        return points[r * cols + c];
    }
    
    auto operator[](int row)
    {
        return row_index(*this, row);
    }

    const auto operator[](int row) const
    {
        return const_row_index(*this, row);
    }

    size_t col_size() const
    {
        return cols;
    }
    
    size_t row_size() const
    {
        return rows;
    }
    
private:
    size_t rows;
    size_t cols;
    std::vector<v<DIM>> points;
};

v3 normale_of_varmesh(const varmesh<4>& m, size_t r, size_t c);

template<std::size_t N> varmesh<N - 1> remove_dimension(const varmesh<N>& m)
{
    varmesh<N - 1> result(m.row_size(), m.col_size());

    for (size_t i = 0; i < m.row_size(); i++)
    {
        for (size_t j = 0; j < m.col_size(); j++)
        {
            result.element(i, j) = remove_dimension(m.element(i, j));
        }
    }

    return result;
}

template <size_t d> v<d> barycentre_of_mesh(const varmesh<d>& m)
{
    v<d> origin{ {0} };

    for (int i = 0; i < m.row_size(); i++)
    {
        for (int j = 0; j < m.col_size(); j++)
        {
            origin = origin + m.element(i, j);
        }
    }

    origin = (1. / (m.row_size() * m.col_size())) * origin;

    return origin;
}

template<size_t N> std::vector<v<N>> row(const varmesh<N>& m, size_t index)
{
    std::vector<v<N>> result;

    for (int i = 0; i < m.col_size(); i++)
    {
        result.push_back(m.element(index, i));
    }

    return result;
}

template<size_t N> std::vector<v<N>> col(const varmesh<N>& m, size_t index)
{
    std::vector<v<N>> result;

    for (int i = 0; i < m.row_size(); i++)
    {
        result.push_back(m.element(i, index));
    }

    return result;
}

template<size_t N> varmesh<N> transpose(const varmesh<N>& m)
{
    varmesh<N> result(m.col_size(), m.row_size());

    for (int i = 0; i < m.col_size(); i++)
    {
        for (int j = 0; j < m.row_size(); j++)
        {
            result.element(i, j) = m.element(j, i);
        }
    }

    return result;
}

template <size_t d>
class col_ref {
    varmesh<d>& m;
    size_t ci;
public:
    col_ref(varmesh<d>& pm, size_t pci) : m{ pm }, ci{ pci }
    {

    }
    v<d>& operator[](const size_t ri)
    {
        return m.element(ri, ci);
    }
    size_t size()
    {
        return m.row_size();
    }
};

template <size_t d>
class row_ref {
    varmesh<d>& m;
    size_t ri;
public:
    row_ref(varmesh<d>& pm, size_t pri) : m{ pm }, ri{ pri }
    {

    }
    v<d>& operator[](const size_t ci)
    {
        return m.element(ri, ci);
    }
    size_t size()
    {
        return m.col_size();
    }
};

template <size_t d>
class row_ref_const {
    const varmesh<d>& m;
    size_t ri;
public:
    row_ref_const(const varmesh<d>& pm, size_t pri) : m{ pm }, ri{ pri }
    {

    }
    const v<d>& operator[](const size_t ci) const
    {
        return m.element(ri, ci);
    }
    size_t size() const
    {
        return m.col_size();
    }
};

template <size_t d>
class col_ref_const {
    const varmesh<d>& m;
    size_t ci;
public:
    col_ref_const(const varmesh<d>& pm, size_t pci) : m{ pm }, ci{ pci }
    {

    }
    const v<d>& operator[](const size_t ri) const
    {
        return m.element(ri, ci);
    }
    size_t size() const
    {
        return m.row_size();
    }
};

template<size_t d> varmesh<d> operator+(varmesh<d> mesh, v<d> translate)
{
    for (int i = 0; i < mesh.row_size(); i++)
        for (int j = 0; j < mesh.col_size(); j++)
            mesh.element(i, j) = mesh.element(i, j) + translate;

    return mesh;
}

varmesh<4> move(varmesh<4> mesh, v<3> translate);

varmesh<4> scale(varmesh<4> mesh, double scale);


#endif /* varmesh_h */
