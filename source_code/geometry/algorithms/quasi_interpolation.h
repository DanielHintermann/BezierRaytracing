#ifndef quasi_interpolation_hpp
#define quasi_interpolation_hpp

#include "intersection.h"

std::vector<std::vector<bool>> does_increased_mesh_contain_origin(const varmesh<2>& mesh, double offset);


std::vector<v2> bezier_quasi_interpolation_clipping(const varmesh<2> &points, double epsilon);


std::vector<v2> get_intersections_quasi(const v3& origin, const v3& direction, const varmesh<4>& m, double epsilon);


#endif