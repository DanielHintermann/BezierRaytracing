#ifndef BILINEAR_H
#define BILINEAR_H 

#include<geometry/types/varmesh.h>

std::vector<v2> bilinear_patch_roots(const varmesh<2>& mesh, double epsilon);

#endif