#include "bezier.h"
#include <geometry/algorithms/intersection.h>
#include <algorithm>
#include "bezier_surface.h"

double bezier_nairn_coefficient(int degree)
{
    return floor((double)degree/2)*ceil((double)degree/2)/(2*degree);
}

double bezier_z_coefficient(int degree)
{
    assert(0 < degree);
    
    if (1 == degree) return 0;
    
    if (2 == degree) return 1./16;
    
    return bezier_nairn_coefficient(degree) - 0.25;
}

std::vector<v2> solve_linear_form(const v2 &y, const varmesh<2> &m, double epsilon)
{
    auto solution_v = solve_linear_form(y, m.element(0, 0), m.element(1, 0), m.element(0, 1), m.element(1, 1));
    
    auto solution_u = solve_linear_form(y, m.element(0, 0), m.element(0, 1), m.element(1, 0), m.element(1, 1));
    
    std::vector<v2> solutions;
    
    for (int i = 0; i < solution_u.size(); i++)
    {
        for (int j = 0; j < solution_v.size(); j++)
        {
            v2 ey = evaluate_bezier_surface(m, solution_u[i], solution_v[j]);
            
            auto diff = l_inf(y - ey);

            if (diff <= epsilon)
            {
                solutions.push_back(v2{{solution_u[i], solution_v[j]}});
            }
        }
    }
    
    return solutions;
}



