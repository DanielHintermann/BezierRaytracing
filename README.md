# Intersection of a ray and a rational Bezier patch with quasi-interpolating polygons

This work dates back to 2015, when I wanted to experiment with new C++11/C++14 features and raytracing of rational Bezier patches on multicore CPUs.
As NURBS, the backbone in many CAD systems, are piecewise rational Bezier patches, being able to intersect rational Bezier patches and rays can be the foundation for raytracing NURBS.

The basic ideas for the intersection algorithm in this work is based on the two following papers:
* Fougerolle, Yohan & Lanquetin, Sandrine & Neveu, Marc & Lauthelier, Thierry. (2009). A Geometric Algorithm for Ray/Bézier Surfaces Intersection Using Quasi-Interpolating Control Net. SITIS 2008 - Proceedings of the 4th International Conference on Signal Image Technology and Internet Based Systems. 451 - 457. 10.1109/SITIS.2008.24. https://www.researchgate.net/publication/224363114_A_Geometric_Algorithm_for_RayBezier_Surfaces_Intersection_Using_Quasi-Interpolating_Control_Net
* Alexander Efremov, Vlastimil Havran, and Hans-Peter Seidel. 2005. Robust and numerically stable Bézier clipping method for ray tracing NURBS surfaces. In Proceedings of the 21st Spring Conference on Computer Graphics (SCCG '05). Association for Computing Machinery, New York, NY, USA, 127–135. https://doi.org/10.1145/1090122.1090144

Brief outline of the intersection algorithm:
* A ray can be represented as intersection of two planes (a vertical and horizontal plane)
* This leads to finding the roots of a 2D (non-rational) Bezier patch (see the paper of Efremov)
* According to the paper of Fougerolle, the 2D Bezier patch can be approximated with a quasi-interpolating polygon (there is a formular to tell the maximum difference between the Bezier patch and the quasi-interpolating polygon)
* If D is this maximum difference, then instead of intersecting the D-envelope of the quasi-interpolation polygon and the point (0, 0), the quasi-interpolating polygon is intersected with a circle of diameter D.
* This allows clipping until the areas potentially containing the roots are small enough

The code should build and run under Linux, Windows and MacOS. It's using cmake to build and the google test framework.
There is also a Dockerfile for a Ubuntu Linux build and a batch file to build the Docker image and run the Docker container.
There are some regression tests (test_integration.cpp), which compare images from the current run with the previous run to track, if the algorithm accidentally changed during some refactoring work.

Currently, there is only a simple lighting model and a ray is only traced until the first intersection. Once I find time, I'll change that as it would be fairly straight forward to implement that.
The source code is using some C++20 features.

![a curved rational Bezier patch](/doc/test_curved_patch.png "a curved rational Bezier patch")
 
