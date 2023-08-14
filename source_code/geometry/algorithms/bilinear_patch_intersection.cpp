#include "bilinear_patch_intersection.h"

#include <math.h>
#include <iostream>

class ostream;
class istream;

class Vector {
    double d[3];

public:

    inline Vector(double x, double y, double z = 0);
    inline Vector(const Vector& v);
    inline Vector();
    inline double length() const;
    inline double length2() const;
    inline Vector& operator=(const Vector& v);
    inline bool operator==(const Vector& v) const;
    inline Vector operator*(double s) const;
    inline Vector operator*(const Vector& v) const;
    inline Vector operator/(const Vector& v) const;
    inline Vector& operator*=(double s);
    Vector operator/(double s) const;
    inline Vector operator+(const Vector& v) const;
    inline Vector& operator+=(const Vector& v);
    inline Vector operator-() const;
    inline Vector operator-(const Vector& v) const;
    Vector& operator-=(const Vector& v);
    inline double normalize();
    Vector normal() const;
    inline Vector cross(const Vector& v) const;
    inline double dot(const Vector& v) const;
    inline void x(double xx) { d[0] = xx; }
    inline double x() const;
    inline void y(double yy) { d[1] = yy; }
    inline double y() const;
    inline void z(double zz) { d[2] = zz; }
    inline double z() const;
    inline double minComponent() const;
    friend ostream& operator<<(ostream& os, const Vector& p);
    friend istream& operator>>(istream& os, Vector& p);
    inline bool operator != (const Vector& v) const;
    inline double* ptr() const { return (double*)&d[0]; }

    void make_ortho(Vector& v1, Vector& v2)
    {
        Vector v0(this->cross(Vector(1, 0, 0)));
        if (v0.length2() == 0) {
            v0 = this->cross(this->cross(Vector(0, 1, 0)));
        }
        v1 = this->cross(v0);
        v1.normalize();
        v2 = this->cross(v1);
        v2.normalize();
    }

};


inline Vector::Vector(double x, double y, double z) {
    d[0] = x;
    d[1] = y;
    d[2] = z;
}

inline Vector::Vector(const Vector& v) {
    d[0] = v.d[0];
    d[1] = v.d[1];
    d[2] = v.d[2];
}

inline Vector::Vector() {
}


inline double Vector::length() const {
    return sqrt(length2());
}

inline double Vector::length2() const {
    return d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
}

inline Vector& Vector::operator=(const Vector& v) {
    d[0] = v.d[0];
    d[1] = v.d[1];
    d[2] = v.d[2];
    return *this;
}



inline Vector Vector::operator*(double s) const {
    return Vector(d[0] * s, d[1] * s, d[2] * s);
}

inline Vector operator*(double s, const Vector& v) {
    return v * s;
}

inline Vector Vector::operator*(const Vector& v) const {
    return Vector(d[0] * v.d[0], d[1] * v.d[1], d[2] * v.d[2]);
}

inline Vector Vector::operator/(const Vector& v) const {
    return Vector(d[0] / v.d[0], d[1] / v.d[1], d[2] / v.d[2]);
}

inline Vector Vector::operator+(const Vector& v) const {
    return Vector(d[0] + v.d[0], d[1] + v.d[1], d[2] + v.d[2]);
}

inline Vector& Vector::operator+=(const Vector& v) {
    d[0] += v.d[0];
    d[1] += v.d[1];
    d[2] += v.d[2];
    return *this;
}

inline Vector& Vector::operator*=(double s) {
    d[0] *= s;
    d[1] *= s;
    d[2] *= s;
    return *this;
}

inline Vector Vector::operator-() const {
    return Vector(-d[0], -d[1], -d[2]);
}

inline Vector Vector::operator-(const Vector& v) const {
    return Vector(d[0] - v.d[0], d[1] - v.d[1], d[2] - v.d[2]);
}


inline double Vector::normalize() {
    double l = length();
    if (l != 0)
    {
        d[0] /= l;
        d[1] /= l;
        d[2] /= l;
    }
    return l;
}

inline Vector Vector::cross(const Vector& v) const {
    return Vector(d[1] * v.d[2] - d[2] * v.d[1],
        d[2] * v.d[0] - d[0] * v.d[2],
        d[0] * v.d[1] - d[1] * v.d[0]);
}

inline double Vector::dot(const Vector& v) const {
    return d[0] * v.d[0] + d[1] * v.d[1] + d[2] * v.d[2];
}


inline double Vector::x() const {
    return d[0];
}

inline double Vector::y() const {
    return d[1];
}

inline double Vector::z() const {
    return d[2];
}

inline double Vector::minComponent() const {
    return (d[0] < d[1] && d[0] < d[2]) ? d[0] : d[1] < d[2] ? d[1] : d[2];
}

inline bool Vector::operator != (const Vector& v) const {
    return d[0] != v.d[0] || d[1] != v.d[1] || d[2] != v.d[2];
}

inline bool Vector::operator == (const Vector& v) const {
    return d[0] == v.d[0] && d[1] == v.d[1] && d[2] == v.d[2];
}

#define ray_epsilon 1e-12 // some small epsilon for flt pt
//#define twoplanes true // comment out this line to use raypatch 
#ifndef twoplanes // if we're not using patch-twoplanes intersection
#define raypatch true //then use ray-patch intersections
#endif




//find roots of ax^2+bx+c=0  in the interval min,max.
// place the roots in u[2] and return how many roots found
int QuadraticRoot(double a, double b, double c,
    double min, double max, double* u);

// Bilinear patch class
class BilinearPatch
{
    // The four points defining the patch
    Vector P00, P01, P10, P11;

public:

    // Constructors
    BilinearPatch(Vector Pt00, Vector Pt01, Vector Pt10, Vector Pt11);
    // Destructor
    ~BilinearPatch() {}
    Vector getP00() { return P00; }
    // Return the point P01
    Vector getP01() { return P01; }
    // Return the point P10
    Vector getP10() { return P10; }
    // Return the point P11
    Vector getP11() { return P11; }
    // Find the tangent (du)
    Vector TanU(double v);
    // Find the tangent (dv)
    Vector TanV(double u);
    // Find dudv
    Vector Normal(double u, double v);
    // Evaluate the surface of the patch at u,v
    Vector SrfEval(double u, double v);
    // Find the local closest point to spacept
    std::vector<v2> RayPatchIntersection(Vector r, Vector d);
};

BilinearPatch::BilinearPatch(Vector Pt00, Vector Pt01, Vector Pt10, Vector Pt11)
{
    P00 = Pt00;
    P01 = Pt01;
    P10 = Pt10;
    P11 = Pt11;
}

// What is the x,y,z position of a point at params u and v?
Vector BilinearPatch::SrfEval(double u, double v)
{
    Vector respt;
    respt.x(((1.0 - u) * (1.0 - v) * P00.x() +
        (1.0 - u) * v * P01.x() +
        u * (1.0 - v) * P10.x() +
        u * v * P11.x()));
    respt.y(((1.0 - u) * (1.0 - v) * P00.y() +
        (1.0 - u) * v * P01.y() +
        u * (1.0 - v) * P10.y() +
        u * v * P11.y()));
    respt.z(((1.0 - u) * (1.0 - v) * P00.z() +
        (1.0 - u) * v * P01.z() +
        u * (1.0 - v) * P10.z() +
        u * v * P11.z()));
    return respt;
}

//+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+
// Find tangent (du)
//+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+
Vector BilinearPatch::TanU(double v)
{
    Vector tanu;
    tanu.x((1.0 - v) * (P10.x() - P00.x()) + v * (P11.x() - P01.x()));
    tanu.y((1.0 - v) * (P10.y() - P00.y()) + v * (P11.y() - P01.y()));
    tanu.z((1.0 - v) * (P10.z() - P00.z()) + v * (P11.z() - P01.z()));
    return tanu;
}

//+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+
// Find tanget (dv)
//+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+
Vector BilinearPatch::TanV(double u)
{
    Vector tanv;
    tanv.x((1.0 - u) * (P01.x() - P00.x()) + u * (P11.x() - P10.x()));
    tanv.y((1.0 - u) * (P01.y() - P00.y()) + u * (P11.y() - P10.y()));
    tanv.z((1.0 - u) * (P01.z() - P00.z()) + u * (P11.z() - P10.z()));
    return tanv;
}


//+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+
// Find the normal of the patch
//+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+
Vector BilinearPatch::Normal(double u, double v)
{
    Vector tanu, tanv;
    tanu = TanU(v);
    tanv = TanV(u);
    return tanu.cross(tanv);
}

//choose between the best denominator to avoid singularities
//and to get the most accurate root possible
inline double getu(double v, double M1, double M2, double J1, double J2,
    double K1, double K2, double R1, double R2)
{

    double denom = (v * (M1 - M2) + J1 - J2);
    double d2 = (v * M1 + J1);
    if (fabs(denom) > fabs(d2)) // which denominator is bigger
    {
        return (v * (K2 - K1) + R2 - R1) / denom;
    }
    return -(v * K1 + R1) / d2;
}

// compute t with the best accuracy by using the component
// of the direction that is largest
double computet(Vector dir, Vector orig, Vector srfpos)
{
    // if x is bigger than y and z
    if (fabs(dir.x()) >= fabs(dir.y()) && fabs(dir.x()) >= fabs(dir.z()))
        return (srfpos.x() - orig.x()) / dir.x();
    // if y is bigger than x and z
    else if (fabs(dir.y()) >= fabs(dir.z())) // && fabs(dir.y()) >= fabs(dir.x()))
        return (srfpos.y() - orig.y()) / dir.y();
    // otherwise x isn't bigger than both and y isn't bigger than both
    else  //if(fabs(dir.z()) >= fabs(dir.x()) && fabs(dir.z()) >= fabs(dir.y()))
        return (srfpos.z() - orig.z()) / dir.z();
}



//+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+
//             RayPatchIntersection
// intersect rays of the form p = r + t q where t is the parameter
// to solve for. With the patch pointed to by *this
// for valid intersections:
// place the u,v intersection point in uv[0] and uv[1] respectively.
// place the t value in uv[2]
// return true to this function
// for invalid intersections - simply return false uv values can be 
// anything
//+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+
std::vector<v2> BilinearPatch::RayPatchIntersection(Vector r, Vector q)
{
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Equation of the patch:
    // P(u, v) = (1-u)(1-v)P00 + (1-u)vP01 + u(1-v)P10 + uvP11
    // Equation of the ray:
    // R(t) = r + tq
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Vector pos1, pos2; //Vector pos = ro + t*rd;
    int num_sol; // number of solutions to the quadratic
    double vsol[2]; // the two roots from quadraticroot
    double t2, u; // the t values of the two roots

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Variables for substitition
    // a = P11 - P10 - P01 + P00
    // b = P10 - P00
    // c = P01 - P00
    // d = P00  (d is shown below in the #ifdef raypatch area)
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~  

    // Find a w.r.t. x, y, z
    double ax = P11.x() - P10.x() - P01.x() + P00.x();
    double ay = P11.y() - P10.y() - P01.y() + P00.y();
    double az = P11.z() - P10.z() - P01.z() + P00.z();


    // Find b w.r.t. x, y, z
    double bx = P10.x() - P00.x();
    double by = P10.y() - P00.y();
    double bz = P10.z() - P00.z();

    // Find c w.r.t. x, y, z
    double cx = P01.x() - P00.x();
    double cy = P01.y() - P00.y();
    double cz = P01.z() - P00.z();


    double rx = r.x();
    double ry = r.y();
    double rz = r.z();

    // Retrieve the xyz of the q part of ray
    double qx = q.x();
    double qy = q.y();
    double qz = q.z();

    std::vector<v2> intersections;

#ifndef twoplanes
    {
        

        Vector p1n, p2n, dir(qx, qy, qz), orig(rx, ry, rz);
        dir.normalize();
        dir.make_ortho(p1n, p2n);

        double D1 = (p1n.x() * rx + p1n.y() * ry + p1n.z() * rz);
        double D2 = (p2n.x() * rx + p2n.y() * ry + p2n.z() * rz);

        Vector a(ax, ay, az);
        Vector b(bx, by, bz);
        Vector c(cx, cy, cz);
        Vector d(P00.x(), P00.y(), P00.z());


        double M1 = p1n.dot(a);
        double M2 = p2n.dot(a);
        double J1 = p1n.dot(b);
        double J2 = p2n.dot(b);
        double K1 = p1n.dot(c);
        double K2 = p2n.dot(c);
        double R1 = p1n.dot(d) - D1;
        double R2 = p2n.dot(d) - D2;

        double A = M1 * K2 - M2 * K1;
        double B = M1 * R2 - M2 * R1 - J2 * K1 + J1 * K2;
        double C = J1 * R2 - R1 * J2;

        Vector uv;
        uv.x(-2); uv.y(-2); uv.z(-2);
        num_sol = QuadraticRoot(A, B, C, -ray_epsilon, 1 + ray_epsilon, vsol);
        switch (num_sol)
        {
        case 0:
            //return false; // no solutions found
            break;
        case 1:
            uv.y(vsol[0]); //the v value
            uv.x(getu(vsol[0], M1, M2, J1, J2, K1, K2, R1, R2));
            if (uv.x() < 1 + ray_epsilon && uv.x() > -ray_epsilon) // u is valid
            {
                pos1 = SrfEval(uv.x(), uv.y());
                uv.z(computet(dir, orig, pos1));
                if (uv.z() > 0) //t is valid
                    intersections.push_back({ uv.x(), uv.y() });
                
            }
            //return false; // no other soln - so ret false
            break;
        case 2: // two solutions found
            uv.x(getu(vsol[0], M1, M2, J1, J2, K1, K2, R1, R2));
            uv.y(vsol[0]);
            pos1 = SrfEval(uv.x(), uv.y());
            uv.z(computet(dir, orig, pos1));
            if (uv.x() < 1 + ray_epsilon && uv.x() > -ray_epsilon && uv.z() > 0)//valid vars?
            {
                intersections.push_back({ uv.x(), uv.y() });
            }
            uv.y(vsol[1]);
            uv.x(getu(vsol[1], M1, M2, J1, J2, K1, K2, R1, R2));
            pos1 = SrfEval(uv.x(), uv.y());
            uv.z(computet(dir, orig, pos1));
            if (uv.x() < 1 + ray_epsilon && uv.x() > -ray_epsilon && uv.z() > 0) // variables are okay?
            {
                intersections.push_back({ uv.x(), uv.y() });
            }
                
            break;
        } //end 2 root case.
    }
#endif // end two planes 
#ifndef raypatch

    // Find d w.r.t. x, y, z - subtracting r just after  
    double dx = P00.x() - r.x();
    double dy = P00.y() - r.y();
    double dz = P00.z() - r.z();


    // Find A1 and A2
    double A1 = ax * qz - az * qx;
    double A2 = ay * qz - az * qy;

    // Find B1 and B2
    double B1 = bx * qz - bz * qx;
    double B2 = by * qz - bz * qy;

    // Find C1 and C2
    double C1 = cx * qz - cz * qx;
    double C2 = cy * qz - cz * qy;

    // Find D1 and D2
    double D1 = dx * qz - dz * qx;
    double D2 = dy * qz - dz * qy;

    Vector dir(qx, qy, qz), orig(rx, ry, rz);
    double A = A2 * C1 - A1 * C2;
    double B = A2 * D1 - A1 * D2 + B2 * C1 - B1 * C2;
    double C = B2 * D1 - B1 * D2;

    Vector uv;

    uv.x(-2); uv.y(-2); uv.z(-2);

    num_sol = QuadraticRoot(A, B, C, -ray_epsilon, 1 + ray_epsilon, vsol);

    switch (num_sol)
    {
    case 0:
        return {}; // no solutions found
    case 1:
        uv.y(vsol[0]);
        uv.x(getu(uv.y(), A2, A1, B2, B1, C2, C1, D2, D1));
        pos1 = SrfEval(uv.x(), uv.y());
        uv.z(computet(dir, orig, pos1));
        if (uv.x() < 1 + ray_epsilon && uv.x() > -ray_epsilon && uv.z() > 0)//vars okay?
        {
            intersections.push_back({ uv.x(), uv.y() });
        }
        break;
    case 2: // two solutions found
        uv.y(vsol[0]);
        uv.x(getu(uv.y(), A2, A1, B2, B1, C2, C1, D2, D1));
        pos1 = SrfEval(uv.x(), uv.y());
        uv.z(computet(dir, orig, pos1));
        if (uv.x() < 1 + ray_epsilon && uv.x() > -ray_epsilon && uv.z() > 0)
        {
            intersections.push_back({ uv.x(), uv.y() });
        }
        uv.y(vsol[1]);
        uv.x(getu(vsol[1], A2, A1, B2, B1, C2, C1, D2, D1));
        pos1 = SrfEval(uv.x(), uv.y());
        uv.z(computet(dir, orig, pos1));
        if (uv.x() < 1 + ray_epsilon && uv.x() > -ray_epsilon && uv.z() > 0)
        {
            intersections.push_back({ uv.x(), uv.y() });
        }
        break;
    }
#endif    // end ray patch
    
    return intersections; 
}


// a x ^2 + b x + c = 0
// in this case, the root must be between min and max
// it returns the # of solutions found
// x = [ -b +/- sqrt(b*b - 4 *a*c) ] / 2a
// or x = 2c / [-b +/- sqrt(b*b-4*a*c)]
int QuadraticRoot(double a, double b, double c,
    double min, double max, double* u)
{
    u[0] = u[1] = min - min; // make it lower than min
    if (a == 0.0) // then its close to 0
    {
        if (b != 0.0) // not close to 0
        {
            u[0] = -c / b;
            if (u[0] > min && u[0] < max) //its in the interval
                return 1; //1 soln found
            else  //its not in the interval
                return 0;
        }
        else
            return 0;
    }
    double d = b * b - 4 * a * c; //discriminant
    if (d <= 0.0) // single or no root
    {
        if (d == 0.0) // close to 0
        {
            u[0] = -b / a;
            if (u[0] > min && u[0] < max) // its in the interval
                return 1;
            else //its not in the interval
                return 0;
        }

        else // no root d must be below 0
            return 0;
    }

    double q = -0.5 * (b + copysign(sqrt(d), b));
    u[0] = c / q;
    u[1] = q / a;

    if ((u[0] > min && u[0] < max)
        && (u[1] > min && u[1] < max))
        return 2;
    else if (u[0] > min && u[0] < max) //then one wasn't in interval
        return 1;
    else if (u[1] > min && u[1] < max)
    {  // make it easier, make u[0] be the valid one always
        double dummy;
        dummy = u[0];
        u[0] = u[1];
        u[1] = dummy; // just in case somebody wants to check it
        return 1;
    }
    return 0;

}

std::vector<v2> bilinear_patch_roots(const varmesh<2>& mesh, double epsilon)
{
    Vector P00(mesh.element(0, 0)[0], mesh.element(0, 0)[1], 0);
    Vector P01(mesh.element(0, 1)[0], mesh.element(0, 1)[1], 0);
    Vector P10(mesh.element(1, 0)[0], mesh.element(1, 0)[1], 0);
    Vector P11(mesh.element(1, 1)[0], mesh.element(1, 1)[1], 0);
    // make them into a bilinear patch
    BilinearPatch bp(P00, P01, P10, P11);
    //you have some ray information
    Vector r(0, 0, -1000); //origin of the ray
    Vector q(0, 0, 1); // a ray direction
    q.normalize();
    Vector uv; // variables returned
    auto intersections = bp.RayPatchIntersection(r, q);

    //for (auto i : intersections)
    //{
    //    auto y = bp.SrfEval(i[0], i[1]);
    //    std::cout << "\n " << y.x() << ", " << y.y() << ", " << y.z();
   // }

    return intersections;
}