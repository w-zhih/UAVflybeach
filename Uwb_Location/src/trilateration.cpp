// -------------------------------------------------------------------------------------------------------------------
//
//  File: trilateration.cpp
//
//  Copyright 2016 (c) Decawave Ltd, Dublin, Ireland.
//
//  All rights reserved.
//
//  Author:
//
// -------------------------------------------------------------------------------------------------------------------


#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "time.h"

#include "Uwb_Location/trilateration.h"
#include "iostream"
using namespace std;
/* Largest nonnegative number still considered zero */
#define   MAXZERO  0.001

#define		ERR_TRIL_CONCENTRIC						-1
#define		ERR_TRIL_COLINEAR_2SOLUTIONS			-2
#define		ERR_TRIL_SQRTNEGNUMB					-3
#define		ERR_TRIL_NOINTERSECTION_SPHERE4			-4
#define		ERR_TRIL_NEEDMORESPHERE					-5

#define CM_ERR_ADDED (10) //was 5

/* Return the difference of two vectors, (vector1 - vector2). */
vec3d vdiff(const vec3d vector1, const vec3d vector2)
{
    vec3d v;
    v.x = vector1.x - vector2.x;
    v.y = vector1.y - vector2.y;
    v.z = vector1.z - vector2.z;
    return v;
}

/* Return the sum of two vectors. */
vec3d vsum(const vec3d vector1, const vec3d vector2)
{
    vec3d v;
    v.x = vector1.x + vector2.x;
    v.y = vector1.y + vector2.y;
    v.z = vector1.z + vector2.z;
    return v;
}

/* Multiply vector by a number. */
vec3d vmul(const vec3d vector, const double n)
{
    vec3d v;
    v.x = vector.x * n;
    v.y = vector.y * n;
    v.z = vector.z * n;
    return v;
}

/* Divide vector by a number. */
vec3d vdiv(const vec3d vector, const double n)
{
    vec3d v;
    v.x = vector.x / n;
    v.y = vector.y / n;
    v.z = vector.z / n;
    return v;
}

/* Return the Euclidean norm. */
double vdist(const vec3d v1, const vec3d v2)
{
    double xd = v1.x - v2.x;
    double yd = v1.y - v2.y;
    double zd = v1.z - v2.z;
    return sqrt(xd * xd + yd * yd + zd * zd);
}

/* Return the Euclidean norm. */
double vnorm(const vec3d vector)
{
    return sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}

/* Return the dot product of two vectors. */
double dot(const vec3d vector1, const vec3d vector2)
{
    return vector1.x * vector2.x + vector1.y * vector2.y + vector1.z * vector2.z;
}

/* Replace vector with its cross product with another vector. */
vec3d cross(const vec3d vector1, const vec3d vector2)
{
    vec3d v;
    v.x = vector1.y * vector2.z - vector1.z * vector2.y;
    v.y = vector1.z * vector2.x - vector1.x * vector2.z;
    v.z = vector1.x * vector2.y - vector1.y * vector2.x;
    return v;
}

/* Return the GDOP (Geometric Dilution of Precision) rate between 0-1.
 * Lower GDOP rate means better precision of intersection.
 */
double gdoprate(const vec3d tag, const vec3d p1, const vec3d p2, const vec3d p3)
{
    vec3d ex, t1, t2, t3;
    double h, gdop1, gdop2, gdop3, result;

    ex = vdiff(p1, tag);
    h = vnorm(ex);
    t1 = vdiv(ex, h);

    ex = vdiff(p2, tag);
    h = vnorm(ex);
    t2 = vdiv(ex, h);

    ex = vdiff(p3, tag);
    h = vnorm(ex);
    t3 = vdiv(ex, h);

    gdop1 = fabs(dot(t1, t2));
    gdop2 = fabs(dot(t2, t3));
    gdop3 = fabs(dot(t3, t1));

    if (gdop1 < gdop2) result = gdop2; else result = gdop1;
    if (result < gdop3) result = gdop3;

    return result;
}

/* Intersecting a sphere sc with radius of r, with a line p1-p2.
 * Return zero if successful, negative error otherwise.
 * mu1 & mu2 are constant to find points of intersection.
*/
int sphereline(const vec3d p1, const vec3d p2, const vec3d sc, double r, double *const mu1, double *const mu2)
{
   double a,b,c;
   double bb4ac;
   vec3d dp;

   dp.x = p2.x - p1.x;
   dp.y = p2.y - p1.y;
   dp.z = p2.z - p1.z;

   a = dp.x * dp.x + dp.y * dp.y + dp.z * dp.z;

   b = 2 * (dp.x * (p1.x - sc.x) + dp.y * (p1.y - sc.y) + dp.z * (p1.z - sc.z));

   c = sc.x * sc.x + sc.y * sc.y + sc.z * sc.z;
   c += p1.x * p1.x + p1.y * p1.y + p1.z * p1.z;
   c -= 2 * (sc.x * p1.x + sc.y * p1.y + sc.z * p1.z);
   c -= r * r;

   bb4ac = b * b - 4 * a * c;

   if (fabs(a) == 0 || bb4ac < 0) {
      *mu1 = 0;
      *mu2 = 0;
      return -1;
   }

   *mu1 = (-b + sqrt(bb4ac)) / (2 * a);
   *mu2 = (-b - sqrt(bb4ac)) / (2 * a);

   return 0;
}

/* Return TRIL_3SPHERES if it is performed using 3 spheres and return
 * TRIL_4SPHERES if it is performed using 4 spheres
 * For TRIL_3SPHERES, there are two solutions: result1 and result2
 * For TRIL_4SPHERES, there is only one solution: best_solution
 *
 * Return negative number for other errors
 *
 * To force the function to work with only 3 spheres, provide a duplicate of
 * any sphere at any place among p1, p2, p3 or p4.
 *
 * The last parameter is the largest nonnegative number considered zero;
 * it is somewhat analogous to machine epsilon (but inclusive).
*/
int trilateration(vec3d *const result1,
                  vec3d *const result2,
                  vec3d *const best_solution,
                  const vec3d p1, const double r1,
                  const vec3d p2, const double r2,
                  const vec3d p3, const double r3,
                  const vec3d p4, const double r4,
                  const double maxzero)
{
    vec3d	ex, ey, ez, t1, t2, t3;
    double	h, i, j, x, y, z, t;
    double	mu1, mu2, mu;
    int result;

    /*********** FINDING TWO POINTS FROM THE FIRST THREE SPHERES **********/

    // if there are at least 2 concentric spheres within the first 3 spheres
    // then the calculation may not continue, drop it with error -1

    /* h = |p3 - p1|, ex = (p3 - p1) / |p3 - p1| */
    ex = vdiff(p3, p1); // vector p13
    h = vnorm(ex); // scalar p13
    if (h <= maxzero) {
        /* p1 and p3 are concentric, not good to obtain a precise intersection point */
        //printf("concentric13 return -1\n");
        return ERR_TRIL_CONCENTRIC;
    }

    /* h = |p3 - p2|, ex = (p3 - p2) / |p3 - p2| */
    ex = vdiff(p3, p2); // vector p23
    h = vnorm(ex); // scalar p23
    if (h <= maxzero) {
        /* p2 and p3 are concentric, not good to obtain a precise intersection point */
        //printf("concentric23 return -1\n");
        return ERR_TRIL_CONCENTRIC;
    }

    /* h = |p2 - p1|, ex = (p2 - p1) / |p2 - p1| */
    ex = vdiff(p2, p1); // vector p12
    h = vnorm(ex); // scalar p12
    if (h <= maxzero) {
        /* p1 and p2 are concentric, not good to obtain a precise intersection point */
        //printf("concentric12 return -1\n");
        return ERR_TRIL_CONCENTRIC;
    }
    ex = vdiv(ex, h); // unit vector ex with respect to p1 (new coordinate system)

    /* t1 = p3 - p1, t2 = ex (ex . (p3 - p1)) */
    t1 = vdiff(p3, p1); // vector p13
    i = dot(ex, t1); // the scalar of t1 on the ex direction
    t2 = vmul(ex, i); // colinear vector to p13 with the length of i

    /* ey = (t1 - t2), t = |t1 - t2| */
    ey = vdiff(t1, t2); // vector t21 perpendicular to t1
    t = vnorm(ey); // scalar t21
    if (t > maxzero) {
        /* ey = (t1 - t2) / |t1 - t2| */
        ey = vdiv(ey, t); // unit vector ey with respect to p1 (new coordinate system)

        /* j = ey . (p3 - p1) */
        j = dot(ey, t1); // scalar t1 on the ey direction
    } else
        j = 0.0;

    /* Note: t <= maxzero implies j = 0.0. */
    if (fabs(j) <= maxzero) {

        /* Is point p1 + (r1 along the axis) the intersection? */
        t2 = vsum(p1, vmul(ex, r1));
        if (fabs(vnorm(vdiff(p2, t2)) - r2) <= maxzero &&
            fabs(vnorm(vdiff(p3, t2)) - r3) <= maxzero) {
            /* Yes, t2 is the only intersection point. */
            if (result1)
                *result1 = t2;
            if (result2)
                *result2 = t2;
            return TRIL_3SPHERES;
        }

        /* Is point p1 - (r1 along the axis) the intersection? */
        t2 = vsum(p1, vmul(ex, -r1));
        if (fabs(vnorm(vdiff(p2, t2)) - r2) <= maxzero &&
            fabs(vnorm(vdiff(p3, t2)) - r3) <= maxzero) {
            /* Yes, t2 is the only intersection point. */
            if (result1)
                *result1 = t2;
            if (result2)
                *result2 = t2;
            return TRIL_3SPHERES;
        }
        /* p1, p2 and p3 are colinear with more than one solution */
        return ERR_TRIL_COLINEAR_2SOLUTIONS;
    }

    /* ez = ex x ey */
    ez = cross(ex, ey); // unit vector ez with respect to p1 (new coordinate system)

    x = (r1*r1 - r2*r2) / (2*h) + h / 2;
    y = (r1*r1 - r3*r3 + i*i) / (2*j) + j / 2 - x * i / j;
    z = r1*r1 - x*x - y*y;
    if (z < -maxzero) {
        /* The solution is invalid, square root of negative number */
        return ERR_TRIL_SQRTNEGNUMB;
    } else
    if (z > 0.0)
        z = sqrt(z);
    else
        z = 0.0;

    /* t2 = p1 + x ex + y ey */
    t2 = vsum(p1, vmul(ex, x));
    t2 = vsum(t2, vmul(ey, y));

    /* result1 = p1 + x ex + y ey + z ez */
    if (result1)
        *result1 = vsum(t2, vmul(ez, z));

    /* result1 = p1 + x ex + y ey - z ez */
    if (result2)
        *result2 = vsum(t2, vmul(ez, -z));

    /*********** END OF FINDING TWO POINTS FROM THE FIRST THREE SPHERES **********/
    /********* RESULT1 AND RESULT2 ARE SOLUTIONS, OTHERWISE RETURN ERROR *********/


    /************* FINDING ONE SOLUTION BY INTRODUCING ONE MORE SPHERE ***********/

    // check for concentricness of sphere 4 to sphere 1, 2 and 3
    // if it is concentric to one of them, then sphere 4 cannot be used
    // to determine the best solution and return -1

    /* h = |p4 - p1|, ex = (p4 - p1) / |p4 - p1| */
    ex = vdiff(p4, p1); // vector p14
    h = vnorm(ex); // scalar p14
    if (h <= maxzero) {
        /* p1 and p4 are concentric, not good to obtain a precise intersection point */
        //printf("concentric14 return 0\n");
        return TRIL_3SPHERES;
    }
    /* h = |p4 - p2|, ex = (p4 - p2) / |p4 - p2| */
    ex = vdiff(p4, p2); // vector p24
    h = vnorm(ex); // scalar p24
    if (h <= maxzero) {
        /* p2 and p4 are concentric, not good to obtain a precise intersection point */
        //printf("concentric24 return 0\n");
        return TRIL_3SPHERES;
    }
    /* h = |p4 - p3|, ex = (p4 - p3) / |p4 - p3| */
    ex = vdiff(p4, p3); // vector p34
    h = vnorm(ex); // scalar p34
    if (h <= maxzero) {
        /* p3 and p4 are concentric, not good to obtain a precise intersection point */
        //printf("concentric34 return 0\n");
        return TRIL_3SPHERES;
    }

    // if sphere 4 is not concentric to any sphere, then best solution can be obtained
    /* find i as the distance of result1 to p4 */
    t3 = vdiff(*result1, p4);
    i = vnorm(t3);
    /* find h as the distance of result2 to p4 */
    t3 = vdiff(*result2, p4);
    h = vnorm(t3);

    /* pick the result1 as the nearest point to the center of sphere 4 */
    if (i > h) {
        *best_solution = *result1;
        *result1 = *result2;
        *result2 = *best_solution;
    }

    int count4 = 0;
    double rr4 = r4;
    result = 1;
    /* intersect result1-result2 vector with sphere 4 */
    while(result && count4 < 10)
    {
        result=sphereline(*result1, *result2, p4, rr4, &mu1, &mu2);
        rr4+=0.1;
        count4++;
    }

    if (result) 
    {
        /* No intersection between sphere 4 and the line with the gradient of result1-result2! */
        *best_solution = *result1; // result1 is the closer solution to sphere 4
        //return ERR_TRIL_NOINTERSECTION_SPHERE4;

    } 
    else 
    {
        if (mu1 < 0 && mu2 < 0) {

            /* if both mu1 and mu2 are less than 0 */
            /* result1-result2 line segment is outside sphere 4 with no intersection */
            if (fabs(mu1) <= fabs(mu2)) mu = mu1; else mu = mu2;
            /* h = |result2 - result1|, ex = (result2 - result1) / |result2 - result1| */
            ex = vdiff(*result2, *result1); // vector result1-result2
            h = vnorm(ex); // scalar result1-result2
            ex = vdiv(ex, h); // unit vector ex with respect to result1 (new coordinate system)
            /* 50-50 error correction for mu */
            mu = 0.5*mu;
            /* t2 points to the intersection */
            t2 = vmul(ex, mu*h);
            t2 = vsum(*result1, t2);
            /* the best solution = t2 */
            *best_solution = t2;

        } else if ((mu1 < 0 && mu2 > 1) || (mu2 < 0 && mu1 > 1)) {

            /* if mu1 is less than zero and mu2 is greater than 1, or the other way around */
            /* result1-result2 line segment is inside sphere 4 with no intersection */
            if (mu1 > mu2) mu = mu1; else mu = mu2;
            /* h = |result2 - result1|, ex = (result2 - result1) / |result2 - result1| */
            ex = vdiff(*result2, *result1); // vector result1-result2
            h = vnorm(ex); // scalar result1-result2
            ex = vdiv(ex, h); // unit vector ex with respect to result1 (new coordinate system)
            /* t2 points to the intersection */
            t2 = vmul(ex, mu*h);
            t2 = vsum(*result1, t2);
            /* vector t2-result2 with 50-50 error correction on the length of t3 */
            t3 = vmul(vdiff(*result2, t2),0.5);
            /* the best solution = t2 + t3 */
            *best_solution = vsum(t2, t3);

        } else if (((mu1 > 0 && mu1 < 1) && (mu2 < 0 || mu2 > 1))
                || ((mu2 > 0 && mu2 < 1) && (mu1 < 0 || mu1 > 1))) {

            /* if one mu is between 0 to 1 and the other is not */
            /* result1-result2 line segment intersects sphere 4 at one point */
            if (mu1 >= 0 && mu1 <= 1) mu = mu1; else mu = mu2;
            /* add or subtract with 0.5*mu to distribute error equally onto every sphere */
            if (mu <= 0.5) mu-=0.5*mu; else mu-=0.5*(1-mu);
            /* h = |result2 - result1|, ex = (result2 - result1) / |result2 - result1| */
            ex = vdiff(*result2, *result1); // vector result1-result2
            h = vnorm(ex); // scalar result1-result2
            ex = vdiv(ex, h); // unit vector ex with respect to result1 (new coordinate system)
            /* t2 points to the intersection */
            t2 = vmul(ex, mu*h);
            t2 = vsum(*result1, t2);
            /* the best solution = t2 */
            *best_solution = t2;

        } else if (mu1 == mu2) {

            /* if both mu1 and mu2 are between 0 and 1, and mu1 = mu2 */
            /* result1-result2 line segment is tangential to sphere 4 at one point */
            mu = mu1;
            /* add or subtract with 0.5*mu to distribute error equally onto every sphere */
            if (mu <= 0.25) mu-=0.5*mu;
            else if (mu <=0.5) mu-=0.5*(0.5-mu);
            else if (mu <=0.75) mu-=0.5*(mu-0.5);
            else mu-=0.5*(1-mu);
            /* h = |result2 - result1|, ex = (result2 - result1) / |result2 - result1| */
            ex = vdiff(*result2, *result1); // vector result1-result2
            h = vnorm(ex); // scalar result1-result2
            ex = vdiv(ex, h); // unit vector ex with respect to result1 (new coordinate system)
            /* t2 points to the intersection */
            t2 = vmul(ex, mu*h);
            t2 = vsum(*result1, t2);
            /* the best solution = t2 */
            *best_solution = t2;

        } else {

            /* if both mu1 and mu2 are between 0 and 1 */
            /* result1-result2 line segment intersects sphere 4 at two points */

            //return ERR_TRIL_NEEDMORESPHERE;

            mu = mu1 + mu2;
            /* h = |result2 - result1|, ex = (result2 - result1) / |result2 - result1| */
            ex = vdiff(*result2, *result1); // vector result1-result2
            h = vnorm(ex); // scalar result1-result2
            ex = vdiv(ex, h); // unit vector ex with respect to result1 (new coordinate system)
            /* 50-50 error correction for mu */
            mu = 0.5*mu;
            /* t2 points to the intersection */
            t2 = vmul(ex, mu*h);
            t2 = vsum(*result1, t2);
            /* the best solution = t2 */
            *best_solution = t2;

        }

    }

    return TRIL_4SPHERES;

    /******** END OF FINDING ONE SOLUTION BY INTRODUCING ONE MORE SPHERE *********/
}


/* This function calls trilateration to get the best solution.
 *
 * If any three spheres does not produce valid solution,
 * then each distance is increased to ensure intersection to happens.
 *
 * Return the selected trilateration mode between TRIL_3SPHERES or TRIL_4SPHERES
 * For TRIL_3SPHERES, there are two solutions: solution1 and solution2
 * For TRIL_4SPHERES, there is only one solution: best_solution
 *
 * nosolution_count = the number of failed attempt before intersection is found
 * by increasing the sphere diameter.
*/
int deca_3dlocate (	vec3d	*const solution1,
                    vec3d	*const solution2,
                    vec3d	*const best_solution,
                    int		*const nosolution_count,
                    double	*const best_3derror,
                    double	*const best_gdoprate,
                    vec3d p1, double r1,
                    vec3d p2, double r2,
                    vec3d p3, double r3,
                    vec3d p4, double r4,
                    int *combination)
{
    vec3d	o1, o2, solution, ptemp;
    vec3d	solution_compare1, solution_compare2;
    double	/*error_3dcompare1, error_3dcompare2,*/ rtemp;
    double	gdoprate_compare1, gdoprate_compare2;
    double	ovr_r1, ovr_r2, ovr_r3, ovr_r4;
    int		overlook_count, combination_counter;
    int		trilateration_errcounter, trilateration_mode34;
    int		success, concentric, result;

    trilateration_errcounter = 0;
    trilateration_mode34 = 0;
    combination_counter = 1; /* four spheres combination */
    *best_gdoprate = 1; /* put the worst gdoprate init */
    gdoprate_compare1 = 1; gdoprate_compare2 = 1;
    solution_compare1.x = 0; solution_compare1.y = 0; solution_compare1.z = 0;

    do {
        success = 0;
        concentric = 0;
        overlook_count = 0;
        ovr_r1 = r1; ovr_r2 = r2; ovr_r3 = r3; ovr_r4 = r4;

        do {

            result = trilateration(&o1, &o2, &solution, p1, ovr_r1, p2, ovr_r2, p3, ovr_r3, p4, ovr_r4, MAXZERO);

            switch (result)
            {
                case TRIL_3SPHERES: // 3 spheres are used to get the result
                    trilateration_mode34 = TRIL_3SPHERES;
                    success = 1;
                    break;

                case TRIL_4SPHERES: // 4 spheres are used to get the result
                    trilateration_mode34 = TRIL_4SPHERES;
                    success = 1;
                    break;

                case ERR_TRIL_CONCENTRIC:
                    concentric = 1;
                    break;

                default: // any other return value goes here
                    ovr_r1 += 0.10;
                    ovr_r2 += 0.10;
                    ovr_r3 += 0.10;
                    ovr_r4 += 0.10;
                    overlook_count++;
                    break;
            }

            //qDebug() << "while(!success)" << overlook_count << concentric << "result" << result;

        } while (!success && (overlook_count <= CM_ERR_ADDED) && !concentric);

        if (success)
        {
            switch (result)
            {
            case TRIL_3SPHERES:
                *solution1 = o1;
                *solution2 = o2;
                *nosolution_count = overlook_count;

                combination_counter = 0;
                break;

            case TRIL_4SPHERES:

                /* calculate the new gdop */
                gdoprate_compare1	= gdoprate(solution, p1, p2, p3);

                /* compare and swap with the better result */
                if (gdoprate_compare1 <= gdoprate_compare2)
                {

                    *solution1 = o1;
                    *solution2 = o2;
                    *best_solution	= solution;
                    *nosolution_count = overlook_count;
                    *best_3derror	= sqrt((vnorm(vdiff(solution, p1))-r1)*(vnorm(vdiff(solution, p1))-r1) +
                                        (vnorm(vdiff(solution, p2))-r2)*(vnorm(vdiff(solution, p2))-r2) +
                                        (vnorm(vdiff(solution, p3))-r3)*(vnorm(vdiff(solution, p3))-r3) +
                                        (vnorm(vdiff(solution, p4))-r4)*(vnorm(vdiff(solution, p4))-r4));
                    *best_gdoprate	= gdoprate_compare1;

                    /* save the previous result */
                    solution_compare2 = solution_compare1;
                    gdoprate_compare2 = gdoprate_compare1;
                    *combination = 5 - combination_counter;

                }

                ptemp = p1; p1 = p2; p2 = p3; p3 = p4; p4 = ptemp;
                rtemp = r1; r1 = r2; r2 = r3; r3 = r4; r4 = rtemp;
                combination_counter--;
                if(combination_counter<0)
                {
                    combination_counter=0;
                }
                break;

            default:
                break;
            }
        }
        else
        {
            trilateration_errcounter++;
            combination_counter--;
            if(combination_counter<0)
            {
               combination_counter=0;
            }
        }

    } while (combination_counter);

    // if it gives error for all 4 sphere combinations then no valid result is given
    // otherwise return the trilateration mode used
    if (trilateration_errcounter >= 4)
        return -1;
    else
        return trilateration_mode34;

}


int GetLocation(vec3d *best_solution, int use4thAnchor, vec3d* anchorArray, float *distanceArray)
{

    vec3d	o1, o2, p1, p2, p3, p4;
    double	r1 = 0, r2 = 0, r3 = 0, r4 = 0, best_3derror, best_gdoprate;
    int		result;
    int     error, combination;

    if (use4thAnchor == 0)//3基站定位
    {
        if(distanceArray[3]==0)//A3无效 A0 A1 A2有效，执行3基站定位
        {
            /* Anchors coordinate */
            p1.x = anchorArray[0].x;		p1.y = anchorArray[0].y;	p1.z = anchorArray[0].z;
            p2.x = anchorArray[1].x;		p2.y = anchorArray[1].y;	p2.z = anchorArray[1].z;
            p3.x = anchorArray[2].x;		p3.y = anchorArray[2].y;	p3.z = anchorArray[2].z;
            p4.x = p1.x;		            p4.y = p1.y;	            p4.z = p1.z;

            r1 = (double) distanceArray[0] / 1000.0;
            r2 = (double) distanceArray[1] / 1000.0;
            r3 = (double) distanceArray[2] / 1000.0;
            r4 = r1;
        }
        else if(distanceArray[0]==0)//A0无效 A1 A2 A3有效，执行3基站定位
        {
            /* Anchors coordinate */
            p1.x = anchorArray[1].x;		p1.y = anchorArray[1].y;	p1.z = anchorArray[1].z;
            p2.x = anchorArray[2].x;		p2.y = anchorArray[2].y;	p2.z = anchorArray[2].z;
            p3.x = anchorArray[3].x;		p3.y = anchorArray[3].y;	p3.z = anchorArray[3].z;
            p4.x = p1.x;		            p4.y = p1.y;	            p4.z = p1.z;

            r1 = (double) distanceArray[1] / 1000.0;
            r2 = (double) distanceArray[2] / 1000.0;
            r3 = (double) distanceArray[3] / 1000.0;
            r4 = r1;
        }
        else if(distanceArray[1]==0)//A1无效 A0 A2 A3有效，执行3基站定位
        {
            /* Anchors coordinate */
            p1.x = anchorArray[0].x;		p1.y = anchorArray[0].y;	p1.z = anchorArray[0].z;
            p2.x = anchorArray[2].x;		p2.y = anchorArray[2].y;	p2.z = anchorArray[2].z;
            p3.x = anchorArray[3].x;		p3.y = anchorArray[3].y;	p3.z = anchorArray[3].z;
            p4.x = p1.x;		            p4.y = p1.y;	            p4.z = p1.z;

            r1 = (double) distanceArray[0] / 1000.0;
            r2 = (double) distanceArray[2] / 1000.0;
            r3 = (double) distanceArray[3] / 1000.0;
            r4 = r1;
        }
        else if(distanceArray[2]==0)//A2无效 A0 A1 A3有效，执行3基站定位
        {
            /* Anchors coordinate */
            p1.x = anchorArray[0].x;		p1.y = anchorArray[0].y;	p1.z = anchorArray[0].z;
            p2.x = anchorArray[1].x;		p2.y = anchorArray[1].y;	p2.z = anchorArray[1].z;
            p3.x = anchorArray[3].x;		p3.y = anchorArray[3].y;	p3.z = anchorArray[3].z;
            p4.x = p1.x;		            p4.y = p1.y;	            p4.z = p1.z;

            r1 = (double) distanceArray[0] / 1000.0;
            r2 = (double) distanceArray[1] / 1000.0;
            r3 = (double) distanceArray[3] / 1000.0;
            r4 = r1;
        }
    }
    else//4基站定位
    {
        /* Anchors coordinate */
        p1.x = anchorArray[0].x;		p1.y = anchorArray[0].y;	p1.z = anchorArray[0].z;
        p2.x = anchorArray[1].x;		p2.y = anchorArray[1].y;	p2.z = anchorArray[1].z;
        p3.x = anchorArray[2].x;		p3.y = anchorArray[2].y;	p3.z = anchorArray[2].z;
        p4.x = anchorArray[3].x;		p4.y = anchorArray[3].y;	p4.z = anchorArray[3].z;


        r1 = (double) distanceArray[0] / 1000.0;
        r2 = (double) distanceArray[1] / 1000.0;
        r3 = (double) distanceArray[2] / 1000.0;
        r4 = (double) distanceArray[3] / 1000.0;

    }
    

    result = deca_3dlocate (&o1, &o2, best_solution, &error, &best_3derror, &best_gdoprate,
                            p1, r1, p2, r2, p3, r3, p4, r4, &combination);

    if (result >= 0)
    {
        if(o1.z <= o2.z) best_solution->z = o1.z; else best_solution->z = o2.z;
        if (use4thAnchor == 0 || result == TRIL_3SPHERES)
        {
            if(o1.z < p1.z) *best_solution = o1; else *best_solution = o2; //assume tag is below the anchors (1, 2, and 3)
        }

        return result;

    }

    return -1;
}

//加速度和角速度结算四元数
Quaternion GetAngle(const ImuData_t *pMpu, float dt) 
{		
	volatile struct V
	{
		float x;
		float y;
		float z;
	} Gravity,Acc,Gyro,AccGravity;

	static struct V GyroIntegError = {0};
	static  float KpDef = 0.8f ;
	static  float KiDef = 0.0003f;
	Quaternion NumQ;  // 四元素
    NumQ.q0=1;
    NumQ.q1=0;
    NumQ.q2=0;
    NumQ.q3=0;
	float q0_t,q1_t,q2_t,q3_t;
	float NormQuat; 
	float HalfTime = dt * 0.5f;  
	// 提取等效旋转矩阵中的重力分量 
	Gravity.x = 2*(NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);								
	Gravity.y = 2*(NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);						  
	Gravity.z = 1-2*(NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);	
	
	// 加速度归一化
    NormQuat = Q_rsqrt(squa(pMpu->accX)+ squa(pMpu->accY) +squa(pMpu->accZ));
        
    Acc.x = pMpu->accX * NormQuat;
    Acc.y = pMpu->accY * NormQuat;
    Acc.z = pMpu->accZ * NormQuat;	
	
 	//向量叉乘得出的值
	AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
	AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
	AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);
	
	//再做加速度积分补偿角速度的补偿值
    GyroIntegError.x += AccGravity.x * KiDef;
    GyroIntegError.y += AccGravity.y * KiDef;
    GyroIntegError.z += AccGravity.z * KiDef;
        
        //角速度融合加速度积分补偿值
    Gyro.x = pMpu->gyroX * Gyro_Gr + KpDef * AccGravity.x  +  GyroIntegError.x;//弧度制
    Gyro.y = pMpu->gyroY * Gyro_Gr + KpDef * AccGravity.y  +  GyroIntegError.y;
    Gyro.z = pMpu->gyroZ * Gyro_Gr + KpDef * AccGravity.z  +  GyroIntegError.z;		
	
	// 一阶龙格库塔法, 更新四元数
	q0_t = (-NumQ.q1*Gyro.x - NumQ.q2*Gyro.y - NumQ.q3*Gyro.z) * HalfTime;
	q1_t = ( NumQ.q0*Gyro.x - NumQ.q3*Gyro.y + NumQ.q2*Gyro.z) * HalfTime;
	q2_t = ( NumQ.q3*Gyro.x + NumQ.q0*Gyro.y - NumQ.q1*Gyro.z) * HalfTime;
	q3_t = (-NumQ.q2*Gyro.x + NumQ.q1*Gyro.y + NumQ.q0*Gyro.z) * HalfTime;
	
	NumQ.q0 += q0_t;
	NumQ.q1 += q1_t;
	NumQ.q2 += q2_t;
	NumQ.q3 += q3_t;
	// 四元数归一化
	NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
	NumQ.q0 *= NormQuat;
	NumQ.q1 *= NormQuat;
	NumQ.q2 *= NormQuat;
	NumQ.q3 *= NormQuat;
    // cout<<NumQ.q0<<endl;
    // cout<<NumQ.q1<<endl;
    // cout<<NumQ.q2<<endl;
    // cout<<NumQ.q3<<endl;
    return NumQ;
}
float Q_rsqrt(float number)
{
	long i;
	float x2, y;
	const float threehalfs = 1.5F;
 
	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;                      
	i  = 0x5f3759df - ( i >> 1 );               
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );
	return y;
} 




// int main()
// {
// 	int result = 0; 
// 	vec3d anchorArray[4];
// 	vec3d report;
// 	int Range_deca[4];
	
// 	//A0
// 	anchorArray[0].x = 0.0; //anchor0.x uint:m
// 	anchorArray[0].y = 0.0; //anchor0.y uint:m
// 	anchorArray[0].z = 2.0; //anchor0.z uint:m

// 	//A1
// 	anchorArray[1].x = -4.28; //anchor2.x uint:m
// 	anchorArray[1].y = -14.74; //anchor2.y uint:m
// 	anchorArray[1].z = 2; //anchor2.z uint:m

// 	//A2
// 	anchorArray[2].x = 26.26; //anchor2.x uint:m
// 	anchorArray[2].y = -15.10; //anchor2.y uint:m
// 	anchorArray[2].z = 2; //anchor2.z uint:m

// 	//A3
// 	anchorArray[3].x = 25.44; //anchor2.x uint:m
// 	anchorArray[3].y = -0.65; //anchor2.y uint:m
// 	anchorArray[3].z = 2; //anchor2.z uint:m


// 	Range_deca[0] = 26393; //tag to A0 distance
// 	Range_deca[1] = 29368; //tag to A1 distance
// 	Range_deca[2] = 8655; //tag to A2 distance
// 	Range_deca[3] = 8278; //tag to A2 distance

// 	result = GetLocation(&report, 1, &anchorArray[0], &Range_deca[0]);

// 	printf("result = %d\r\n",result);
// 	printf("tag.x=%.3f\r\ntag.y=%.3f\r\ntag.z=%.3f\r\n",report.x,report.y,report.z);


// 	return 0;
// }