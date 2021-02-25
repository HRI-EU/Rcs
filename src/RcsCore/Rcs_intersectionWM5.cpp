/*******************************************************************************

  Copyright (c) 2017, Honda Research Institute Europe GmbH.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.

  3. All advertising materials mentioning features or use of this software
     must display the following acknowledgement: This product includes
     software developed by the Honda Research Institute Europe GmbH.

  4. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER "AS IS" AND ANY EXPRESS OR
  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#include "Rcs_intersectionWM5.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_math.h"
#include "Rcs_mesh.h"
#include "Rcs_geometry.h"
#include "Rcs_shape.h"

#if defined (USE_WM5)

#include <Wm5IntrPlane3Plane3.h>
#include <Wm5IntrLine3Box3.h>
#include <Wm5IntrLine3Cylinder3.h>
#include <Wm5IntrLine3Capsule3.h>
#include <Wm5IntrLine3Sphere3.h>
#include <Wm5IntrLine3Torus3.h>
#include <Wm5IntrLine3Cone3.h>
#include <Wm5IntrLine3Plane3.h>
#include <Wm5IntrPlane3Cylinder3.h>
#include <Wm5PolynomialRoots.h>

#include <Wm5Delaunay3.h>

#include <Wm5ContBox3.h>

#include <limits>


typedef Wm5::Vector3d Vec3;



/*******************************************************************************
 *
 ******************************************************************************/
template <typename T>
static bool intersectsLine(double* closestLinePt, T intrsec,
                           const double linePt[3],
                           const bool testImplemented=true)
{

  if (testImplemented && (closestLinePt==NULL))
  {
    //intrsec.Test() is e.g. not implemented for cylinders
    if (!intrsec.Test())
    {
      return false;
    }
  }
  else
  {
    intrsec.Find();

    if (intrsec.GetQuantity() < 1)
    {
      return false;
    }
  }

  if (closestLinePt)
  {
    // return the closest point to linePt
    double dist = std::numeric_limits<double>::infinity();
    for (int i = 0; i < intrsec.GetQuantity(); i++)
    {
      Vec3 v = intrsec.GetPoint(i);
      double pt[3] = { v[0], v[1], v[2] };
      double temp[3];

      Vec3d_sub(temp, linePt, pt);
      double d = Vec3d_getLength(temp);
      if (d < dist)
      {
        dist = d;
        Vec3d_copy(closestLinePt, pt);
      }
    }
  }

  return true;
}

/*******************************************************************************
 * Computes the intersection between a line given by a point and a direction
 * and a box. The function returns true if both intersect, and stores the
 * closest intersection point in closestLinePt (if it is not NULL).
 ******************************************************************************/
bool Rcs_intersectionLineBox(const double linePt[3],
                             const double lineDir[3],
                             const HTr* A_box,
                             const double extentsBox[3],
                             double* closestLinePt)
{
  // COG line
  Wm5::Line3d line(Vec3(linePt[0], linePt[1], linePt[2]),
                   Vec3(lineDir[0], lineDir[1], lineDir[2]));

  // Box
  Wm5::Box3d box(Vec3(A_box->org[0], A_box->org[1], A_box->org[2]),
                 Vec3(A_box->rot[0][0], A_box->rot[0][1], A_box->rot[0][2]),
                 Vec3(A_box->rot[1][0], A_box->rot[1][1], A_box->rot[1][2]),
                 Vec3(A_box->rot[2][0], A_box->rot[2][1], A_box->rot[2][2]),
                 0.5 * extentsBox[0], 0.5 * extentsBox[1], 0.5 * extentsBox[2]);

  Wm5::IntrLine3Box3d intrsec(line, box);

  return intersectsLine<Wm5::IntrLine3Box3d>(closestLinePt, intrsec,
                                             linePt);
}

/*******************************************************************************
 * Computes the intersection between a line given by a point and a direction
 * and a cylinder. The function returns true if both intersect, and stores the
 * closest intersection point in closestLinePt (if it is not NULL).
 ******************************************************************************/
bool Rcs_intersectionLineCylinder(const double linePt[3],
                                  const double lineDir[3],
                                  const HTr* A_cyl,
                                  double height,
                                  double radius,
                                  double* closestLinePt)
{
  // COG line
  Wm5::Line3d line(Vec3(linePt[0], linePt[1], linePt[2]),
                   Vec3(lineDir[0], lineDir[1], lineDir[2]));

  // Cylinder
  Wm5::Line3d cylLine(Vec3(A_cyl->org[0], A_cyl->org[1], A_cyl->org[2]),
                      Vec3(A_cyl->rot[2][0], A_cyl->rot[2][1],
                           A_cyl->rot[2][2]));

  Wm5::Cylinder3d cylinder(cylLine, radius, height);

  Wm5::IntrLine3Cylinder3d intrsec(line, cylinder);

  return intersectsLine<Wm5::IntrLine3Cylinder3d>(closestLinePt,
                                                  intrsec, linePt,
                                                  false);
}

/*******************************************************************************
 * Computes the intersection between a line given by a point and a direction
 * and a SSL/capsule. The function returns true if both intersect, and stores
 * the closest intersection point in closestLinePt (if it is not NULL).
 ******************************************************************************/
bool Rcs_intersectionLineSSL(const double linePt[3],
                             const double lineDir[3],
                             const HTr* A_cap,
                             double height,
                             double radius,
                             double* closestLinePt)
{
  // COG line
  Wm5::Line3d line(Vec3(linePt[0], linePt[1], linePt[2]),
                   Vec3(lineDir[0], lineDir[1], lineDir[2]));

  // SSL/Capsule
  Wm5::Segment3d capSegment(Vec3(A_cap->org[0], A_cap->org[1], A_cap->org[2]),
                            Vec3(A_cap->org[0]+height*A_cap->rot[2][0],
                                 A_cap->org[1]+height*A_cap->rot[2][1],
                                 A_cap->org[2]+height*A_cap->rot[2][2]));

  Wm5::Capsule3d capsule(capSegment, radius);

  Wm5::IntrLine3Capsule3d intrsec(line, capsule);

  return intersectsLine<Wm5::IntrLine3Capsule3d>(closestLinePt,
                                                 intrsec, linePt);
}

/*******************************************************************************
 * Computes the intersection between a line given by a point and a direction
 * and a sphere. The function returns true if both intersect, and stores the
 * closest intersection point in closestLinePt (if it is not NULL).
 ******************************************************************************/
bool Rcs_intersectionLineSphere(const double linePt[3],
                                const double lineDir[3],
                                const HTr* A_sph,
                                double radius,
                                double* closestLinePt)
{
  // COG line
  Wm5::Line3d line(Vec3(linePt[0], linePt[1], linePt[2]),
                   Vec3(lineDir[0], lineDir[1], lineDir[2]));

  // Sphere
  Wm5::Sphere3d sphere(Vec3(A_sph->org[0], A_sph->org[1], A_sph->org[2]),
                       radius);

  Wm5::IntrLine3Sphere3d intrsec(line, sphere);

  return intersectsLine<Wm5::IntrLine3Sphere3d>(closestLinePt,
                                                intrsec, linePt);
}

/*******************************************************************************
 * Computes the intersection between a line given by a point and a direction
 * and a torus. The function returns true if both intersect, and stores the
 * closest intersection point in closestLinePt (if it is not NULL).
 ******************************************************************************/
bool Rcs_intersectionLineTorus(const double linePt[3],
                               const double lineDir[3],
                               const HTr* A_tor,
                               double height,
                               double radius,
                               double* closestLinePt)
{
  // Wm5 torus is always at (0,0,0) with rotation axis in z direction
  double linePtTrafo[3];
  double lineDiHTrafo[3];

  Vec3d_rotate(lineDiHTrafo, const_cast<double (*)[3]>(A_tor->rot), lineDir);
  Vec3d_invTransform(linePtTrafo, A_tor, linePt);

  // COG line
  Wm5::Line3d line(Vec3(linePtTrafo[0], linePtTrafo[1], linePtTrafo[2]),
                   Vec3(lineDiHTrafo[0], lineDiHTrafo[1], lineDiHTrafo[2]));

  // Torus
  Wm5::Torus3d torus(radius, height/2.);

  Wm5::IntrLine3Torus3d intrsec(line, torus);

  bool res = intersectsLine<Wm5::IntrLine3Torus3d>(closestLinePt,
                                                   intrsec,
                                                   linePtTrafo,
                                                   false);

  if (closestLinePt && res)
  {
    Vec3d_transformSelf(closestLinePt, A_tor);
  }
  return res;
}

/*******************************************************************************
 * Computes the intersection between a line given by a point and a direction
 * and a cone. The function returns true if both intersect, and stores the
 * closest intersection point in closestLinePt (if it is not NULL).
 ******************************************************************************/
bool Rcs_intersectionLineCone(const double linePt[3],
                              const double lineDir[3],
                              const HTr* A_cone,
                              double height,
                              double radius,
                              double* closestLinePt)
{
  // COG line
  Wm5::Line3d line(Vec3(linePt[0], linePt[1], linePt[2]),
                   Vec3(lineDir[0], lineDir[1], lineDir[2]));

  double vertex[3];

  for (size_t id = 0; id < 3; id++)
  {
    vertex[id] = A_cone->org[id]+0.75*height*A_cone->rot[2][id];
  }

  // Cone
  Wm5::Cone3d cone(Vec3(vertex[0], vertex[1], vertex[2]),
                   Vec3(-A_cone->rot[2][0], -A_cone->rot[2][1],
                        -A_cone->rot[2][2]), atan(radius/height), height);

  Wm5::IntrLine3Cone3d intrsec(line, cone);

  // Wm5 ignores the height of the cone and finds intersections with an
  // infinite cone hence we always calculate the closest intersection point and
  // subsequently check whether it lies within the height
  // If an intersection is detected, the intersection with the base disc could
  // be closer, which needs to be checked additionally
  double temp_closestLinePt[3];

  bool res = intersectsLine<Wm5::IntrLine3Cone3d>(temp_closestLinePt,
                                                  intrsec, linePt,
                                                  false);

  if (Vec3d_distance(temp_closestLinePt,vertex) >
      sqrt(height*height + radius*radius) || (!res))
  {
    return false;
  }

  // here we check for intersections with the base disc
  if (closestLinePt)
  {
    Vec3d_copy(closestLinePt, temp_closestLinePt);

    double base[3];

    for (size_t id = 0; id < 3; id++)
    {
      base[id] = A_cone->org[id]-.25*height*A_cone->rot[2][id];
    }

    Wm5::Plane3d plane(Vec3(A_cone->rot[2][0], A_cone->rot[2][1],
                            A_cone->rot[2][2]),
                       Vec3(base[0], base[1], base[2]));

    Wm5::IntrLine3Plane3d intrsec_base(line, plane);

    double base_closestLinePt[3];

    // ignore if line is parallel to plane
    if (intrsec_base.Find() && intrsec_base.GetIntersectionType() != Wm5::Intersector<double,Wm5::Vector3<double> >::IT_LINE)
    {
      // intersection point
      Vec3d_constMulAndAdd(base_closestLinePt, linePt, lineDir,
                           intrsec_base.GetLineParameter());

      // restrict plane to circle
      if (Vec3d_distance(base_closestLinePt,base)<radius)
      {
        // check whether actually closer than other point
        if (Vec3d_distance(base_closestLinePt,linePt) <
            Vec3d_distance(temp_closestLinePt,linePt))
        {
          Vec3d_copy(closestLinePt, base_closestLinePt);
        }
      }
    }
  }

  return true;
}

/*******************************************************************************
 * Computes the intersection between a line given by a point and a direction
 * and a SSR. The function returns true if both intersect, and stores the
 * closest intersection point in closestLinePt (if it is not NULL).
 ******************************************************************************/
bool Rcs_intersectionLineSSR(const double linePt[3],
                             const double lineDir[3],
                             const HTr* A_ssr,
                             const double extentsSSR[3],
                             double* closestLinePt)
{
  double temp_closestLinePt[3];

  bool res = Rcs_intersectionLineBox(linePt, lineDir, A_ssr,
                                     extentsSSR, closestLinePt);
  if (closestLinePt==NULL && res)
  {
    // if we only want to detect an intersection we are done
    return true;
  }

  if (closestLinePt && res)
  {
    Vec3d_copy(temp_closestLinePt, closestLinePt);
  }

  // calculate the positions of all 4 corners
  double corners[4][3] = { {extentsSSR[0]/2.0, extentsSSR[1]/2.0, 0.0},
    {extentsSSR[0]/2.0, -extentsSSR[1]/2.0, 0.0},
    {-extentsSSR[0]/2.0, -extentsSSR[1]/2.0, 0.0},
    {-extentsSSR[0]/2.0, extentsSSR[1]/2.0, 0.0}
  };

  for (size_t id = 0; id < 4; id++)
  {
    Vec3d_transRotateSelf(corners[id], const_cast<double (*)[3]>(A_ssr->rot));
    Vec3d_addSelf(corners[id], const_cast<double (*)>(A_ssr->org));
  }

  // now we check for intersections with the 4 SSL/capsules
  Wm5::Line3d line(Vec3(linePt[0], linePt[1], linePt[2]),
                   Vec3(lineDir[0], lineDir[1], lineDir[2]));

  for (size_t id = 0; id < 4; id++)
  {
    size_t id2 = (id+1 > 3) ? 0: id+1;

    Wm5::Segment3d cap(Vec3(corners[id][0], corners[id][1], corners[id][2]),
                       Vec3(corners[id2][0], corners[id2][1], corners[id2][2]));

    Wm5::Capsule3d capsule(cap, extentsSSR[2]/2.);

    Wm5::IntrLine3Capsule3d intrsec(line, capsule);

    if (intersectsLine<Wm5::IntrLine3Capsule3d>(closestLinePt, intrsec, linePt))
    {
      if (closestLinePt==NULL)
      {
        return true;
      }

      if (!res) // first intersection point
      {
        Vec3d_copy(temp_closestLinePt, closestLinePt);
        res = true; // keep track of detected intersection
      }
      // compare to previous intersection point
      else if (Vec3d_distance(closestLinePt,linePt) <
               Vec3d_distance(temp_closestLinePt,linePt))
      {
        Vec3d_copy(temp_closestLinePt, closestLinePt);
      }
    }
  }

  if (closestLinePt && res)
  {
    Vec3d_copy(closestLinePt, temp_closestLinePt);
  }

  return res;
}

/*******************************************************************************
 * Computes the intersection between a plane given by a point and a direction
 * and a cylinder. The function returns true if both intersect.
 *
 * Compute extremes of signed distance Dot(N,X)-d for points on the
 * cylinder. These are
 * min = (Dot(N,C)-d) - r*sqrt(1-Dot(N,W)^2) - (h/2)*|Dot(N,W)|
 * max = (Dot(N,C)-d) + r*sqrt(1-Dot(N,W)^2) + (h/2)*|Dot(N,W)|
 ******************************************************************************/
bool Rcs_intersectionPlaneCylinder(const double planePt[3],
                                   const double planeNormal[3],
                                   const double cylinderPt[3],
                                   const double cylinderDir[3],
                                   double radius)
{
  Wm5::Plane3d plane(Vec3(planeNormal[0], planeNormal[1], planeNormal[2]),
                     Vec3(planePt[0], planePt[1], planePt[2]));

  Wm5::Line3d cylLine(Vec3(cylinderPt[0], cylinderPt[1], cylinderPt[2]),
                      Vec3(cylinderDir[0], cylinderDir[1], cylinderDir[2]));

  Wm5::Cylinder3d cylinder(cylLine, radius, 0.5*Vec3d_getLength(cylinderDir));

  // Compute extremes of signed distance Dot(N,X)-d for points on the
  // cylinder.  These are
  //   min = (Dot(N,C)-d) - r*sqrt(1-Dot(N,W)^2) - (h/2)*|Dot(N,W)|
  //   max = (Dot(N,C)-d) + r*sqrt(1-Dot(N,W)^2) + (h/2)*|Dot(N,W)|
  double sDist = plane.DistanceTo(cylinder.Axis.Origin);
  double absNdW = fabs(plane.Normal.Dot(cylinder.Axis.Direction));
  double root = sqrt(fabs(1.0 - absNdW*absNdW));
  double term = cylinder.Radius*root + ((double)0.5)*cylinder.Height*absNdW;

  // RLOG(0, "sDist: %f", sDist);
  // RLOG(0, "absNdW: %f", absNdW);
  // RLOG(0, "root: %f", root);
  // RLOG(0, "term: %f", term);
  // RLOG(0, "Radius: %f", cylinder.Radius);

  // Intersection occurs if and only if 0 is in the interval [min,max].
  return fabs(sDist) <= term;
}

/*******************************************************************************
 * Intersection tests
 ******************************************************************************/
extern "C" {
  bool RcsShape_computeLineIntersection(const double linePt[3],
                                        const double lineDir[3],
                                        const HTr* A_BI,
                                        const RcsShape* shape,
                                        double closestLinePt[3])
  {
    HTr A_CI;
    HTr_transform(&A_CI, A_BI, &shape->A_CB);

    switch (shape->type)
    {
      case RCSSHAPE_BOX:
      {
        return Rcs_intersectionLineBox(linePt, lineDir, &A_CI,
                                       shape->extents, closestLinePt);
      }
      case RCSSHAPE_SSL:
      {
        return Rcs_intersectionLineSSL(linePt, lineDir, &A_CI,
                                       shape->extents[2], shape->extents[0],
                                       closestLinePt);
      }
      case RCSSHAPE_CYLINDER:
      {
        return Rcs_intersectionLineCylinder(linePt, lineDir, &A_CI,
                                            shape->extents[2],
                                            shape->extents[0], closestLinePt);
      }
      case RCSSHAPE_SPHERE:
      {
        return Rcs_intersectionLineSphere(linePt, lineDir, &A_CI,
                                          shape->extents[0], closestLinePt);
      }
      case RCSSHAPE_CONE:
      {
        return Rcs_intersectionLineCone(linePt, lineDir, &A_CI,
                                        shape->extents[2], shape->extents[0],
                                        closestLinePt);
      }
      case RCSSHAPE_SSR:
      {
        return Rcs_intersectionLineSSR(linePt, lineDir, &A_CI, shape->extents,
                                       closestLinePt);
      }
      case RCSSHAPE_TORUS:
      {
        return Rcs_intersectionLineTorus(linePt, lineDir, &A_CI,
                                         shape->extents[2], shape->extents[0],
                                         closestLinePt);
      }
      case RCSSHAPE_REFFRAME:
      {
        return false;
      }
      case RCSSHAPE_MESH:
      {
        RLOG(4, "MESH intersection not implemented");
        return false;
      }
      case RCSSHAPE_OCTREE:
      {
        RLOG(4, "OCTREE intersection not implemented");
        return false;
      }
      case RCSSHAPE_POINT:
      {
        double d, linePt2[3];
        Vec3d_add(linePt2, linePt, lineDir);
        //d = Rcs_distancePointLine(A_CI.org, linePt, linePt2, NULL);
        double tmp[3];
        d = sqrt(Math_sqrDistPointLine(A_CI.org, linePt, lineDir, tmp));
        return (d==0.0) ? true : false;
      }
      default:
      {
        RFATAL("Unknown shape type %d", shape->type);
        return false;
      }
    }
  }
} // extern "C"

/*******************************************************************************
 *
 ******************************************************************************/
extern "C" {

  RcsMeshData* RcsMesh_fromVertices(const double* vCoords,
                                    unsigned int numVertices)
  {
    const double epsilon = 1.0e-8;
    const bool owner = false;
    Vec3* vertsWm5 = new Vec3[numVertices];

#if 0
    // Ignore duplicates
    for (unsigned int i=0; i<numVertices; ++i)
    {
      vertsWm5[i] = Vec3(vCoords[i*3+0], vCoords[i*3+1], vCoords[i*3+2]);
    }
#else
    // Remove duplicates
    for (unsigned int i=0; i<numVertices; ++i)
    {
      Vec3 verts_i = Vec3(vCoords[i*3+0], vCoords[i*3+1], vCoords[i*3+2]);

      for (unsigned int j=0; j<i; ++j)
      {
        if (pow(verts_i.X()-vertsWm5[j].X(), 2) +
            pow(verts_i.Y()-vertsWm5[j].Y(), 2) +
            pow(verts_i.Z()-vertsWm5[j].Z(), 2) < epsilon)
        {
          RLOG(1, "Found duplicate for index %d - skipping", i);
        }
      }

      vertsWm5[i] = verts_i;
    }
#endif
    Wm5::Delaunay3<double> d(numVertices, vertsWm5, epsilon, owner,
                             Wm5::Query::QT_INTEGER);

    NLOG(0, "Created Delaunay triangulation with %d vertices", numVertices);

    RcsMeshData* mesh = RALLOC(RcsMeshData);

    int numTriangles = 0;
    int* indices = NULL;
    bool success = d.GetHull(numTriangles, indices);

    if (success == false)
    {
      RLOG(4, "Failed to create Delaunay reiangulation for mesh");
      RFREE(mesh);
      delete [] vertsWm5;
      return NULL;
    }

    mesh->nFaces = numTriangles;
    mesh->faces = RNALLOC(3*mesh->nFaces, unsigned int);

    for (unsigned int i=0; i<3*mesh->nFaces; ++i)
    {
      mesh->faces[i] = indices[i];
    }

    mesh->nVertices = numVertices;
    mesh->vertices = VecNd_clone(vCoords, 3*numVertices);

    delete [] vertsWm5;
    delete [] indices;

    return mesh;
  }

} // extern "C"

/*******************************************************************************
 *
 ******************************************************************************/
extern "C" {

  bool Rcs_computeOrientedBox(HTr* A_box, double extents[3],
                              const double* points, unsigned int nPoints)
  {
    bool success = true;

    Wm5::Vector3d* vertsWm5 = new Wm5::Vector3d[nPoints];

    for (unsigned int i=0; i<nPoints; ++i)
    {
      const double* row = &points[3*i];
      vertsWm5[i] = Wm5::Vector3d(row[0], row[1], row[2]);
    }

    Wm5::Box3<double> box = Wm5::ContOrientedBox(nPoints, vertsWm5);

    for (int i=0; i<3; ++i)
    {
      A_box->org[i] = box.Center[i];
      A_box->rot[0][i] = box.Axis[0][i];
      A_box->rot[1][i] = box.Axis[1][i];
      Vec3d_crossProduct(A_box->rot[2], A_box->rot[0], A_box->rot[1]);
      //A_box->rot[2][i] = box.Axis[2][i];
      extents[i] = 2.0*box.Extent[i];
    }

    if (!Mat3d_isValid(A_box->rot))
    {
      RLOG(4, "Enclosing box has invalid rotation matrix");
      REXEC(4)
      {
        Mat3d_printCommentDigits("A_box", A_box->rot, 6);
      }
      success = false;
    }

    delete [] vertsWm5;

    return success;
  }

} // extern "C"

#else // USE_WM5

extern "C" {

  bool RcsShape_computeLineIntersection(const double linePt[3],
                                        const double lineDir[3],
                                        const HTr* A_BI,
                                        const RcsShape* shape,
                                        double closestLinePt[3])
  {
    RLOG(4, "RcsShape_computeLineIntersection requires GeometricTools library"
         " - not available");
    return false;
  }

  RcsMeshData* RcsMesh_fromVertices(const double* vCoords,
                                    unsigned int numVertices)
  {
    RLOG(4, "Delaunay triangulation requires GeometricTools library - "
         "not available");
    return NULL;
  }

  bool Rcs_computeOrientedBox(HTr* A_box, double extents[3],
                              const double* points, unsigned int nPoints)
  {
    RLOG(4, "Rcs_computeOrientedBox requires GeometricTools library - "
         "not available");
    return false;
  }

} // extern "C"

#endif
