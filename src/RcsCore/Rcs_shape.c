/*******************************************************************************

  Copyright (c) 2017, Honda Research Institute Europe GmbH

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are
  met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
  IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#include "Rcs_shape.h"
#include "Rcs_typedef.h"
#include "Rcs_kinematics.h"
#include "Rcs_macros.h"
#include "Rcs_utils.h"
#include "Rcs_math.h"
#include "Rcs_mesh.h"
#include "Rcs_geometry.h"
#include "Rcs_octree.h"



/*******************************************************************************
 * See header.
 ******************************************************************************/
double RcsShape_computeVolume(const RcsShape* self)
{
  double v = 0.0;

  switch (self->type)
  {
    case RCSSHAPE_NONE:
    case RCSSHAPE_REFFRAME:
    case RCSSHAPE_POINT:
    {
      break;
    }
    case RCSSHAPE_SSL:
    {
      v = M_PI*self->extents[0]*self->extents[0]*self->extents[2]; // cylinder
      v += 4.0/3.0*M_PI* pow(self->extents[0], 3);        // sphere
      break;
    }
    case RCSSHAPE_SSR:
    {
      double r = 0.5*self->extents[2];
      double g = M_PI*r*r; // circle area
      v = self->extents[0]*self->extents[1]*self->extents[2]; // box part
      v += g*self->extents[0]; // two half cylinders along extent x
      v += g*self->extents[1]; // two half cylinders along extent y
      v += 0.75*M_PI*pow(r, 3); // four sphere quarters at edges
      break;
    }
    case RCSSHAPE_MESH:
    {
      v = RcsMesh_computeVolume(self->mesh);
      break;
    }
    case RCSSHAPE_BOX:
    {
      v = self->extents[0] * self->extents[1] * self->extents[2];
      break;
    }
    case RCSSHAPE_CYLINDER:
    {
      v = M_PI * pow(self->extents[0], 2) * self->extents[2];
      break;
    }
    case RCSSHAPE_SPHERE:
    {
      v = 4.0 / 3.0 * M_PI * pow(self->extents[0], 3);
      break;
    }
    case RCSSHAPE_CONE:
    {
      v = M_PI / 3.0 * pow(self->extents[0], 2) * self->extents[2];
      break;
    }
    case RCSSHAPE_TORUS:
    {
      v = (M_PI*pow(self->extents[2]/2, 2))*(2.0*M_PI*self->extents[0]);
      break;
    }
    case RCSSHAPE_OCTREE:
    {
      v = Rcs_getOctreeVolume(self->mesh);
      break;
    }
    default:
    {
      RFATAL("Unknown shape type %d", self->type);
      break;
    }
  }

  return v;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
RcsMeshData* RcsShape_createMesh(const RcsShape* self)
{
  RcsMeshData* mesh = NULL;
  const unsigned int segments = 32;

  switch (self->type)
  {
    case RCSSHAPE_SSL:
    {
      mesh = RcsMesh_createCapsule(self->extents[0], self->extents[2],
                                   segments/2);
      break;
    }
    case RCSSHAPE_SSR:
    {
      mesh = RcsMesh_createSSR(self->extents, segments/4);
      break;
    }
    case RCSSHAPE_MESH:
    {
      mesh = RcsMesh_clone(self->mesh);
      break;
    }
    case RCSSHAPE_BOX:
    {
      mesh = RcsMesh_createBox(self->extents);
      break;
    }
    case RCSSHAPE_CYLINDER:
    {
      mesh = RcsMesh_createCylinder(self->extents[0], self->extents[2],
                                    segments);
      break;
    }
    case RCSSHAPE_SPHERE:
    {
      mesh = RcsMesh_createSphere(self->extents[0], segments/2);
      break;
    }
    case RCSSHAPE_CONE:
    {
      mesh = RcsMesh_createCone(self->extents[0], self->extents[2], segments);
      break;
    }
    case RCSSHAPE_TORUS:
    {
      mesh = RcsMesh_createTorus(self->extents[0], self->extents[2],
                                 segments, segments);
      break;
    }
    default:
    {
      RLOG(1, "Shape type %s (%d) can't be converted to mesh",
           RcsShape_name(self->type), self->type);
      break;
    }
  }

  return mesh;
}

/*******************************************************************************
 * Computes the local COM of the shape in the bodie's frame according to its
 * volume and the relative transformation A_CB from body frame to shape frame.
 ******************************************************************************/
void RcsShape_computeLocalCOM(const RcsShape* self, double r_com[3])
{
  switch (self->type)
  {
    // The SSL's reference point is the ball point. We must consider the
    // offset to the COM.
    case RCSSHAPE_SSL:
    {
      double c_offs[3], b_offs[3];
      Vec3d_set(c_offs, 0.0, 0.0, 0.5*self->extents[2]);
      Vec3d_transRotate(b_offs, (double(*)[3]) self->A_CB.rot, c_offs);
      Vec3d_add(r_com, self->A_CB.org, b_offs);
      break;
    }
    // The cone's reference point is the base plane. We must consider the
    // offset to the COM which is a quarter of it's height.
    case RCSSHAPE_CONE:
    {
      double c_offs[3], b_offs[3];
      Vec3d_set(c_offs, 0.0, 0.0, 0.25*self->extents[2]);
      Vec3d_transRotate(b_offs, (double(*)[3]) self->A_CB.rot, c_offs);
      Vec3d_add(r_com, self->A_CB.org, b_offs);
      break;
    }
    // All other shapes have the COM as the reference point.
    default:
    {
      Vec3d_copy(r_com, self->A_CB.org);
      break;
    }

  }   // switch(...)

}

/*******************************************************************************
 * Inertia tensor of a cylinder about its COM
 *
 * From https://en.wikipedia.org/wiki/List_of_moments_of_inertia :
 * I_xx = I_yy = (m/12.0)*(3.0*r^2 + h^2);
 * I_zz = 0.5 m r^2
 ******************************************************************************/
static void RcsShape_cylinderInertia(double I[3], double m, double r, double h)
{
  I[0] = (1.0/12.0)*m*(3.0*r*r + h*h);
  I[1] = I[0];
  I[2] = 0.5*m*r*r;
}

/*******************************************************************************
 * Inertia tensor of a sphere about its COM
 *
 * From https://en.wikipedia.org/wiki/List_of_moments_of_inertia :
 * I_xx = I_yy = I_zz = 2/5 m r^2
 ******************************************************************************/
static void RcsShape_sphereInertia(double I[3], double m, double r)
{
  I[0] = 0.4*m*r*r;
  I[1] = I[0];
  I[2] = I[0];
}

/*******************************************************************************
 * Inertia tensor of a torus about its COM. The symmetry axis is z.
 ******************************************************************************/
static void RcsShape_torusInertia(double I[3], double m, double r, double h)
{
  I[0] = m*(4.0*r*r+5.0*(h/2.0)*(h/2.0))/8.0;
  I[1] = I[0];
  I[2] = m*(4.0*r*r+3.0*(h/2.0)*(h/2.0))/4.0;
}

/*******************************************************************************
 * Inertia tensor of a box about its COM.
 ******************************************************************************/
static void RcsShape_boxInertia(double I[3], double m, const double extents[3])
{
  double xx = pow(extents[0], 2);
  double yy = pow(extents[1], 2);
  double zz = pow(extents[2], 2);
  I[0] = (m / 12.0) * (yy + zz);
  I[1] = (m / 12.0) * (xx + zz);
  I[2] = (m / 12.0) * (xx + yy);
}

/*******************************************************************************
 * Inertia tensor of a capsule about its COM. The symmetry axis is z.
 ******************************************************************************/
static void RcsShape_capsuleInertia(double I[3], double density, double r,
                                    double h)
{
  // Cylinder part about cylinder COM
  double I_cyl[3], m_cyl = M_PI*r*r*h*density;
  RcsShape_cylinderInertia(I_cyl, m_cyl, r, h);

  // Hemisphere inertia about its COM. The hemisphere COM has an offset
  // of 3.0*r/8.0 (later considered in the Steiner term)
  double I_hem[3], m_hem = (2.0/3.0)*M_PI*r*r*r*density;
  I_hem[0] = 83.0/320.0*m_hem*r*r;
  I_hem[1] = I_hem[0];
  I_hem[2] = 0.4*m_hem*r*r;

  // Hemisphere Steiner term. The COM is displaced 0.5*h+3.0*r/8.0
  double offset_hem = 0.5*h+3.0*r/8.0;
  double tmp = m_hem*offset_hem*offset_hem;
  I_hem[0] += tmp;
  I_hem[1] += tmp;

  // We have 2 hemispheres
  Vec3d_constMulSelf(I_hem, 2.0);
  Vec3d_add(I, I_cyl, I_hem);
}

/*******************************************************************************
 * Inertia tensor of a triangle mesh about its COM.
 ******************************************************************************/
static void RcsShape_meshInertia(double I_diag[3], const RcsShape* self,
                                 double density)
{
  if (self->mesh)
  {
    double I[3][3], com[3];
    RcsMesh_computeInertia(self->mesh, I, com);
    Vec3d_set(I_diag, I[0][0], I[1][1], I[2][2]);
    Vec3d_constMulSelf(I_diag, density);
  }
  else
  {
    Vec3d_setZero(I_diag);
  }
}

/*******************************************************************************
 * Inertia tensor of a cone about its COM.
 ******************************************************************************/
//! \todo That's the wrong reference point (upside down cone)
static void RcsShape_coneInertia(double I[3], double m, double r, double h)
{
  I[0] = (3.0/80.0)*m*(4*r*r+h*h);
  I[1] = I[0];
  I[2] = (3.0/10.0)*m*r*r;
}

/*******************************************************************************
 * Computes the local inertia tensor of the shape with respect to the shape's
 * COM (not its origin), and represented in the shape's frame.
 ******************************************************************************/
//! \todo SSR inertia if there is time ...
void RcsShape_computeInertiaTensor(const RcsShape* self, const double density,
                                   double I[3][3])
{
  Mat3d_setZero(I);

  // rho = m / V   =>   m_shape = rho * V
  double m_sh = density*RcsShape_computeVolume(self);
  double r = self->extents[0];
  double h = self->extents[2];
  double I_diag[3];

  Vec3d_setZero(I_diag);

  // Inertia tensor in shape frame around individual shape's COM.
  switch (self->type)
  {
    case RCSSHAPE_NONE:
    case RCSSHAPE_REFFRAME:
    case RCSSHAPE_POINT:
      break;
    case RCSSHAPE_MESH:
      RcsShape_meshInertia(I_diag, self, density);
      break;
    case RCSSHAPE_OCTREE:
    {
      RLOG(5, "Inertia computation for shape type \"%s\" not implemented",
           RcsShape_name(self->type));
      break;
    }
    case RCSSHAPE_SSR:
    case RCSSHAPE_BOX:
    {
      RcsShape_boxInertia(I_diag, m_sh, self->extents);
      break;
    }
    case RCSSHAPE_SSL:
    {
      RcsShape_capsuleInertia(I_diag, density, r, h);
      break;
    }
    case RCSSHAPE_CYLINDER:
    {
      RcsShape_cylinderInertia(I_diag, m_sh, r, h);
      break;
    }
    case RCSSHAPE_SPHERE:
    {
      RcsShape_sphereInertia(I_diag, m_sh, r);
      break;
    }
    case RCSSHAPE_CONE:
    {
      RcsShape_coneInertia(I_diag, m_sh, r, h);
      break;
    }
    case RCSSHAPE_TORUS:
    {
      RcsShape_torusInertia(I_diag, m_sh, r, h);
      break;
    }
    default:
    {
      RFATAL("Unknown shape type %d", self->type);
      break;
    }

  } // switch(self->type)

  Mat3d_setDiag(I, I_diag);
}

/*******************************************************************************
 *
 ******************************************************************************/
//! \todo Cone needs update
double RcsShape_boundingSphereDistance(const double Pt[3],
                                       const HTr* A_BI,
                                       const RcsShape* shape)
{
  double com[3];
  RcsShape_computeLocalCOM(shape, com);
  Vec3d_transformSelf(com, A_BI);

  switch (shape->type)
  {
    case RCSSHAPE_POINT:
    {
      return Vec3d_distance(com, Pt);
    }
    case RCSSHAPE_BOX:
    {
      return Vec3d_distance(com, Pt) - 0.5*Vec3d_getLength(shape->extents);
    }
    case RCSSHAPE_SSL:
    {
      return Vec3d_distance(com, Pt)-0.5*shape->extents[2]-shape->extents[0];
    }
    case RCSSHAPE_CYLINDER:
    {
      return Vec3d_distance(com, Pt) -
             sqrt(shape->extents[2]*shape->extents[2]/4.0 +
                  shape->extents[0]*shape->extents[0]);
    }
    case RCSSHAPE_SPHERE:
    {
      return Vec3d_distance(com, Pt) - shape->extents[0];
    }
    case RCSSHAPE_CONE:
    {
      return Vec3d_distance(com, Pt) -
             fmax(0.75*shape->extents[2],
                  sqrt(shape->extents[2]*shape->extents[2]/16.0 +
                       shape->extents[0]*shape->extents[0]));
    }
    case RCSSHAPE_SSR:
    {
      return Vec3d_distance(com, Pt) -
             sqrt(shape->extents[0]*shape->extents[0] +
                  shape->extents[1]*shape->extents[1])/2.0 - shape->extents[2]/2.0;
    }
    case RCSSHAPE_TORUS:
    {
      return Vec3d_distance(com, Pt)-shape->extents[0]-0.5*shape->extents[2];
    }
    case RCSSHAPE_REFFRAME:
    {
      return Math_infinity();
    }
    case RCSSHAPE_MESH:
    {
      RLOG(4, "MESH not implemented");
      return Math_infinity();
    }
    case RCSSHAPE_OCTREE:
    {
      RLOG(4, "OCTREE not implemented");
      return Math_infinity();
    }
    default:
    {
      RFATAL("Unknown shape type %d", shape->type);
      return Math_infinity();
    }
  }
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsShape_clear(RcsShape* self)
{
  if (self == NULL)
  {
    return;
  }

  if (self->type == RCSSHAPE_OCTREE)
  {
    // We assume userData is an Octomap type
    Rcs_destroyOctree(self->mesh);
  }

  if (self->type == RCSSHAPE_MESH)
  {
    RcsMesh_destroy(self->mesh);
  }

}

/*******************************************************************************
 * See header.
 ******************************************************************************/
const char* RcsShape_name(int shapeType)
{
  static char sName[][32] = {"RCSSHAPE_NONE",
                             "RCSSHAPE_SSL",
                             "RCSSHAPE_SSR",
                             "RCSSHAPE_MESH",
                             "RCSSHAPE_BOX",
                             "RCSSHAPE_CYLINDER",
                             "RCSSHAPE_REFFRAME",
                             "RCSSHAPE_SPHERE",
                             "RCSSHAPE_CONE",
                             "RCSSHAPE_TORUS",
                             "RCSSHAPE_OCTREE",
                             "RCSSHAPE_POINT",
                             "Unknown shape type"
                            };
  const char* ptr = NULL;

  switch (shapeType)
  {
    case RCSSHAPE_NONE:
      ptr = sName[0];
      break;
    case RCSSHAPE_SSL:
      ptr = sName[1];
      break;
    case RCSSHAPE_SSR:
      ptr = sName[2];
      break;
    case RCSSHAPE_MESH:
      ptr = sName[3];
      break;
    case RCSSHAPE_BOX:
      ptr = sName[4];
      break;
    case RCSSHAPE_CYLINDER:
      ptr = sName[5];
      break;
    case RCSSHAPE_REFFRAME:
      ptr = sName[6];
      break;
    case RCSSHAPE_SPHERE:
      ptr = sName[7];
      break;
    case RCSSHAPE_CONE:
      ptr = sName[8];
      break;
    case RCSSHAPE_TORUS:
      ptr = sName[9];
      break;
    case RCSSHAPE_OCTREE:
      ptr = sName[10];
      break;
    case RCSSHAPE_POINT:
      ptr = sName[11];
      break;
    default:
      ptr = sName[12];
      break;
  }

  return ptr;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsShape_copy(RcsShape* dst, const RcsShape* src)
{
  // Copy the members that are not pointing to somewhere
  RcsMeshData* tmp = dst->mesh;
  memcpy(dst, src, sizeof(RcsShape));
  dst->mesh = tmp;

  switch (dst->type)
  {
    // Load Octree from file
    case RCSSHAPE_OCTREE:
      Rcs_destroyOctree(dst->mesh);
      dst->mesh = (RcsMeshData*)Rcs_loadOctree(dst->meshFile);
      break;

    case RCSSHAPE_MESH:
    {
      RcsMesh_destroy(dst->mesh);   // Does nothing if dstMesh is NULL
      dst->mesh = RcsMesh_clone(src->mesh);
    }
    break;

    default:
      break;
  }

}

/*******************************************************************************
 * See header.
 ******************************************************************************/
RcsShape* RcsShape_clone(const RcsShape* src)
{
  RcsShape* shape = RALLOC(RcsShape);
  RcsShape_copy(shape, src);
  return shape;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void RcsShape_fprint(FILE* out, const RcsShape* s)
{
  if (s == NULL)
  {
    RLOG(1, "Shape doesn't exist");
    return;
  }

  // Shape type
  fprintf(out, "[RcsShape_fprint():%d] \n\tShape of type \"%s\"\n",
          __LINE__, RcsShape_name(s->type));

  // Relative transformation
  fprintf(out, "\n\tRelative transformation:\n");
  HTr_fprint(out, &s->A_CB);
  double ea[3];
  Mat3d_toEulerAngles(ea, (double (*)[3])s->A_CB.rot);
  fprintf(out, "\n\tEuler angles:%f %f %f [deg]\n",
          RCS_RAD2DEG(ea[0]), RCS_RAD2DEG(ea[1]), RCS_RAD2DEG(ea[2]));

  // Extents
  fprintf(out, "\tExtents: %.3f %.3f %.3f\n",
          s->extents[0], s->extents[1], s->extents[2]);

  // Scale
  fprintf(out, "\tScale: %f %f %f\n",
          s->scale3d[0], s->scale3d[1], s->scale3d[2]);

  // Compute type
  fprintf(out, "\tCompute type: ");
  if ((s->computeType & RCSSHAPE_COMPUTE_DISTANCE) != 0)
  {
    fprintf(out, "distance ");
  }
  if ((s->computeType & RCSSHAPE_COMPUTE_PHYSICS) != 0)
  {
    fprintf(out, "physics ");
  }
  if ((s->computeType & RCSSHAPE_COMPUTE_GRAPHICS) != 0)
  {
    fprintf(out, "graphics ");
  }
  if ((s->computeType & RCSSHAPE_COMPUTE_CONTACT) != 0)
  {
    fprintf(out, "contact ");
  }
  if ((s->computeType & RCSSHAPE_COMPUTE_SOFTPHYSICS) != 0)
  {
    fprintf(out, "softPhysics ");
  }
  if ((s->computeType & RCSSHAPE_COMPUTE_DEPTHBUFFER) != 0)
  {
    fprintf(out, "depth ");
  }
  if ((s->computeType & RCSSHAPE_COMPUTE_ATTACHMENT) != 0)
  {
    fprintf(out, "attachement ");
  }
  if ((s->computeType & RCSSHAPE_COMPUTE_WELDPOS) != 0)
  {
    fprintf(out, "weldPos ");
  }
  if ((s->computeType & RCSSHAPE_COMPUTE_WELDORI) != 0)
  {
    fprintf(out, "weldOri ");
  }
  if ((s->computeType & RCSSHAPE_COMPUTE_MARKER) != 0)
  {
    fprintf(out, "marker ");
  }
  fprintf(out, "\n");

  // Resizeable
  fprintf(out, "\tResizeable: %s\n", s->resizeable ? "true" : "false");

  // File names
  fprintf(out, "\tmeshFile   : \"%s\"\n", s->meshFile);
  fprintf(out, "\ttextureFile: \"%s\"\n", s->textureFile);
  fprintf(out, "\tcolor      : \"%s\"\n", s->color);
  fprintf(out, "\tmaterial   : \"%s\"\n", s->material);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
int RcsShape_fprintXML(FILE* out, const RcsShape* self)
{
  int nErr = 0;
  char buf[256];

  fprintf(out, "    <Shape ");

  switch (self->type)
  {
    case RCSSHAPE_SSL:
      fprintf(out, "type=\"SSL\" ");
      break;

    case RCSSHAPE_SSR:
      fprintf(out, "type=\"SSR\" ");
      break;

    case RCSSHAPE_BOX:
      fprintf(out, "type=\"BOX\" ");
      break;

    case RCSSHAPE_CYLINDER:
      fprintf(out, "type=\"CYLINDER\" ");
      break;

    case RCSSHAPE_MESH:
      fprintf(out, "type=\"MESH\" ");
      break;

    case RCSSHAPE_REFFRAME:
      fprintf(out, "type=\"FRAME\" ");
      break;

    case RCSSHAPE_SPHERE:
      fprintf(out, "type=\"SPHERE\" ");
      break;

    case RCSSHAPE_CONE:
      fprintf(out, "type=\"CONE\" ");
      break;

    case RCSSHAPE_TORUS:
      fprintf(out, "type=\"TORUS\" ");
      break;

    case RCSSHAPE_OCTREE:
      fprintf(out, "type=\"OCTREE\" ");
      break;

    case RCSSHAPE_POINT:
      fprintf(out, "type=\"POINT\" ");
      break;

    default:
      RLOG(1, "Unknown shape type: %d", self->type);
      nErr++;
  }

  // Extents
  switch (self->type)
  {
    case RCSSHAPE_SSL:
    case RCSSHAPE_CYLINDER:
    case RCSSHAPE_TORUS:
    case RCSSHAPE_CONE:
      fprintf(out, "length=\"%s\" ",
              String_fromDouble(buf, self->extents[2], 6));
      fprintf(out, "radius=\"%s\" ",
              String_fromDouble(buf, self->extents[0], 6));
      break;

    case RCSSHAPE_SPHERE:
      fprintf(out, "radius=\"%s\" ",
              String_fromDouble(buf, self->extents[0], 6));
      break;

    case RCSSHAPE_SSR:
    case RCSSHAPE_BOX:
      fprintf(out, "extents=\"%s ",
              String_fromDouble(buf, self->extents[0], 6));
      fprintf(out, "%s ",   String_fromDouble(buf, self->extents[1], 6));
      fprintf(out, "%s\" ", String_fromDouble(buf, self->extents[2], 6));
      break;
  }

  // Compute type, only written out if shape is not a frame
  if (self->type != RCSSHAPE_REFFRAME)
  {
    if (self->computeType & RCSSHAPE_COMPUTE_DISTANCE)
    {
      fprintf(out, "distance=\"true\" ");
    }
    else
    {
      fprintf(out, "distance=\"false\" ");
    }

    if (self->computeType & RCSSHAPE_COMPUTE_PHYSICS)
    {
      fprintf(out, "physics=\"true\" ");
    }
    else
    {
      fprintf(out, "physics=\"false\" ");
    }

    if (self->computeType & RCSSHAPE_COMPUTE_GRAPHICS)
    {
      fprintf(out, "graphics=\"true\" ");
    }
    else
    {
      fprintf(out, "graphics=\"false\" ");
    }
    if (self->computeType & RCSSHAPE_COMPUTE_SOFTPHYSICS)
    {
      fprintf(out, "softPhysics=\"true\" ");
    }

    if (self->computeType & RCSSHAPE_COMPUTE_CONTACT)
    {
      fprintf(out, "contact=\"true\" ");
    }

    if (self->computeType & RCSSHAPE_COMPUTE_DEPTHBUFFER)
    {
      fprintf(out, "depth=\"true\" ");
    }

    if (self->computeType & RCSSHAPE_COMPUTE_ATTACHMENT)
    {
      fprintf(out, "attachment=\"true\" ");
    }

    if (self->computeType & RCSSHAPE_COMPUTE_WELDPOS)
    {
      fprintf(out, "weldpos=\"true\" ");
    }

    if (self->computeType & RCSSHAPE_COMPUTE_WELDORI)
    {
      fprintf(out, "weldori=\"true\" ");
    }

    if (self->computeType & RCSSHAPE_COMPUTE_MARKER)
    {
      fprintf(out, "marker=\"true\" ");
    }
  }

  // Resizeable
  if (self->resizeable)
  {
    fprintf(out, "resizeable=\"true\" ");
  }

  // Scale
  if ((self->scale3d[0]!=1.0) || (self->scale3d[1]!=1.0) || (self->scale3d[2]!=1.0))
  {
    if ((self->scale3d[0]==self->scale3d[1]) &&
        (self->scale3d[0]==self->scale3d[2]))
    {
      fprintf(out, "scale=\"%s\" ", String_fromDouble(buf, self->scale3d[0], 6));
    }
    else
    {
      fprintf(out, "scale=\"%s ", String_fromDouble(buf, self->scale3d[0], 6));
      fprintf(out, "%s ", String_fromDouble(buf, self->scale3d[1], 6));
      fprintf(out, "%s\" ", String_fromDouble(buf, self->scale3d[2], 6));
    }
  }

  // Relative transformation only if non-zero elements exist
  {
    double trf[6];
    Vec3d_copy(&trf[0], self->A_CB.org);
    Mat3d_toEulerAngles(&trf[3], (double (*)[3]) self->A_CB.rot);
    Vec3d_constMulSelf(&trf[3], 180.0 / M_PI);

    if (VecNd_maxAbsEle(trf, 6) > 1.0e-8)
    {
      fprintf(out, "transform=\"%s ", String_fromDouble(buf, trf[0], 6));
      fprintf(out, "%s ", String_fromDouble(buf, trf[1], 6));
      fprintf(out, "%s ", String_fromDouble(buf, trf[2], 6));
      fprintf(out, "%s ", String_fromDouble(buf, trf[3], 6));
      fprintf(out, "%s ", String_fromDouble(buf, trf[4], 6));
      fprintf(out, "%s\" ", String_fromDouble(buf, trf[5], 6));
    }
  }

  // Mesh file
  if (strlen(self->meshFile)>0)
  {
    fprintf(out, "meshFile=\"%s\" ", self->meshFile);
  }

  // Texture file
  if (strlen(self->textureFile)>0)
  {
    fprintf(out, "textureFile=\"%s\" ", self->textureFile);
  }

  // Color
  if ((self->type != RCSSHAPE_REFFRAME) && (strlen(self->color)>0)
      && (!STREQ(self->color, "DEFAULT")))
  {
    fprintf(out, "color=\"%s\" ", self->color);
  }

  // Material
  if ((strlen(self->material)>0) && (!STREQ(self->material, "default")))
  {
    fprintf(out, "material=\"%s\" ", self->material);
  }


  fprintf(out, "/>\n");

  return nErr;
}

/*******************************************************************************
 * Initialize shape with default properties
 ******************************************************************************/
void RcsShape_init(RcsShape* self)
{
  memset(self, 0, sizeof(RcsShape));
  Vec3d_setElementsTo(self->scale3d, 1.0);
  HTr_setIdentity(&self->A_CB);
}

/*******************************************************************************
 *
 ******************************************************************************/
bool RcsShape_isSupported(int shapeType)
{
#if !defined (USE_OCTOMAP)
  if (shapeType == RCSSHAPE_OCTREE)
  {
    return false;
  }
#endif

  return true;
}

/*******************************************************************************
 * Return random shape type. We put it here to not expose the details about
 * available shape types such as Octomap shapes to the outside.
 ******************************************************************************/
int RcsShape_randomShapeType()
{
  int shapeType = Math_getRandomInteger(1, RCSSHAPE_SHAPE_MAX - 1);

  // We return the first one in case an OCTREE shape has been selected and is
  // not available. This might bias a distribution. The clean solution would be
  // a recursion, which I don't think is necessary and potentially introduces
  // issues.
#if !defined (USE_OCTOMAP)
  if (shapeType == RCSSHAPE_OCTREE)
  {
    shapeType = 1;
  }
#endif

  return shapeType;
}

/*******************************************************************************
 * Create a shape with random properties
 ******************************************************************************/
bool RcsShape_initRandom(RcsShape* shape, int shapeType)
{
  if (shape==NULL)
  {
    RLOG(1, "NULL shape of type %d cannot be initialized", shapeType);
    return false;
  }

  if ((shapeType<=RCSSHAPE_NONE) || (shapeType>=RCSSHAPE_SHAPE_MAX))
  {
    RLOG(1, "Unknown shape type %d", shapeType);
    return false;
  }

#if !defined (USE_OCTOMAP)
  if (shapeType==RCSSHAPE_OCTREE)
  {
    RLOG(1, "Can't create octree, no OctoMap support compiled in");
    return false;
  }
#endif

  RcsShape_init(shape);

  // Allocate memory and set defaults
  shape->type = shapeType;
  shape->resizeable = true;
  Vec3d_setRandom(shape->extents, 0.1, 0.3);
  Vec3d_setRandom(shape->A_CB.org, -0.1, 0.1);
  Mat3d_setRandomRotation(shape->A_CB.rot);

  int rr = Math_getRandomInteger(0, 255);
  int gg = Math_getRandomInteger(0, 255);
  int bb = Math_getRandomInteger(0, 255);
  sprintf(shape->color, "#%02x%02x%02xff", rr, gg, bb); // RGBA with A=255=0xff

  shape->computeType |= RCSSHAPE_COMPUTE_DISTANCE;
  shape->computeType |= RCSSHAPE_COMPUTE_PHYSICS;
  shape->computeType |= RCSSHAPE_COMPUTE_GRAPHICS;

  if (shapeType==RCSSHAPE_MESH)
  {
    strcpy(shape->meshFile, "Random mesh");
    shape->mesh = RcsMesh_createConvexRandom(0.2, 0.4, 0.6, 128);
    RCHECK(shape->mesh);
  }

#if defined (USE_OCTOMAP)
  if (shapeType==RCSSHAPE_OCTREE)
  {
    const char* sit = getenv("SIT");
    if (sit!=NULL)
    {
      snprintf(shape->meshFile, RCS_MAX_FILENAMELEN,
               "%s/Data/RobotMeshes/1.0/data/octomap/test.bt", sit);
      shape->mesh = (RcsMeshData*)Rcs_loadOctree(shape->meshFile);
      RCHECK(shape->mesh);
    }
  }
#endif

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
RcsShape* RcsShape_createFrameShape(double scale)
{
  RcsShape* shape = RALLOC(RcsShape);
  RcsShape_init(shape);
  Vec3d_setElementsTo(shape->scale3d, scale);
  shape->type = RCSSHAPE_REFFRAME;
  shape->computeType |= RCSSHAPE_COMPUTE_GRAPHICS;
  shape->resizeable = false;
  shape->extents[0] = 0.9;
  shape->extents[1] = 0.9;
  shape->extents[2] = 0.9;

  return shape;
}

/*******************************************************************************
 * Default distance function: infinity distance.
 ******************************************************************************/
static double RcsShape_noDistance(const RcsShape* s1, const RcsShape* s2,
                                  const HTr* A_1I, const HTr* A_2I,
                                  double I_cp1[3], double I_cp2[3],
                                  double I_n[3])
{
  RLOG(4, "Distance function for pair \"%s\" - \"%s\" undefined!",
       RcsShape_name(s1->type), RcsShape_name(s2->type));
  return Math_infinity();
}

/*******************************************************************************
 * Computes the 3d distance between two points. If the points coincide, the
 * normal vector is set to the unit z direction vector.
 ******************************************************************************/
static double RcsShape_closestPointToPoint(const RcsShape* p1,
                                           const RcsShape* p2,
                                           const HTr* A_p1,
                                           const HTr* A_p2,
                                           double I_cp1[3],
                                           double I_cp2[3],
                                           double I_n12[3])
{
  Vec3d_copy(I_cp1, A_p1->org);
  Vec3d_copy(I_cp2, A_p2->org);
  Vec3d_sub(I_n12, A_p2->org, A_p1->org);
  double len = Vec3d_normalizeSelf(I_n12);
  if (len==0.0)
  {
    Vec3d_setUnitVector(I_n12, 2);
  }

  return Vec3d_distance(A_p1->org, A_p2->org);
}

/*******************************************************************************
 * Computes the 3d distance between a point and a sphere. If the point and the
 * sphere center coincide, the normal vector is set to the unit z direction
 * vector.
 ******************************************************************************/
static double RcsShape_closestPointToSphere(const RcsShape* pt,
                                            const RcsShape* sp,
                                            const HTr* A_pt,
                                            const HTr* A_sp,
                                            double I_cp1[3],
                                            double I_cp2[3],
                                            double I_n12[3])
{
  Vec3d_copy(I_cp1, A_pt->org);

  Vec3d_sub(I_n12, A_sp->org, A_pt->org);
  double d = Vec3d_normalizeSelf(I_n12);
  if (d==0.0)
  {
    Vec3d_setUnitVector(I_n12, 2);
  }

  Vec3d_constMulAndAdd(I_cp2, A_sp->org, I_n12, -sp->extents[0]);

  return d-sp->extents[0];
}

/*******************************************************************************
 * Computes the 3d distance between two spheres. If the sphere centers
 * coincide, the normal vector is set to the unit z direction vector.
 ******************************************************************************/
static double RcsShape_closestSphereToSphere(const RcsShape* sphere1,
                                             const RcsShape* sphere2,
                                             const HTr* A_sphere1,
                                             const HTr* A_sphere2,
                                             double I_cp1[3],
                                             double I_cp2[3],
                                             double I_n12[3])
{
  Vec3d_copy(I_cp1, A_sphere1->org);

  Vec3d_sub(I_n12, A_sphere2->org, A_sphere1->org);
  double dist = Vec3d_normalizeSelf(I_n12);
  if (dist==0.0)
  {
    Vec3d_setUnitVector(I_n12, 2);
  }

  Vec3d_constMulAndAdd(I_cp1, A_sphere1->org, I_n12, sphere1->extents[0]);
  Vec3d_constMulAndAdd(I_cp2, A_sphere2->org, I_n12, -sphere2->extents[0]);

  return dist-sphere2->extents[0]-sphere1->extents[0];
}

/*******************************************************************************
 * Computes the distance between a sphere and a point primitive.
 ******************************************************************************/
static double RcsShape_closestSphereToPoint(const RcsShape* sphere,
                                            const RcsShape* point,
                                            const HTr* A_sphere,
                                            const HTr* A_point,
                                            double I_cpSphere[3],
                                            double I_cpPoint[3],
                                            double I_nSpherePoint[3])
{
  double dist = RcsShape_closestPointToSphere(point, sphere, A_point, A_sphere,
                                              I_cpPoint, I_cpSphere,
                                              I_nSpherePoint);

  // Revert the normal, because we are calling the reverse method
  Vec3d_constMulSelf(I_nSpherePoint, -1.0);

  return dist;
}

/*******************************************************************************
 * Computes the distance between a point and a SSL primitive.
 ******************************************************************************/
static double RcsShape_closestPointToSSL(const RcsShape* point,
                                         const RcsShape* ssl,
                                         const HTr* A_point,
                                         const HTr* A_ssl,
                                         double I_cpPoint[3],
                                         double I_cpSSL[3],
                                         double I_nPointSSL[3])
{
  double dist = Math_distPointCapsule(A_point->org, A_ssl->org, A_ssl->rot[2],
                                      ssl->extents[2], ssl->extents[0],
                                      I_cpSSL, I_nPointSSL);
  Vec3d_copy(I_cpPoint, A_point->org);

  return dist;
}

/*******************************************************************************
 * Computes the distance between a SSL and a point primitive.
 ******************************************************************************/
static double RcsShape_closestSSLToPoint(const RcsShape* ssl,
                                         const RcsShape* point,
                                         const HTr* A_ssl,
                                         const HTr* A_point,
                                         double I_cpSSL[3],
                                         double I_cpPoint[3],
                                         double I_nSSLPoint[3])
{
  double dist = RcsShape_closestPointToSSL(point, ssl, A_point, A_ssl,
                                           I_cpPoint, I_cpSSL, I_nSSLPoint);

  // Revert the normal, because we are calling the reverse method
  Vec3d_constMulSelf(I_nSSLPoint, -1.0);

  return dist;
}

/*******************************************************************************
 * Computes the distance between a SSL and sphere primitive.
 ******************************************************************************/
static double RcsShape_closestSSLToSphere(const RcsShape* ssl,
                                          const RcsShape* sphere,
                                          const HTr* A_ssl,
                                          const HTr* A_sphere,
                                          double I_cp1[3],
                                          double I_cp2[3],
                                          double I_n[3])
{
  double d = sqrt(Math_sqrDistPointLineseg(A_sphere->org, A_ssl->org,
                                           A_ssl->rot[2], ssl->extents[2],
                                           I_cp1));
  Vec3d_copy(I_cp2, A_sphere->org);
  Vec3d_sub(I_n, I_cp2, I_cp1);
  Vec3d_normalizeSelf(I_n);

  // Surface points
  for (int i = 0; i < 3; ++i)
  {
    I_cp1[i] = I_cp1[i] + ssl->extents[0] * I_n[i];
    I_cp2[i] = I_cp2[i] - sphere->extents[0] * I_n[i];
  }
  return d - (ssl->extents[0] + sphere->extents[0]);
}

/*******************************************************************************
 * Computes the distance between a sphere and SSL primitive.
 ******************************************************************************/
static double RcsShape_closestSphereToSSL(const RcsShape* sphere,
                                          const RcsShape* ssl,
                                          const HTr* A_sphere,
                                          const HTr* A_ssl,
                                          double I_cp1[3],
                                          double I_cp2[3],
                                          double I_n[3])
{
  double dist = RcsShape_closestSSLToSphere(ssl, sphere, A_ssl, A_sphere,
                                            I_cp2, I_cp1, I_n);

  // Revert the normal, because we are calling the reverse method
  Vec3d_constMulSelf(I_n, -1.0);

  return dist;
}

/*******************************************************************************
 * Computes the distance between two SSL shape primitives.
 ******************************************************************************/
static double RcsShape_closestSSLToSSL(const RcsShape* ssl1,
                                       const RcsShape* ssl2,
                                       const HTr* A_ssl1I,
                                       const HTr* A_ssl2I,
                                       double I_cp1[3],
                                       double I_cp2[3],
                                       double I_n[3])
{
  return Math_distCapsuleCapsule(A_ssl1I->org,
                                 A_ssl1I->rot[2],
                                 ssl1->extents[2],
                                 ssl1->extents[0],
                                 A_ssl2I->org,
                                 A_ssl2I->rot[2],
                                 ssl2->extents[2],
                                 ssl2->extents[0],
                                 I_cp1, I_cp2, I_n);
}

/*******************************************************************************
 *
 ******************************************************************************/
static double RcsShape_closestPointToCone(const RcsShape* point,
                                          const RcsShape* cone,
                                          const HTr* A_point,
                                          const HTr* A_cone,
                                          double I_cp1[3],
                                          double I_cp2[3],
                                          double I_n[3])
{
  double dist = Math_distPointCone(A_point->org, A_cone, cone->extents[2],
                                   cone->extents[0], I_cp2);
  Vec3d_copy(I_cp1, A_point->org);
  Vec3d_sub(I_n, I_cp2, I_cp1);
  Vec3d_normalizeSelf(I_n);

  return dist;
}

/*******************************************************************************
 *
 ******************************************************************************/
static double RcsShape_closestConeToPoint(const RcsShape* cone,
                                          const RcsShape* point,
                                          const HTr* A_cone,
                                          const HTr* A_point,
                                          double I_cp1[3],
                                          double I_cp2[3],
                                          double I_n[3])
{
  double dist = RcsShape_closestPointToCone(point, cone, A_point, A_cone,
                                            I_cp2, I_cp1, I_n);

  // Revert the normal, because we are calling the reverse method
  Vec3d_constMulSelf(I_n, -1.0);

  return dist;
}

/*******************************************************************************
 * Computes the distance between a sphere and a cone shape primitives.
 ******************************************************************************/
static double RcsShape_closestSphereToCone(const RcsShape* sphere,
                                           const RcsShape* cone,
                                           const HTr* A_sphere,
                                           const HTr* A_cone,
                                           double I_cp1[3],
                                           double I_cp2[3],
                                           double I_n[3])
{
  double dist = RcsShape_closestPointToCone(sphere, cone, A_sphere, A_cone,
                                            I_cp1, I_cp2, I_n);

  // Point on sphere surface: cp_sphere = cp_point + radius_sphere*n
  Vec3d_constMulAndAddSelf(I_cp1, I_n, sphere->extents[0]);

  return dist - sphere->extents[0];
}

/*******************************************************************************
 * Computes the distance between a cone and a sphere shape primitives.
 ******************************************************************************/
static double RcsShape_closestConeToSphere(const RcsShape* cone,
                                           const RcsShape* sphere,
                                           const HTr* A_cone,
                                           const HTr* A_sphere,
                                           double I_cp2[3],
                                           double I_cp1[3],
                                           double I_n[3])
{
  double dist = RcsShape_closestSphereToCone(sphere, cone, A_sphere, A_cone,
                                             I_cp1, I_cp2, I_n);

  // revert the normal, because we are calling the reverse method
  Vec3d_constMulSelf(I_n, -1.0);
  return dist;
}


/*******************************************************************************
 *
 ******************************************************************************/
static double RcsShape_closestPointToCylinder(const RcsShape* point,
                                              const RcsShape* cyl,
                                              const HTr* A_point,
                                              const HTr* A_cyl,
                                              double I_cp1[3],
                                              double I_cp2[3],
                                              double I_n[3])
{
  double dist = Math_distPointCylinder(A_point->org, A_cyl, cyl->extents[2],
                                       cyl->extents[0], I_cp2);
  Vec3d_copy(I_cp1, A_point->org);
  Vec3d_sub(I_n, I_cp2, I_cp1);
  Vec3d_normalizeSelf(I_n);

  if (dist < 0.0)
  {
    Vec3d_constMulSelf(I_n, -1.0);
  }

  return dist;
}

/*******************************************************************************
 *
 ******************************************************************************/
static double RcsShape_closestCylinderToPoint(const RcsShape* cyl,
                                              const RcsShape* point,
                                              const HTr* A_cyl,
                                              const HTr* A_point,
                                              double I_cp1[3],
                                              double I_cp2[3],
                                              double I_n[3])
{
  double dist = RcsShape_closestPointToCylinder(point, cyl, A_point, A_cyl,
                                                I_cp2, I_cp1, I_n);

  // Revert the normal, because we are calling the reverse method
  Vec3d_constMulSelf(I_n, -1.0);

  return dist;
}

/*******************************************************************************
 * Computes the distance between a sphere and a cylinder shape primitives.
 ******************************************************************************/
static double RcsShape_closestSphereToCylinder(const RcsShape* sphere,
                                               const RcsShape* cylinder,
                                               const HTr* A_sphere,
                                               const HTr* A_cylinder,
                                               double I_cp1[3],
                                               double I_cp2[3],
                                               double I_n12[3])
{
  double dist = RcsShape_closestPointToCylinder(sphere, cylinder,
                                                A_sphere, A_cylinder,
                                                I_cp1, I_cp2, I_n12);

  // Point on sphere surface: cp_sphere = cp_point + radius_sphere*n
  Vec3d_constMulAndAddSelf(I_cp1, I_n12, sphere->extents[0]);

  return dist - sphere->extents[0];
}

/*******************************************************************************
 * Computes the distance between a cylinder and a sphere shape primitives.
 ******************************************************************************/
static double RcsShape_closestCylinderToSphere(const RcsShape* cylinder,
                                               const RcsShape* sphere,
                                               const HTr* A_cylinder,
                                               const HTr* A_sphere,
                                               double I_cp2[3],
                                               double I_cp1[3],
                                               double I_n[3])
{
  double dist = RcsShape_closestSphereToCylinder(sphere, cylinder, A_sphere,
                                                 A_cylinder, I_cp1, I_cp2, I_n);

  // revert the normal, because we are calling the reverse method
  Vec3d_constMulSelf(I_n, -1.0);
  return dist;
}

/*******************************************************************************
 *
 ******************************************************************************/
static double RcsShape_closestPointToBox(const RcsShape* point,
                                         const RcsShape* box,
                                         const HTr* A_point,
                                         const HTr* A_box,
                                         double I_cpPt[3],
                                         double I_cpBox[3],
                                         double I_nPtBox[3])
{
  double dist = Math_distPointBox(A_point->org, A_box, box->extents,
                                  I_cpBox, I_nPtBox);
  Vec3d_constMulSelf(I_nPtBox, -1.0);
  Vec3d_copy(I_cpPt, A_point->org);

  return dist;
}

/*******************************************************************************
 *
 ******************************************************************************/
static double RcsShape_closestBoxToPoint(const RcsShape* box,
                                         const RcsShape* point,
                                         const HTr* A_box,
                                         const HTr* A_point,
                                         double I_cp1[3],
                                         double I_cp2[3],
                                         double I_n[3])
{
  double dist = RcsShape_closestPointToBox(point, box, A_point, A_box,
                                           I_cp2, I_cp1, I_n);

  // Revert the normal, because we are calling the reverse method
  Vec3d_constMulSelf(I_n, -1.0);

  return dist;
}

/*******************************************************************************
 *
 ******************************************************************************/
static double RcsShape_closestSphereToBox(const RcsShape* sphere,
                                          const RcsShape* box,
                                          const HTr* A_sphere,
                                          const HTr* A_box,
                                          double I_cpSph[3],
                                          double I_cpBox[3],
                                          double I_nSphBox[3])
{
  double distBox = Math_distPointBox(A_sphere->org, A_box, box->extents,
                                     I_cpBox, I_nSphBox);
  Vec3d_constMulSelf(I_nSphBox, -1.0);
  Vec3d_copy(I_cpSph, A_sphere->org);

  // Point on sphere surface: cp_sphere = cp_point + radius_sphere*n
  Vec3d_constMulAndAddSelf(I_cpSph, I_nSphBox, sphere->extents[0]);

  double distSphere = distBox - sphere->extents[0];

  // In case the sphere radius leads to a penetration, we need to revert the
  // normal direction
  if ((distSphere < 0.0) && (distBox >= 0.0))
  {
    Vec3d_constMulSelf(I_nSphBox, -1.0);
  }

  return distSphere;
}

/*******************************************************************************
 *
 ******************************************************************************/
static double RcsShape_closestBoxToSphere(const RcsShape* box,
                                          const RcsShape* sphere,
                                          const HTr* A_box,
                                          const HTr* A_sphere,
                                          double I_cpBox[3],
                                          double I_cpSph[3],
                                          double I_nBoxSph[3])
{
  double dist = RcsShape_closestSphereToBox(sphere, box, A_sphere, A_box,
                                            I_cpSph, I_cpBox, I_nBoxSph);

  // Revert the normal, because we are calling the reverse method
  Vec3d_constMulSelf(I_nBoxSph, -1.0);

  return dist;
}

/*******************************************************************************
 * Computes the distance between a point and a SSR.
 ******************************************************************************/
static inline double RcsShape_closestPointToSSR(const RcsShape* pt,
                                                const RcsShape* ssr,
                                                const HTr* A_ptI,
                                                const HTr* A_ssrI,
                                                double I_cpPt[3],
                                                double I_cpSSR[3],
                                                double I_nPtSSR[3])
{
  double d = Math_sqrDistPointRect(A_ptI->org, A_ssrI, ssr->extents,
                                   I_cpSSR, I_nPtSSR);

  Vec3d_constMulSelf(I_nPtSSR, -1.0);

  // Surface point
  const double z = 0.5*ssr->extents[2];
  for (int i = 0; i < 3; i++)
  {
    I_cpSSR[i] -= z*I_nPtSSR[i];
  }

  Vec3d_copy(I_cpPt, A_ptI->org);

  return sqrt(d) - z;
}

/*******************************************************************************
 * SSR to Point distance computation.
 ******************************************************************************/
static inline double RcsShape_closestSSRToPoint(const RcsShape* ssr,
                                                const RcsShape* pt,
                                                const HTr* A_ssrI,
                                                const HTr* A_ptI,
                                                double cpSSR[3],
                                                double cpPt[3],
                                                double I_n[3])
{
  double dist = RcsShape_closestPointToSSR(pt, ssr, A_ptI, A_ssrI,
                                           cpPt, cpSSR, I_n);

  // revert the normal, because we are calling the reverse method
  Vec3d_constMulSelf(I_n, -1.0);

  return dist;
}

/*******************************************************************************
 * Look-up array for distance functions.
 ******************************************************************************/
static RcsDistanceFunction
RcsShapeDistFunc[RCSSHAPE_SHAPE_MAX][RCSSHAPE_SHAPE_MAX] =
{
  // RCSSHAPE_NONE
  {
    RcsShape_noDistance,                // 0  RCSSHAPE_NONE
    RcsShape_noDistance,                // 1  RCSSHAPE_SSL
    RcsShape_noDistance,                // 2  RCSSHAPE_SSR
    RcsShape_noDistance,                // 3  RCSSHAPE_MESH
    RcsShape_noDistance,                // 4  RCSSHAPE_BOX
    RcsShape_noDistance,                // 5  RCSSHAPE_CYLINDER
    RcsShape_noDistance,                // 6  RCSSHAPE_REFFRAME
    RcsShape_noDistance,                // 7  RCSSHAPE_SPHERE
    RcsShape_noDistance,                // 8  RCSSHAPE_CONE
    RcsShape_noDistance,                // 9  RCSSHAPE_TORUS
    RcsShape_noDistance,                // 10 RCSSHAPE_OCTREE
    RcsShape_noDistance                 // 11 RCSSHAPE_POINT
  },

  // RCSSHAPE_SSL
  {
    RcsShape_noDistance,                // 0  RCSSHAPE_NONE
    RcsShape_closestSSLToSSL,           // 1  RCSSHAPE_SSL
    RcsShape_noDistance,                // 2  RCSSHAPE_SSR
    RcsShape_noDistance,                // 3  RCSSHAPE_MESH
    RcsShape_noDistance,                // 4  RCSSHAPE_BOX
    RcsShape_noDistance,                // 5  RCSSHAPE_CYLINDER
    RcsShape_noDistance,                // 6  RCSSHAPE_REFFRAME
    RcsShape_closestSSLToSphere,        // 7  RCSSHAPE_SPHERE
    RcsShape_noDistance,                // 8  RCSSHAPE_CONE
    RcsShape_noDistance,                // 9  RCSSHAPE_TORUS
    RcsShape_noDistance,                // 10 RCSSHAPE_OCTREE
    RcsShape_closestSSLToPoint          // 11 RCSSHAPE_POINT
  },

  // RCSSHAPE_SSR
  {
    RcsShape_noDistance,                // 0  RCSSHAPE_NONE
    RcsShape_noDistance,                // 1  RCSSHAPE_SSL
    RcsShape_noDistance,                // 2  RCSSHAPE_SSR
    RcsShape_noDistance,                // 3  RCSSHAPE_MESH
    RcsShape_noDistance,                // 4  RCSSHAPE_BOX
    RcsShape_noDistance,                // 5  RCSSHAPE_CYLINDER
    RcsShape_noDistance,                // 6  RCSSHAPE_REFFRAME
    RcsShape_noDistance,                // 7  RCSSHAPE_SPHERE
    RcsShape_noDistance,                // 8  RCSSHAPE_CONE
    RcsShape_noDistance,                // 9  RCSSHAPE_TORUS
    RcsShape_noDistance,                // 10 RCSSHAPE_OCTREE
    RcsShape_closestSSRToPoint          // 11 RCSSHAPE_POINT
  },

  // RCSSHAPE_MESH
  {
    RcsShape_noDistance,                // 0  RCSSHAPE_NONE
    RcsShape_noDistance,                // 1  RCSSHAPE_SSL
    RcsShape_noDistance,                // 2  RCSSHAPE_SSR
    RcsShape_noDistance,                // 3  RCSSHAPE_MESH
    RcsShape_noDistance,                // 4  RCSSHAPE_BOX
    RcsShape_noDistance,                // 5  RCSSHAPE_CYLINDER
    RcsShape_noDistance,                // 6  RCSSHAPE_REFFRAME
    RcsShape_noDistance,                // 7  RCSSHAPE_SPHERE
    RcsShape_noDistance,                // 8  RCSSHAPE_CONE
    RcsShape_noDistance,                // 9  RCSSHAPE_TORUS
    RcsShape_noDistance,                // 10 RCSSHAPE_OCTREE
    RcsShape_noDistance                 // 11 RCSSHAPE_POINT
  },

  // RCSSHAPE_BOX
  {
    RcsShape_noDistance,                // 0  RCSSHAPE_NONE
    RcsShape_noDistance,                // 1  RCSSHAPE_SSL
    RcsShape_noDistance,                // 2  RCSSHAPE_SSR
    RcsShape_noDistance,                // 3  RCSSHAPE_MESH
    RcsShape_noDistance,                // 4  RCSSHAPE_BOX
    RcsShape_noDistance,                // 5  RCSSHAPE_CYLINDER
    RcsShape_noDistance,                // 6  RCSSHAPE_REFFRAME
    RcsShape_closestBoxToSphere,        // 7  RCSSHAPE_SPHERE
    RcsShape_noDistance,                // 8  RCSSHAPE_CONE
    RcsShape_noDistance,                // 9  RCSSHAPE_TORUS
    RcsShape_noDistance,                // 10 RCSSHAPE_OCTREE
    RcsShape_closestBoxToPoint          // 11 RCSSHAPE_POINT
  },

  // RCSSHAPE_CYLINDER
  {
    RcsShape_noDistance,                // 0  RCSSHAPE_NONE
    RcsShape_noDistance,                // 1  RCSSHAPE_SSL
    RcsShape_noDistance,                // 2  RCSSHAPE_SSR
    RcsShape_noDistance,                // 3  RCSSHAPE_MESH
    RcsShape_noDistance,                // 4  RCSSHAPE_BOX
    RcsShape_noDistance,                // 5  RCSSHAPE_CYLINDER
    RcsShape_noDistance,                // 6  RCSSHAPE_REFFRAME
    RcsShape_closestCylinderToSphere,   // 7  RCSSHAPE_SPHERE
    RcsShape_noDistance,                // 8  RCSSHAPE_CONE
    RcsShape_noDistance,                // 9  RCSSHAPE_TORUS
    RcsShape_noDistance,                // 10 RCSSHAPE_OCTREE
    RcsShape_closestCylinderToPoint     // 11 RCSSHAPE_POINT
  },

  // RCSSHAPE_REFFRAME
  {
    RcsShape_noDistance,                // 0  RCSSHAPE_NONE
    RcsShape_noDistance,                // 1  RCSSHAPE_SSL
    RcsShape_noDistance,                // 2  RCSSHAPE_SSR
    RcsShape_noDistance,                // 3  RCSSHAPE_MESH
    RcsShape_noDistance,                // 4  RCSSHAPE_BOX
    RcsShape_noDistance,                // 5  RCSSHAPE_CYLINDER
    RcsShape_noDistance,                // 6  RCSSHAPE_REFFRAME
    RcsShape_noDistance,                // 7  RCSSHAPE_SPHERE
    RcsShape_noDistance,                // 8  RCSSHAPE_CONE
    RcsShape_noDistance,                // 9  RCSSHAPE_TORUS
    RcsShape_noDistance,                // 10 RCSSHAPE_OCTREE
    RcsShape_noDistance                 // 11 RCSSHAPE_POINT
  },

  // RCSSHAPE_SPHERE
  {
    RcsShape_noDistance,                // 0  RCSSHAPE_NONE
    RcsShape_closestSphereToSSL,        // 1  RCSSHAPE_SSL
    RcsShape_noDistance,                // 2  RCSSHAPE_SSR
    RcsShape_noDistance,                // 3  RCSSHAPE_MESH
    RcsShape_closestSphereToBox,        // 4  RCSSHAPE_BOX
    RcsShape_closestSphereToCylinder,   // 5  RCSSHAPE_CYLINDER
    RcsShape_noDistance,                // 6  RCSSHAPE_REFFRAME
    RcsShape_closestSphereToSphere,     // 7  RCSSHAPE_SPHERE
    RcsShape_closestSphereToCone,       // 8  RCSSHAPE_CONE
    RcsShape_noDistance,                // 9  RCSSHAPE_TORUS
    RcsShape_noDistance,                // 10 RCSSHAPE_OCTREE
    RcsShape_closestSphereToPoint       // 11 RCSSHAPE_POINT
  },

  // RCSSHAPE_CONE
  {
    RcsShape_noDistance,                // 0  RCSSHAPE_NONE
    RcsShape_noDistance,                // 1  RCSSHAPE_SSL
    RcsShape_noDistance,                // 2  RCSSHAPE_SSR
    RcsShape_noDistance,                // 3  RCSSHAPE_MESH
    RcsShape_noDistance,                // 4  RCSSHAPE_BOX
    RcsShape_noDistance,                // 5  RCSSHAPE_CYLINDER
    RcsShape_noDistance,                // 6  RCSSHAPE_REFFRAME
    RcsShape_closestConeToSphere,       // 7  RCSSHAPE_SPHERE
    RcsShape_noDistance,                // 8  RCSSHAPE_CONE
    RcsShape_noDistance,                // 9  RCSSHAPE_TORUS
    RcsShape_noDistance,                // 10 RCSSHAPE_OCTREE
    RcsShape_closestConeToPoint         // 11 RCSSHAPE_POINT
  },

  // RCSSHAPE_TORUS
  {
    RcsShape_noDistance,                // 0  RCSSHAPE_NONE
    RcsShape_noDistance,                // 1  RCSSHAPE_SSL
    RcsShape_noDistance,                // 2  RCSSHAPE_SSR
    RcsShape_noDistance,                // 3  RCSSHAPE_MESH
    RcsShape_noDistance,                // 4  RCSSHAPE_BOX
    RcsShape_noDistance,                // 5  RCSSHAPE_CYLINDER
    RcsShape_noDistance,                // 6  RCSSHAPE_REFFRAME
    RcsShape_noDistance,                // 7  RCSSHAPE_SPHERE
    RcsShape_noDistance,                // 8  RCSSHAPE_CONE
    RcsShape_noDistance,                // 9  RCSSHAPE_TORUS
    RcsShape_noDistance,                // 10 RCSSHAPE_OCTREE
    RcsShape_noDistance                 // 11 RCSSHAPE_POINT
  },

  // RCSSHAPE_OCTREE
  {
    RcsShape_noDistance,                // 0  RCSSHAPE_NONE
    RcsShape_noDistance,                // 1  RCSSHAPE_SSL
    RcsShape_noDistance,                // 2  RCSSHAPE_SSR
    RcsShape_noDistance,                // 3  RCSSHAPE_MESH
    RcsShape_noDistance,                // 4  RCSSHAPE_BOX
    RcsShape_noDistance,                // 5  RCSSHAPE_CYLINDER
    RcsShape_noDistance,                // 6  RCSSHAPE_REFFRAME
    RcsShape_noDistance,                // 7  RCSSHAPE_SPHERE
    RcsShape_noDistance,                // 8  RCSSHAPE_CONE
    RcsShape_noDistance,                // 9  RCSSHAPE_TORUS
    RcsShape_noDistance,                // 10 RCSSHAPE_OCTREE
    RcsShape_noDistance,                // 11 RCSSHAPE_POINT
  },

  // RCSSHAPE_POINT
  {
    RcsShape_noDistance,                // 0  RCSSHAPE_NONE
    RcsShape_closestPointToSSL,         // 1  RCSSHAPE_SSL
    RcsShape_closestPointToSSR,         // 2  RCSSHAPE_SSR
    RcsShape_noDistance,                // 3  RCSSHAPE_MESH
    RcsShape_closestPointToBox,         // 4  RCSSHAPE_BOX
    RcsShape_closestPointToCylinder,    // 5  RCSSHAPE_CYLINDER
    RcsShape_noDistance,                // 6  RCSSHAPE_REFFRAME
    RcsShape_closestPointToSphere,      // 7  RCSSHAPE_SPHERE
    RcsShape_closestPointToCone,        // 8  RCSSHAPE_CONE
    RcsShape_noDistance,                // 9  RCSSHAPE_TORUS
    RcsShape_noDistance,                // 10 RCSSHAPE_OCTREE
    RcsShape_closestPointToPoint        // 11 RCSSHAPE_POINT
  }

};

/*******************************************************************************
 * Distance function pointer access.
 ******************************************************************************/
RcsDistanceFunction RcsShape_getDistanceFunction(unsigned int shapeTypeIdx1,
                                                 unsigned int shapeTypeIdx2)
{
  if ((shapeTypeIdx1>=RCSSHAPE_SHAPE_MAX) ||
      (shapeTypeIdx2>=RCSSHAPE_SHAPE_MAX))
  {
    RLOG(4, "Shape index out of range: %d %d must be less than %d",
         shapeTypeIdx1, shapeTypeIdx2, RCSSHAPE_SHAPE_MAX);
    return NULL;
  }

  RcsDistanceFunction fnc = RcsShapeDistFunc[shapeTypeIdx1][shapeTypeIdx2];

  if (fnc==RcsShape_noDistance)
  {
    return NULL;
  }

  return RcsShapeDistFunc[shapeTypeIdx1][shapeTypeIdx2];
}

/*******************************************************************************
 *
 ******************************************************************************/
bool RcsShape_hasDistanceFunction(unsigned int shapeTypeIdx1,
                                  unsigned int shapeTypeIdx2)
{
  RcsDistanceFunction func = RcsShape_getDistanceFunction(shapeTypeIdx1,
                                                          shapeTypeIdx2);

  return (func==NULL) ? false : true;
}

/*******************************************************************************
 * Compute the closest distance via function table lookup.
 ******************************************************************************/
double RcsShape_distance(const RcsShape* s1,
                         const RcsShape* s2,
                         const HTr* A_B1I,
                         const HTr* A_B2I,
                         double I_cp1_[3],
                         double I_cp2_[3],
                         double I_n_[3])
{
  // Get distance function pointer for shape pair
  RcsDistanceFunction func = RcsShapeDistFunc[s1->type][s2->type];

  // Compute the shapes world transformation
  // A_CI  = A_CB * A_BI
  // I_r_C = I_r_B + A_IB * B_r_BC
  HTr A_C1I, A_C2I;
  HTr_transform(&A_C1I, A_B1I, &s1->A_CB);
  HTr_transform(&A_C2I, A_B2I, &s2->A_CB);

  // We create local arrays for closest points and normals here for two
  // reasons. Firstly, we enable to call this function with NULL arguments.
  // Secondly, we make sure that the function can be called with the same
  // argument arrays without breaking the computation, e.g.
  // RcsShape_distance(..., tmp, tmp, tmp);
  double I_cp1[3], I_cp2[3], I_n[3];
  double d = func(s1, s2, &A_C1I, &A_C2I, I_cp1, I_cp2, I_n);

  if (I_cp1_)
  {
    Vec3d_copy(I_cp1_, I_cp1);
  }

  if (I_cp2_)
  {
    Vec3d_copy(I_cp2_, I_cp2);
  }

  if (I_n_)
  {
    Vec3d_copy(I_n_, I_n);
  }

  return d;
}

/*******************************************************************************
 * Compute the closest distance of a shape to a point.
 ******************************************************************************/
double RcsShape_distanceToPoint(const RcsShape* shape,
                                const HTr* A_BI,
                                const double I_pt[3],
                                double I_cpShape[3],
                                double I_nShapePt[3])
{
  RcsShape ptShape;
  memset(&ptShape, 0, sizeof(RcsShape));
  ptShape.type = RCSSHAPE_POINT;
  Mat3d_setIdentity(ptShape.A_CB.rot);
  Vec3d_copy(ptShape.A_CB.org, I_pt);
  Vec3d_setElementsTo(ptShape.scale3d, 1.0);
  ptShape.computeType = RCSSHAPE_COMPUTE_DISTANCE;

  double tmp1[3], tmp2[3], tmp3[3];
  double d = RcsShape_distance(shape, &ptShape, A_BI, HTr_identity(),
                               tmp1, tmp2, tmp3);

  if (I_cpShape)
  {
    Vec3d_copy(I_cpShape, tmp1);
  }

  if (I_nShapePt)
  {
    Vec3d_copy(I_nShapePt, tmp3);
  }

  return d;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool RcsShape_setDistanceFunction(unsigned int shapeTypeIdx1,
                                  unsigned int shapeTypeIdx2,
                                  RcsDistanceFunction func)
{
  if (shapeTypeIdx1>RCSSHAPE_SHAPE_MAX-1)
  {
    RLOG(4, "shapeTypeIdx1 (%d) > RCSSHAPE_SHAPE_MAX-1 (%d) ",
         shapeTypeIdx1, RCSSHAPE_SHAPE_MAX-1);
    return false;
  }

  if (shapeTypeIdx2>RCSSHAPE_SHAPE_MAX-1)
  {
    RLOG(4, "shapeTypeIdx2 (%d) > RCSSHAPE_SHAPE_MAX-1 (%d) ",
         shapeTypeIdx2, RCSSHAPE_SHAPE_MAX-1);
    return false;
  }

  RcsShapeDistFunc[shapeTypeIdx1][shapeTypeIdx2] = func;

  return true;
}

/*******************************************************************************
 * Prints the distance function array to a file
 ******************************************************************************/
void RcsShape_fprintDistanceFunctions(FILE* out)
{
  size_t nBytes = 0;

  for (size_t i=0; i<RCSSHAPE_SHAPE_MAX; ++i)
  {
    const char* name_i = RcsShape_name(i);
    size_t nameLen = strlen(name_i);
    if (nameLen > nBytes)
    {
      nBytes = nameLen;
    }
  }
  nBytes += 4;

  for (size_t i=0; i<nBytes+2; ++i)
  {
    fprintf(out, " ");
  }

  for (int i=0; i<RCSSHAPE_SHAPE_MAX; ++i)
  {
    if ((i==RCSSHAPE_NONE) || (i==RCSSHAPE_REFFRAME))
    {
      continue;
    }
    fprintf(out, "%3d", i);
  }
  fprintf(out, "\n");


  for (int i=0; i<RCSSHAPE_SHAPE_MAX; ++i)
  {
    if ((i==RCSSHAPE_NONE) || (i==RCSSHAPE_REFFRAME))
    {
      continue;
    }
    const char* name_i = RcsShape_name(i);
    fprintf(out, "%2d %s", i, name_i);
    for (size_t k=0; k<nBytes-strlen(name_i); ++k)
    {
      fprintf(out, " ");
    }

    for (int j=0; j<RCSSHAPE_SHAPE_MAX; ++j)
    {
      if ((j==RCSSHAPE_NONE) || (j==RCSSHAPE_REFFRAME))
      {
        continue;
      }

      fprintf(out, "%s",
              RcsShapeDistFunc[i][j]==RcsShape_noDistance ? " - " : " o ");
    }
    fprintf(out, "\n");
  }




  /*
  for (int i=0; i<RCSSHAPE_SHAPE_MAX; ++i)
  {
   if ((i==RCSSHAPE_NONE) || (i==RCSSHAPE_REFFRAME))
   {
     continue;
   }

   for (int j=0; j<RCSSHAPE_SHAPE_MAX; ++j)
   {
     if ((j==RCSSHAPE_NONE) || (j==RCSSHAPE_REFFRAME))
     {
       continue;
     }
     unsigned int baseAddr = (unsigned int) RcsShapeDistFunc[0][0];
     unsigned int fcnAddr = (unsigned int) RcsShapeDistFunc[i][j];
     fprintf(out, "%d-%d: %u\n", i, j, fcnAddr-baseAddr);
   }
   fprintf(out, "\n");
  }
  */

}

/*******************************************************************************
 *
 ******************************************************************************/
void* RcsShape_addOctree(RcsShape* self, const char* fileName)
{
  self->mesh = (RcsMeshData*)Rcs_loadOctree(self->meshFile);

  if (self->mesh == NULL)
  {
    RLOG(1, "Failed to load Octree file \"%s\"", self->meshFile);
  }

  return self->mesh;
}

/*******************************************************************************
 *
 ******************************************************************************/
void RcsShape_computeAABB(const RcsShape* shape,
                          double xyzMin[3], double xyzMax[3])
{
  const double* e = shape->extents;

  switch (shape->type)
  {
    case RCSSHAPE_NONE:
    case RCSSHAPE_REFFRAME:
    case RCSSHAPE_POINT:
    {
      Vec3d_setZero(xyzMin);
      Vec3d_setZero(xyzMax);
      break;
    }
    case RCSSHAPE_SSL:
    {
      // Radius: extents[0], height: extents[2]. The reference point is a
      // ball point, the line direction is z.
      Vec3d_set(xyzMin, -e[0], -e[0], -e[0]);
      Vec3d_set(xyzMax, e[0], e[0], e[2] + e[0]);
      break;
    }
    case RCSSHAPE_SSR:
    {
      Vec3d_set(xyzMin, -0.5*e[0] - 0.5*e[2], -0.5*e[1] - 0.5*e[2], -0.5*e[2]);
      Vec3d_set(xyzMax, 0.5*e[0] + 0.5*e[2], 0.5*e[1] + 0.5*e[2], 0.5*e[2]);
      break;
    }
    case RCSSHAPE_MESH:
    {
      RcsMesh_computeAABB(shape->mesh, xyzMin, xyzMax);
      break;
    }
    case RCSSHAPE_BOX:
    {
      Vec3d_constMul(xyzMin, e, -0.5);
      Vec3d_constMul(xyzMax, e, 0.5);
      break;
    }
    case RCSSHAPE_CYLINDER:
    {
      Vec3d_set(xyzMin, -e[0], -e[0], -0.5*e[2]);
      Vec3d_set(xyzMax, e[0], e[0], 0.5*e[2]);
      break;
    }
    case RCSSHAPE_SPHERE:
    {
      Vec3d_set(xyzMin, -e[0], -e[0], -e[0]);
      Vec3d_set(xyzMax, e[0], e[0], e[0]);
      break;
    }
    case RCSSHAPE_CONE:
    {
      // The cone's reference point is the base plane.
      Vec3d_set(xyzMin, -e[0], -e[0], 0.0);
      Vec3d_set(xyzMax, e[0], e[0], e[2]);
      break;
    }
    case RCSSHAPE_TORUS:
    {
      Vec3d_set(xyzMin, -e[0]-0.5*e[2], -e[0]-0.5*e[2], -0.5*e[2]);
      Vec3d_set(xyzMax, e[0]+0.5*e[2], e[0]+0.5*e[2], 0.5*e[2]);
      break;
    }

    default:
    {
      RFATAL("Unsupported shape type \"%s\"", RcsShape_name(shape->type));
    }
  }
}

/*******************************************************************************
 * origin is represented in the bodie's frame.
 ******************************************************************************/
void RcsShape_scale(RcsShape* shape, double scale)
{
  Vec3d_constMulSelf(shape->A_CB.org, scale);
  Vec3d_constMulSelf(shape->extents, scale);
  Vec3d_constMulSelf(shape->scale3d, scale);

  if ((shape->type==RCSSHAPE_MESH) && shape->mesh)
  {
    RcsMesh_scale(shape->mesh, scale);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void RcsShape_setComputeType(RcsShape* shape, int computeType, bool enable)
{
  if (enable)
  {
    shape->computeType |= computeType;
  }
  else
  {
    shape->computeType &= ~(computeType);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
bool RcsShape_isOfComputeType(const RcsShape* shape, int computeType)
{
  if ((shape->computeType & computeType) == 0)
  {
    return false;
  }

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool RcsShape_isEqual(const RcsShape* s1, const RcsShape* s2)
{
  // We compare the shape's contents except for the pointer to the mesh
  if (memcmp(s1, s2, sizeof(RcsShape) - sizeof(RcsMeshData*)) != 0)
  {
    return false;
  }

  return RcsMesh_isEqual(s1->mesh, s2->mesh);   // Can deal with NULL pointers
}
