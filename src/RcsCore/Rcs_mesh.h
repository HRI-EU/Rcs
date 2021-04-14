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

#ifndef RCS_MESH_H
#define RCS_MESH_H


#include "Rcs_bool.h"

#include <stdio.h>


#ifdef __cplusplus
extern "C" {
#endif


typedef struct
{
  unsigned int nVertices;
  double* vertices;
  unsigned int nFaces;
  unsigned int* faces;

} RcsMeshData;


/*! \ingroup RcsMeshFunctions
 *  \brief Reads the given mesh file and copies the vertex and face data into
 *         the argument meshData. The caller is responsible to destroy the
 *         returned memory, including vertices and faces arrays. Currently, the
 *         supported file formats are stl (both binary and ASCII) and tri.
 *
 *  \param[in] fileName    Absolute file name of the mesh file.
 *  \param[out] meshData   See the above data structure
 *  \return True for success, false for failure.
 */
bool RcsMesh_readFromFile(const char* fileName, RcsMeshData* meshData);

/*! \ingroup RcsMeshFunctions
 *  \brief Reads a mesh file (stl and tri). The caller is responsible to destroy
 *         the returned memory pointed to by vertices and faces. If argument
 *         meshFile is NULL, the file cannot be opened, or the file cannot be
 *         parsed for some reason, the function returns NULL.
 *
 *  \param[in] fileName    Absolute file name of the mesh file.
 *  \return RcsMeshData structure, or NULL for failure.
 */
RcsMeshData* RcsMesh_createFromFile(const char* fileName);

/*! \ingroup RcsMeshFunctions
 *  \brief Prints out the mesh vertex and face lists to stdout.
 *
 *  \param[in] mesh Valid mesh data. If the argument is NULL, a warning is
 *             emitted on debug level 4, and nothing is printed.
 */
void RcsMesh_print(const RcsMeshData* mesh);

/*! \ingroup RcsMeshFunctions
 *  \brief Prints out the mesh vertex and face lists to a file.
 *
 *  \param[in] out Valid file descriptor (not checked).
 *  \param[in] mesh Valid mesh data. If the argument is NULL, a warning is
 *             emitted on debug level 4, and nothing is printed.
 */
void RcsMesh_fprint(FILE* out, const RcsMeshData* mesh);

/*! \ingroup RcsMeshFunctions
 *  \brief Performs some checks on the mesh:
 *         - Largest face index < number of vertices
 *         - The three face indices of each single face must be different
 *
 *  \param[in] mesh Mesh data to be checked
 *  \return true for passed, false for something is wrong. The reason is
 *          printed on debug level 4.
 */
bool RcsMesh_check(const RcsMeshData* mesh);

/*! \ingroup RcsMeshFunctions
 *  \brief Creates a deep copy of the argument mesh. If mesh is NULL, the
 *         function returns NULL and a warning is emitted on debug level 4.
 *
 *  \param[in] mesh Mesh data
 */
RcsMeshData* RcsMesh_clone(const RcsMeshData* mesh);

/*! \ingroup RcsMeshFunctions
 *  \brief Makes a deep copy of src into dst. The vertex and face arrays will
 *         be re-allocated in case the memory of the src mesh is not large
 *         enough. The function will not re-allocate memory if the size of the
 *         src mesh arrays are smaller than the ones of dst.
 *
 *  \param[in,out] dst    Mesh data the src mesh will be copied to. Must not
 *                        be NULL.
 *  \param[in]     src    Mesh data to be copied from. Must not be NULL.
 */
void RcsMesh_copy(RcsMeshData* dst, const RcsMeshData* src);

/*! \ingroup RcsMeshFunctions
 *  \brief Deletes all created memory.
 *
 *  \param[in] mesh Mesh data. If mesh is NULL, nothing is done.
 */
void RcsMesh_destroy(RcsMeshData* mesh);

/*! \ingroup RcsMeshFunctions
 *  \brief Reverts the order of the face indices
 *
 *  \param[in] mesh Mesh data. If it is invalid, the behavior is undefined.
 */
void RcsMesh_flipNormals(RcsMeshData* mesh);

/*! \ingroup RcsMeshFunctions
 *  \brief Computes the volume of a triangle mesh. It is not required that it
 *         is convex. If argument mesh is NULL, the function returns 0.0.
 *
 *  \param[in] mesh Mesh data. If it is invalid, the behavior is undefined.
 *  \return Volume of the mesh.
 */
double RcsMesh_computeVolume(RcsMeshData* mesh);

/*! \ingroup RcsMeshFunctions
 *  \brief Creates a mesh from a set of vertices. This function computes the
 *         Delaunay Triangulation of the vertex set. It only works if the
 *         GeometricTools library is enabled. Otherwise, always NULL is
 *         returned.
 *
 *  \param[in] vertices     Contiguous array of double values holding the vertex
 *                          coordinates
 *  \param[in] numVertices  Number of vertices
 *  \return Mesh on success, NULL on failure. The reason of failure is outputted
 *          on debug level 4.
 */
RcsMeshData* RcsMesh_fromVertices(const double* vertices,
                                  unsigned int numVertices);

/*! \ingroup RcsMeshFunctions
 *  \brief Writes the mesh to the given file name. Depending on the file suffix,
 *         the mesh is written in the corresponding file format. Currently
 *         these formats are supported:
 *         - .stl
 *         - .obj
 *         - .tri
 *
 *  \param[in] mesh       Mesh data
 *  \param[in] fileName   Name of the output file
 *  \return True on success, false otherwise. The reason of failure is outputted
 *          on debug level 4.
 */
bool RcsMesh_toFile(const RcsMeshData* mesh, const char* fileName);

/*! \ingroup RcsMeshFunctions
 *  \brief This function removes the duplicates and adjusts the corresponding
 *         face index. A vertex is considered to be duplicate if its Euclidean
 *         distance to another vertex is less than eps.
 *
 *  \param[in] mesh       Mesh data
 *  \param[in] eps        Distance threshold below which two vertices are
 *                        considered as one.
 *  \return -1 on failure, otherwise the number of compressed vertices. The
 *          reason of failure is outputted on debug level 4.
 */
int RcsMesh_compressVertices(RcsMeshData* mesh, double eps);

/*! \ingroup RcsMeshFunctions
 *  \brief This function computes the axis-aligned bounding box of a mesh.
 *
 *  \param[in] mesh       Mesh data. If it is NULL, the AABB is set to zero,
 * *                      size, and a debug message is issued on debul level 4.
 *  \param[in] xyzMin     Minimum point of the box
 *  \param[in] xyzMax     Maximum point of the box
 */
void RcsMesh_computeAABB(const RcsMeshData* mesh,
                         double xyzMin[3], double xyzMax[3]);

/*! \ingroup RcsMeshFunctions
 *  \brief This function scales all vertices of the mesh with the given scale
 *         factor.
 *
 *  \param[in] mesh       Mesh data, must be not NULL.
 *  \param[in] scale      Scaling factor. All vertices are multiplied with
 *                        this value.
 */
void RcsMesh_scale(RcsMeshData* mesh, double scale);

/*! \ingroup RcsMeshFunctions
 *  \brief This function scales all vertices of the mesh with the given scale
 *         factors for each element.
 *
 *  \param[in] mesh       Mesh data, must be not NULL.
 *  \param[in] scale      Scaling factors for x, y, and z-direction. All
 *                        vertices are multiplied with the corresponding value.
 */
void RcsMesh_scale3D(RcsMeshData* mesh, const double scale[3]);

/*! \ingroup RcsMeshFunctions
 *  \brief Adds a mesh to another one.
 *
 *  \param[in] mesh       Mesh data to be extended by other.
 *  \param[in] other      Other mesh to be added.
 */
void RcsMesh_add(RcsMeshData* mesh, const RcsMeshData* other);

/*! \ingroup RcsMeshFunctions
 *  \brief Shifts all vertices of a mesh by the given offset
 *
 *  \param[in] mesh  Mesh data to be shifted.
 *  \param[in] x     Offset x in whatever coordinates the mesh is represented.
 *  \param[in] y     Offset y.
 *  \param[in] z     Offset z.
 */
void RcsMesh_shift(RcsMeshData* mesh, double x, double y, double z);

/*! \ingroup RcsMeshFunctions
 *  \brief Applies a rotation to a mesh.
 *
 *  \param[in] mesh  Mesh data to be rotated.
 *  \param[in] A_MI  Rotation matrix from I(nertial) to M(esh) frame.
 */
void RcsMesh_rotate(RcsMeshData* mesh, double A_MI[3][3]);

/*! \ingroup RcsMeshFunctions
 *  \brief Applies a transformation to a mesh.
 *
 *  \param[in] mesh  Mesh data to be rotated.
 *  \param[in] pos   Translation.
 *  \param[in] A_MI  Rotation matrix from I(nertial) to M(esh) frame.
 */
void RcsMesh_transform(RcsMeshData* mesh, const double pos[3],
                       double A_MI[3][3]);

/*! \ingroup RcsMeshFunctions
 *  \brief Creates a mesh for a box with the given extents.
 *
 *  \param[in] extents   Box extents
 *  \return Box mesh.
 */
RcsMeshData* RcsMesh_createBox(const double extents[3]);

/*! \ingroup RcsMeshFunctions
 *  \brief Creates the cylindrical envelope of a cylinder without the caps.
 *         This function is based on three.js library licensed under the
 *         MIT license.
 *
 *  \param[in] radiusBottom   Radius at the bottom of the cylinder
 *  \param[in] radiusTop      Radius at the top of the cylinder
 *  \param[in] height         Height of the cylinder
 *  \param[in] radialSegments Number of segments around radius
 *  \param[in] heightSegments Number of slices over height
 *  \param[in] angleAround    Angle of envelope (Pi for closed cylinder)
 *  \return Triangle mesh according to dimensions. The caller is responsible to
 *          delete the memory.
 */
RcsMeshData* RcsMesh_createCylinderHull(double radiusBottom,
                                        double radiusTop,
                                        double height,
                                        unsigned int radialSegments,
                                        unsigned int heightSegments,
                                        double angleAround);

/*! \ingroup RcsMeshFunctions
 *  \brief Creates a cylinder mesh including the caps.
 *         This function is based on three.js library licensed under the
 *         MIT license.
 *
 *  \param[in] radius      Radius at the top of the cylinder
 *  \param[in] height      Height of the cylinder
 *  \param[in] segments    Number of radial segments
 *  \return Triangle mesh according to dimensions. The caller is responsible to
 *          delete the memory.
 */
RcsMeshData* RcsMesh_createCylinder(double radius, double height,
                                    unsigned int segments);

/*! \ingroup RcsMeshFunctions
 *  \brief Creates a sphere segment.
 *         This function is based on three.js library licensed under the
 *         MIT license.
 *
 *  \param[in] radius           Sphere radius
 *  \param[in] heightSegments   Number of slices over height
 *  \param[in] widthSegments    Number of segments around radius
 *  \param[in] phiStart         Vertical start angle slice
 *  \param[in] phiLength        Vertical angle (Pi for complete sphere)
 *  \param[in] thetaStart       Radial start angle slice
 *  \param[in] thetaLength      Radial angle (Pi for complete sphere)
 *  \return Triangle mesh according to dimensions. The caller is responsible to
 *          delete the memory.
 */
RcsMeshData* RcsMesh_createSphereSegment(double radius,
                                         unsigned int heightSegments,
                                         unsigned int widthSegments,
                                         double phiStart,
                                         double phiLength,
                                         double thetaStart,
                                         double thetaLength);

/*! \ingroup RcsMeshFunctions
 *  \brief Creates a sphere.
 *
 *  \param[in] radius           Sphere radius
 *  \param[in] segments         Tesselation
 *  \return Triangle mesh according to dimensions. The caller is responsible to
 *          delete the memory.
 */
RcsMeshData* RcsMesh_createSphere(double radius, unsigned int segments);

/*! \ingroup RcsMeshFunctions
 *  \brief Creates a capsule.
 *
 *  \param[in] radius           Capsule radius
 *  \param[in] height           Distance of ball points
 *  \param[in] segments         Tesselation
 *  \return Triangle mesh according to dimensions. The caller is responsible to
 *          delete the memory.
 */
RcsMeshData* RcsMesh_createCapsule(double radius, double height,
                                   unsigned int segments);

/*! \ingroup RcsMeshFunctions
 *  \brief Copies a capsule into the given mesh. The passed mesh must have
 *         enough memory for vertices and faces as required, this is not being
 *         checked.
 *
 *  \param[in] mesh             Mesh data or NULl, see above.
 *  \param[in] radius           Capsule radius
 *  \param[in] height           Distance of ball points
 *  \param[in] segments         Tesselation
 */
void RcsMesh_copyCapsule(RcsMeshData* mesh, double radius,
                         double height, unsigned int segments);

/*! \ingroup RcsMeshFunctions
 *  \brief Creates a cone with the given dimensions. The cone axis is aligned
 *         with the z-axis, the bottom disk is at z=0.
 *
 *  \param[in] radius           Capsule radius
 *  \param[in] height           Distance of ball points
 *  \param[in] segments         Tesselation
 *  \return Triangle mesh according to dimensions. The caller is responsible to
 *          delete the memory.
 */
RcsMeshData* RcsMesh_createCone(double radius, double height,
                                unsigned int segments);

/*! \ingroup RcsMeshFunctions
 *  \brief Creates a triangle mesh of a torus.
 *
 *  \param[in] radius            Radius about the torus symmetry axis
 *  \param[in] thickness         Thickness, or diameter of swept circle.
 *  \param[in] radialSegments    Number of segments around swept circle.
 *  \param[in] tubularSegments   Number of slices of the torus ring.
 *  \return Triangle mesh according to dimensions. The caller is responsible to
 *          delete the memory.
 */
RcsMeshData* RcsMesh_createTorus(double radius, double thickness,
                                 unsigned int radialSegments,
                                 unsigned int tubularSegments);

/*! \ingroup RcsMeshFunctions
 *  \brief Creates a triangle mesh of a sphere-swept rectangle.
 *
 *  \param[in] extents           Dimensions. The third element is the thickness.
 *  \param[in] segments          Number of segments around curves.
 *  \return Triangle mesh according to dimensions. The caller is responsible to
 *          delete the memory.
 */
RcsMeshData* RcsMesh_createSSR(const double extents[3], unsigned int segments);

/*! \ingroup RcsMeshFunctions
 *  \brief Copies a sphere-swept rectangle into the given mesh. The passed mesh
 *         must have enough memory for vertices and faces as required, this is
 *         not being checked.
 *
 *  \param[in] mesh             Mesh data or NULl, see above.
 *  \param[in] extents          Dimensions, see RCSSHAPE_SSR
 *  \param[in] segments         Tesselation
 */
void RcsMesh_copySSR(RcsMeshData* mesh, const double extents[3],
                     unsigned int segments);

/*! \ingroup RcsMeshFunctions
 *  \brief Creates a four-sided pyramid with side lengths y and y, and height z.
 *
 *  \param[in] x   Side length 1 of base rectangle
 *  \param[in] y   Side length 2 of base rectangle
 *  \param[in] z   Height of pyramid
 *  \return Pyramid mesh according to dimensions. The caller is responsible to
 *          delete the memory.
 */
RcsMeshData* RcsMesh_createPyramid(double x, double y, double z);

/*! \ingroup RcsMeshFunctions
 *  \brief Creates a four-sided view frustum with the origin at (0 0 0),
 *         pointing into the z-direction. It is actually not a real frustum,
 *         but an inverted four-sided pyramid.
 *
 *  \param[in] fovX   Field of view / opening angle in x-direction
 *  \param[in] fovY   Field of view / opening angle in x-direction
 *  \param[in] h      Height of frustum
 *  \return Frustum mesh according to dimensions. The caller is responsible to
 *          delete the memory.
 */
RcsMeshData* RcsMesh_createFrustum(double fovX, double fovY, double h);

/*! \ingroup RcsMeshFunctions
 *  \brief Computes the mesh's center of mass and the inertia tensor around the
 *         COM. If argument mesh is NULL, both I and com are set to zero.
 *
 *  This code is extract from:
 *  "Polyhedral Mass Properties (Revisited)" by David Eberly
 *   http://www.geometrictools.com/Documentation/PolyhedralMassProperties.pdf
 *  The implementation can be found in external/Rcs_thirdPartyMath.c
 *
 *  \param[in] mesh Mesh data. If it is invalid, the behavior is undefined.
 *  \param[out] I   Inertia tensor around COM
 *  \param[out] com Center of mass
 */
void RcsMesh_computeInertia(RcsMeshData* mesh, double I[3][3], double com[3]);

/*! \ingroup RcsMeshFunctions
 *  \brief Creates and returns a normal array. The dimensions are 3*nVertices,
 *         and the normals are ordered to be the rows of a nVertices x 3
 *         matrix. The function goes through all faces and computes the
 *         face normal. For each vertex, each face's contribution will be
 *         added, so that vertices with neighboring faces get a shared normal
 *         direction. This is a bit debatable for meshes that are not smooth.
 *         Some sources recommend distinguishing between small and large
 *         angles between faces, or weighting the normals with the face area.
 *         We do none of these things and keep it for later.
 *
 *  \param[in] mesh Mesh data. If it is invalid, the behavior is undefined.
 *  \return Normal array with nVertices x 3 elements. The caller takes
 *          ownership of the returned array and is responsible for its
 *          deletion.
 */
double* RcsMesh_createNormalArray(const RcsMeshData* mesh);


#ifdef __cplusplus
}
#endif

#endif   // RCS_MESH_H
