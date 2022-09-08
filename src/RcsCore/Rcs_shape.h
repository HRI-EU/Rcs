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

#ifndef RCS_SHAPE_H
#define RCS_SHAPE_H

#include "Rcs_graph.h"
#include "Rcs_mesh.h"



#ifdef __cplusplus
extern "C" {
#endif



/*!
 * \defgroup RcsShapeFunctions Geometric shapes
 *
 */

/*!
 * @name Accessors and generic functions
 *
 */

///@{

/*! \ingroup RcsShapeFunctions
 *  \brief Clears the memory for all meshes / octrees. The function does
 *         nothing if self is NULL.
 */
void RcsShape_clear(RcsShape* self);

/*! \ingroup RcsShapeFunctions
 *  \brief Returns the name of the shape (see enum RCSSHAPE_TYPE)
 *         as char pointer for debugging purposes. If no corresponding
 *         shape type exists, "Unknown shape type" is returned.
 */
const char* RcsShape_name(int shapeType);

/*! \ingroup RcsShapeFunctions
 *  \brief Makes a deep copy of a shape. The userData pointer is copied if the
 *         shape is of type MARKER or OCTREE. If the sizes of the character
 *         strings don't match, they are re-allocated. The memory areas between
 *         src and dst must not overlap.
 */
void RcsShape_copy(RcsShape* dst, const RcsShape* src);

/*! \ingroup RcsShapeFunctions
 *  \brief Returns a deep copy of a shape. See RcsShape_copy() for details.
 */
RcsShape* RcsShape_clone(const RcsShape* src);

/*! \ingroup RcsShapeFunctions
 *  \brief Initializes an empty shape with everything zeroed except for these
 *         defaults:
 *         - scale3d is (1, 1, 1)
 *         - A_CB is identity transform
 */
void RcsShape_init(RcsShape* self);

/*! \ingroup RcsShapeFunctions
 *  \brief Returns a random shape type that is supported by the compiled
 *         version. For instance, the function will not return a RCSSHAPE_OCTREE
 *         type if the OctoMap support has not been compiled.
 */
int RcsShape_randomShapeType();

/*! \ingroup RcsShapeFunctions
 *  \brief Returns true if the shape type is supported by the compiled
 *         version. For instance, the function will return false if a
 *         RCSSHAPE_OCTREE type if the OctoMap support has not been compiled.
 */
bool RcsShape_isSupported(int shapeType);

/*! \ingroup RcsShapeFunctions
 *  \brief Initializes a shape to the given type with random parameters. Used
 *         for distance function testing. Returns false in case the shape
 *         type is out of range or not supported, or the shape is NULL.
 */
bool RcsShape_initRandom(RcsShape* shape, int shapeType);

/*! \ingroup RcsShapeFunctions
 *  \brief Creates a frame shape with default parameters and the given scaling.
 */
RcsShape* RcsShape_createFrameShape(double scale);

/*! \ingroup RcsShapeFunctions
 *  \brief Returns a mesh for the given shape. The following shape types are
 *         supported:
 *         - RCSSHAPE_SSL
 *         - RCSSHAPE_SSR
 *         - RCSSHAPE_MESH
 *         - RCSSHAPE_BOX
 *         - RCSSHAPE_CYLINDER
 *         - RCSSHAPE_SPHERE
 *         - RCSSHAPE_CONE
 *         - RCSSHAPE_TORUS
 *
 *  \param[in] self  Shape to be represented in the mesh.
 *  \return Mesh data structure, or NULL in case of failure.
 */
RcsMeshData* RcsShape_createMesh(const RcsShape* self);

/*! \ingroup RcsShapeFunctions
 *  \brief Sets or clears the bit of the shape's compute type. For instance:
 *         RcsShape_setComputeType(shape, RCSSHAPE_COMPUTE_DISTANCE)
 *         activates the distance computation of the shape.
 *         RcsShape_clearComputeType(shape, RCSSHAPE_COMPUTE_DISTANCE)
 *         deactivates the distance computation of the shape.
 */
void RcsShape_setComputeType(RcsShape* shape, char computeType, bool enable);

/*! \ingroup RcsShapeFunctions
 *  \brief Returns true if the compute type is set, false otherwise.
 */
bool RcsShape_isOfComputeType(const RcsShape* shape, int computeType);

/*! \ingroup RcsShapeFunctions
 *  \brief Returns true if the shapes are indentical, false otherwise. If
 *         a mesh is associated with the shape, it will also be compared.
 */
bool RcsShape_isEqual(const RcsShape* s1, const RcsShape* s2);

///@}



/*!
 * @name Geometric functions
 *
 */

///@{

/*! \ingroup RcsShapeFunctions
 *  \brief Returns the volume of the shape in cubic meter. The volume of
 *         meshes can not yet be computed, the function will return zero
 *         and issue a warning on debug level 1. The following shape types
 *         will have a volume of zero:
 *         - RCSSHAPE_REFFRAME
 *         - RCSSHAPE_POINT
 *         Shapes of type RCSSHAPE_OCTREE will report correct results only if
 *         the octomap library is linked.
 */
double RcsShape_computeVolume(const RcsShape* self);

/*! \ingroup RcsShapeFunctions
 *  \brief Computes the local COM of the shape in the bodie's frame according
 *         to its relative transformation A_CB from body frame to shape frame.
 *         For most shapes it is the origin of the relative transformation,
 *         for a few shapes (e.g. cone, capsule) there is an additional
 *         offset that is considered.
 */
void RcsShape_computeLocalCOM(const RcsShape* self, double r_com[3]);

/*! \ingroup RcsShapeFunctions
 *  \brief Computes the inertia tensor of the shape with respect to its center
 *         of mass (not its origin), and represented in the shape's frame of
 *         reference.
 */
void RcsShape_computeInertiaTensor(const RcsShape* self, const double density,
                                   double I[3][3]);

/*! \ingroup RcsShapeFunctions
 *  \brief Prints the distance function table to out.
 */
void* RcsShape_addOctree(RcsShape* self, const char* fileName);

/*! \ingroup RcsShapeFunctions
 *  \brief This function computes the axis-aligned bounding box of a shape.
 *
 *  \param[in] self       Shape data. If it is NULL, the AABB is set to zero,
 *                        size, and a debug message is issued on debul level 4.
 *  \param[in] xyzMin     Minimum point of the box
 *  \param[in] xyzMax     Maximum point of the box
 */
void RcsShape_computeAABB(const RcsShape* self,
                          double xyzMin[3], double xyzMax[3]);

/*! \ingroup RcsShapeFunctions
 *  \brief Scales the geometry of the shape.
 *
 *  \param[in,out] self      Shape to be scaled.
 *  \param[in] scale         Scaling factor.
 */
void RcsShape_scale(RcsShape* self, double scale);

///@}


/*!
 * @name Printing
 *
 */

///@{

/*! \ingroup RcsShapeFunctions
 *  \brief Prints out the shape's information to out. The function accepts
 *         if s is NULL.
 */
void RcsShape_fprint(FILE* out, const RcsShape* s);

/*! \ingroup RcsShapeFunctions
 *  \brief Prints the shape's xml representation to the given file
 *         descriptor. Both arguments are assumed to be not NULL. Otherwise,
 *         the function exits with a fatal error.
 *
 *  \param[in] out     File to write to.
 *  \param[in] self    Shape to write to the xml file.
 *
 *  \return Number of errors encountered. Errors are reported on debug level 1.
 */
int RcsShape_fprintXML(FILE* out, const RcsShape* self);

///@}


/*!
 * @name Distance functions
 *
 */

///@{

/*! \ingroup RcsShapeFunctions
 *  \brief Prints the distance function table to out.
 */
void RcsShape_fprintDistanceFunctions(FILE* out);

/*! \ingroup RcsShapeFunctions
 *  \brief Distance function signature.
 */
typedef double(*RcsDistanceFunction)(const RcsShape*, const RcsShape*,
                                     const HTr*, const HTr*,
                                     double[3], double[3], double[3]);

/*! \ingroup RcsShapeFunctions
 *  \brief The distance computation between different shapes is implemented
 *         through a table of function pointers. This function allows to add
 *         or change distance functions with other implementations.
 *
 *  \param[in]  shapeTypeIdx1     Shape 1 ID according to enum RCSSHAPE_TYPE
 *  \param[in]  shapeTypeIdx2     Shape 2 ID according to enum RCSSHAPE_TYPE
 *  \param[in]  func              Function pointer to distance function
 *  \return true for success, false otherwise. In case of failure, a console
 *          message is printed on debug level 4. The only failure reason are
 *          shape indices that are out of range.
 */
bool RcsShape_setDistanceFunction(unsigned int shapeTypeIdx1,
                                  unsigned int shapeTypeIdx2,
                                  RcsDistanceFunction func);

/*! \ingroup RcsShapeFunctions
 *  \brief Returns a function poiter of the distance function that computes
 *         the shape distance with the corresponding indices.
 *
 *  \param[in] shapeTypeIdx1   First shape's index (see RCSSHAPE_TYPE)
 *  \param[in] shapeTypeIdx2   Second shape's index (see RCSSHAPE_TYPE)
 *  \return Function pointer to distance function or NULL if it doesn't exist.
 *          The function also returns NULL if either index is equal or larger
 *          than RCSSHAPE_SHAPE_MAX. In this case, a warning is issued on
 *          debug level 4.
 */
RcsDistanceFunction RcsShape_getDistanceFunction(unsigned int shapeTypeIdx1,
                                                 unsigned int shapeTypeIdx2);

/*! \ingroup RcsShapeFunctions
 *  \brief Returns true if a distance function between two shape types
 *         exists, false otherwise.
 *
 *  \param[in] shapeTypeIdx1   First shape's index (see RCSSHAPE_TYPE)
 *  \param[in] shapeTypeIdx2   Second shape's index (see RCSSHAPE_TYPE)
 *  \return True if distance function exists, false otherwise.
 */
bool RcsShape_hasDistanceFunction(unsigned int shapeTypeIdx1,
                                  unsigned int shapeTypeIdx2);

/*! \ingroup RcsShapeFunctions
 *  \brief Returns the distance of s1 and s2. The closest points (in world
 *         coordinates) will be copied to I_cp1 and I_cp2. The unit normal
 *         vector (in world coordinates) of s1 (pointing away from the
 *         surface) is copied to I_n. This is done since for the distance
 *         being zero, the normal vector can't be computed from the closest
 *         points, but internally in the proximity function, it is possible.
 */
double RcsShape_distance(const RcsShape* s1, const RcsShape* s2,
                         const HTr* A_B1I, const HTr* A_B2I,
                         double I_cp1[3], double I_cp2[3], double I_n[3]);

/*! \ingroup RcsShapeFunctions
 *  \brief Computes the distance between a shape and a 3d point.
 *
 *  \param[in]  shape        RcsShape to which distance is to be computed
 *  \param[in]  A_BI         Transformation from world (I) to the bodie's base
 *                           frame. The shape frame might have an additional
 *                           relative transform that is represented in
 *                           RcsShape::A_CB. It will considered by the function.
 *  \param[in]  I_pt         Point in world coordinates
 *  \param[out] I_cpShape    Closest point on the shape in world coordinates
 *  \param[out] I_nShapePt   Unit normal vector from shape towards point
 */
double RcsShape_distanceToPoint(const RcsShape* shape,
                                const HTr* A_BI,
                                const double I_pt[3],
                                double I_cpShape[3],
                                double I_nShapePt[3]);

/*! \ingroup RcsShapeFunctions
 *  \brief Computes the intersection between a line given by a point and a
 *         direction and a shape. The function returns true if both intersect,
 *         and stores the closest intersection point in closestLinePt (if it
 *         is not NULL).
 */
bool RcsShape_computeLineIntersection(const double linePt[3],
                                      const double lineDir[3],
                                      const HTr* A_BI,
                                      const RcsShape* shape,
                                      double closestLinePt[3]);

/*! \ingroup RcsShapeFunctions
 *  \brief Computes the intersection between a ray given by a point and a
 *         direction and a shape. The function returns true if both intersect,
 *         and stores the closest intersection point in closestLinePt (if it
 *         is not NULL).
 */
bool RcsShape_computeRayIntersection(const double linePt[3],
                                     const double lineDir[3],
                                     const HTr* A_BI,
                                     const RcsShape* shape,
                                     double closestLinePt[3]);

/*! \ingroup RcsShapeFunctions
 *  \brief Computes the distance between a point and the bounding sphere of a
 *         shape. The sphere is always around the shape's origin; i.e., not
 *         necessarily the best approximation (e.g., for a cone)
 *
 * \param[in] Pt Point in world coordinates
 * \param[in] A_BI Transformation from world frame into the frame of the body
 *                 that contains the shape
 * \param[in] shape Pointer to shape
 * \return Distance to shape's bounding sphere approximation
 */
double RcsShape_boundingSphereDistance(const double Pt[3],
                                       const HTr* A_BI,
                                       const RcsShape* shape);


///@}



#ifdef __cplusplus
}
#endif

#endif   // RCS_SHAPE_H
