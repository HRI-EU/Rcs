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

#ifndef RCS_TYPEDEF_H
#define RCS_TYPEDEF_H

#ifdef __cplusplus
extern "C" {
#endif


#include "Rcs_graph.h"


#define RCS_GRAVITY              (9.81)
#define RCS_MAX_NAMELEN          (64)
#define RCS_MAX_FILENAMELEN      (256)
#define RCS_MAX_COUPLING_COEFF   (8)
#define RCS_NUM_GENERIC_BODIES   (10)

typedef enum
{
  RCSJOINT_ROT_X      = 0,
  RCSJOINT_ROT_Y      = 1,
  RCSJOINT_ROT_Z      = 2,
  RCSJOINT_TRANS_X    = 3,
  RCSJOINT_TRANS_Y    = 4,
  RCSJOINT_TRANS_Z    = 5
}
RCSJOINT_TYPE;

typedef enum
{
  RCSJOINT_CTRL_POSITION = 0,
  RCSJOINT_CTRL_VELOCITY,
  RCSJOINT_CTRL_TORQUE
} RCSJOINT_CTRL_TYPE;

struct _RcsJoint
{
  double q0;           ///< Joint center position
  double q_init;       ///< Initial joint center position
  double q_min;        ///< Lower joint limit
  double q_max;        ///< Upper joint limit
  double weightJL;     ///< Weighting factor for joint limits
  double weightCA;     ///< Weighting factor for collision avoidance
  double weightMetric; ///< Weighting factor for joint contribution
  bool constrained;    ///< True if the dof is constrained, false else
  int type;            ///< Rotation or translation, see enum RCSJOINT_TYPE
  int dirIdx;          ///< 0 for x-, 1 for y- and 2 for z-direction
  int jointIndex;      ///< Index in the q vector, set during initial parsing
  int jacobiIndex;     ///< Corresponding column in Jacobian
  HTr A_JP;            ///< Relative transformation
  HTr A_JI;            ///< Absolute transformation
  double maxTorque;    ///< Max. torque for physics (default is 1.0)
  double speedLimit;   ///< Speed limit (default is DBL_MAX)
  double accLimit;     ///< Acceleration limit (default is DBL_MAX)
  double decLimit;     ///< Deceleration limit (default is DBL_MAX)
  int ctrlType;        ///< See RCSJOINT_CTRL_TYPE
  char name[RCS_MAX_NAMELEN];///< Joint name
  char coupledJntName[RCS_MAX_NAMELEN];
  double couplingPoly[RCS_MAX_COUPLING_COEFF];
  unsigned int nCouplingCoeff;

  int id;          ///< This joint's id
  int prevId;      ///< Previous joint
  int nextId;      ///< Next joint
  int coupledToId; ///< Joint to which this one is coupled
};



typedef enum
{
  RCSSHAPE_NONE      = 0,   ///< No shape, the default
  RCSSHAPE_SSL       = 1,   ///< Sphere swept line
  RCSSHAPE_SSR       = 2,   ///< Sphere swept rectangle
  RCSSHAPE_MESH      = 3,   ///< Mesh
  RCSSHAPE_BOX       = 4,   ///< Cuboid
  RCSSHAPE_CYLINDER  = 5,   ///< Cylinder
  RCSSHAPE_REFFRAME  = 6,   ///< Reference coordinate system
  RCSSHAPE_SPHERE    = 7,   ///< Sphere
  RCSSHAPE_CONE      = 8,   ///< Cone
  RCSSHAPE_TORUS     = 9,   ///< Torus
  RCSSHAPE_OCTREE    = 10,  ///< Octree
  RCSSHAPE_POINT     = 11,  ///< Point
  RCSSHAPE_SHAPE_MAX = 12   ///< max. number of shape types

} RCSSHAPE_TYPE;



typedef enum
{
  RCSSHAPE_COMPUTE_DISTANCE     = 1,   ///< Distance computation
  RCSSHAPE_COMPUTE_PHYSICS      = 2,   ///< Physics simulation
  RCSSHAPE_COMPUTE_GRAPHICS     = 4,   ///< Graphics visualization
  RCSSHAPE_COMPUTE_CONTACT      = 8,   ///< Contact simulation
  RCSSHAPE_COMPUTE_SOFTPHYSICS  = 16,  ///< Soft physics simulation
  RCSSHAPE_COMPUTE_DEPTHBUFFER  = 32,  ///< Depth buffer simulation
  RCSSHAPE_COMPUTE_ATTACHMENT   = 64   ///< Spring damper connection

} RCSSHAPE_COMPUTE_TYPE;

struct _RcsShape
{
  int type;             ///< SSL, SSR or other
  HTr A_CB;             ///< Relative transformation from body
  double extents[3];    ///< Geometrical parameters
  double scale3d[3];    ///< Scale factor for meshes (applied after parsing)
  bool resizeable;      ///< For visualization and copying
  char computeType;     ///< Bitmask, see RCSSHAPE_COMPUTE_TYPE

  char meshFile[RCS_MAX_FILENAMELEN];    ///< Name of a mesh file (if any)
  char textureFile[RCS_MAX_FILENAMELEN]; ///< File holding texture data
  char color[RCS_MAX_NAMELEN];           ///< Color of shape primitives
  char material[RCS_MAX_NAMELEN];        ///< Physics material of the shape

  RcsMeshData* mesh;
};



typedef enum
{
  RCSBODY_PHYSICS_NONE       = 0,   ///< No physics simulation
  RCSBODY_PHYSICS_KINEMATIC  = 1,   ///< Body attached to transform
  RCSBODY_PHYSICS_DYNAMIC    = 2,   ///< Dynamics simulation
  RCSBODY_PHYSICS_FIXED      = 3    ///< Body attached to previous

} RCSBODY_PHYSICS_SIMULATION_TYPE;


struct _RcsBody
{
  int id;                           ///< Body id
  int parentId;                     ///< Parent body
  int firstChildId;                 ///< First child body
  int lastChildId;                  ///< Last child body
  int nextId;                       ///< Next sibling body
  int prevId;                       ///< Previous sibling body
  int jntId;                        ///< Joint to which body is attached
  HTr A_BP;                         ///< Relative transformation
  HTr A_BI;                         ///< Absolute transformation
  double m;                         ///< Body mass
  bool rigid_body_joints;           ///< Has 6 rigid body dof
  int physicsSim;                   ///< see RCSBODY_PHYSICS_SIMULATION_TYPE
  double x_dot[3];                  ///< Bodie's lin. velocity in world coords
  double omega[3];                  ///< Bodie's ang. velocity in world coords
  double confidence;                ///< Obsolete
  char name[RCS_MAX_NAMELEN];       ///< Fully qualified name including suffix
  char bdyXmlName[RCS_MAX_NAMELEN]; ///< Name of the body from xml file
  char bdySuffix[RCS_MAX_NAMELEN];  ///< Group suffix of the body
  HTr Inertia;                      ///< Inertia tensor and local COG vector
  unsigned int nShapes;             ///< Number of shapes in shapes arrray
  RcsShape* shapes;                 ///< Geometric shapes of the body
};


typedef enum
{
  RCSSENSOR_CUSTOM = 0,             ///< Use this type for self made sensors
  RCSSENSOR_LOAD_CELL,              ///< Measuring forces and torques
  RCSSENSOR_JOINT_TORQUE,           ///< Joint torque or force
  RCSSENSOR_CONTACT_FORCE,          ///< Artificial skin sensor
  RCSSENSOR_PPS                     ///< Artificial skin sensor array
} RCSSENSOR_TYPE;

struct _RcsTexel
{
  double position[3];               ///< Texel position in sensor frame
  double normal[3];                 ///< Texel unit normal in sensor frame
  double extents[3];                ///< Texel side lengths xyz (z is thickness)
};

struct _RcsSensor
{
  RCSSENSOR_TYPE type;              ///< Sensor type, see enum RCSSENSOR_TYPE
  int bodyId;                       ///< Body if the sensor is attached to
  char name[RCS_MAX_NAMELEN];       ///< Name of the sensor
  HTr A_SB;                         ///< Transformation from body to sensor
  unsigned int nTexels;             ///< Number of texels in texel array
  RcsTexel* texel;                  ///< Array of texels for PPS sensors
  MatNd* rawData;                   ///< Raw sensor data array
};



struct _RcsPair
{
  int b1;                ///< First body id
  int b2;                ///< Second body id
  double weight;         ///< Weighting factor for distance
  double dThreshold;     ///< Distance threshold
  double distance;       ///< Shortest distance between b1 and b2
  int cp1;               ///< Row number of closest point on body 1
  int cp2;               ///< Row number of closest point on body 2
  int n1;                ///< Unit normal on body 1 (pointing outwards)
};



struct _RcsCollisionMdl
{
  const RcsGraph* graph;   ///< Graph to which bodies belong
  RcsPair* pair;           ///< Array of collision pairs
  unsigned int nPairs;     ///< Number of pairs
  MatNd* cp;               ///< Closest points in alternating order
  MatNd* n1;               ///< Unit normal on body 1 (pointing outwards)
  double sMixtureCost;     ///< Factor that multiplies the center distances
  double penetrationSlope; ///< Steepness of the cost at zero distance
};



struct _RcsGraph
{
  int rootId;             ///< Id of root body
  RcsJoint* joints;       ///< Array of dof joints
  unsigned int dof;       ///< Number of degrees of freedom
  RcsBody* bodies;        ///< Array of bodies
  unsigned int nBodies;   ///< Number of bodies in the graph
  RcsSensor* sensors;     ///< Array of sensors
  unsigned int nSensors;  ///< Number of sensors in the graph
  unsigned int nJ;        ///< Number of unconstrained degrees of freedom
  MatNd* q;               ///< Array of joint values
  MatNd* q_dot;           ///< Array of joint velocity values
  char cfgFile[RCS_MAX_FILENAMELEN]; ///< Configuration file name (full path)
  int gBody[RCS_NUM_GENERIC_BODIES]; ///< Generic bodies
};



#define RCSJOINT_BY_ID(graph, id) ((id)==-1 ? NULL : &(graph)->joints[id])
#define RCSBODY_BY_ID(graph, id)  ((id)==-1 ? NULL : &(graph)->bodies[id])
#define RCSBODY_NAME_BY_ID(graph, id)  \
  ((id)==-1 ? "NULL ": (graph)->bodies[id].name)

#undef RCSGRAPH_FOREACH_SENSOR
#define RCSGRAPH_FOREACH_SENSOR(graph)                                         \
  for (RcsSensor *S0 = (graph)->nSensors>0 ? &(graph)->sensors[0] : NULL,      \
       *S1 = (S0!=NULL) ? &(graph)->sensors[graph->nSensors-1] : NULL,       \
       *SENSOR = S0; SENSOR && SENSOR<=S1; SENSOR++)

#undef RCSGRAPH_FOREACH_BODY
#define RCSGRAPH_FOREACH_BODY(graph)                                           \
  for (RcsBody *B0 = (graph)->nBodies>0 ? &(graph)->bodies[0] : NULL,          \
       *B1 = (B0!=NULL) ? &(graph)->bodies[(graph)->nBodies-1] : NULL,       \
       *BODY = B0; BODY && BODY<=B1; BODY++)

#undef RCSJOINT_TRAVERSE_FORWARD
#define RCSJOINT_TRAVERSE_FORWARD(graph, joint)                                \
  for (RcsJoint* JNT = (joint); JNT; JNT = RCSJOINT_BY_ID(graph, JNT->nextId))

#undef RCSBODY_FOREACH_JOINT
#define RCSBODY_FOREACH_JOINT(graph, body)                                     \
  RCSJOINT_TRAVERSE_FORWARD(graph, RCSJOINT_BY_ID((graph), (body)->jntId))


#ifdef __cplusplus
}
#endif

#endif   // RCS_TYPEDEF_H
