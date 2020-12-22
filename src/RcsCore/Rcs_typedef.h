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

#ifndef RCS_TYPEDEF_H
#define RCS_TYPEDEF_H

#ifdef __cplusplus
extern "C" {
#endif


#include "Rcs_graph.h"

#define RCS_GRAVITY          (9.81)
#define RCS_MAX_NAMELEN      (64)
#define RCS_MAX_FILENAMELEN  (256)

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
  char* name;          ///< Joint name
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
  HTr* A_JP;           ///< Relative transformation
  HTr A_JI;            ///< Absolute transformation
  double maxTorque;    ///< Max. torque for physics (default is 1.0)
  double speedLimit;   ///< Speed limit (default is DBL_MAX)
  double accLimit;     ///< Acceleration limit (default is DBL_MAX)
  double decLimit;     ///< Deceleration limit (default is DBL_MAX)
  int ctrlType;        ///< See RCSJOINT_CTRL_TYPE
  char* coupledJointName;
  MatNd* couplingFactors;

  RcsJoint* prev;      ///< Previous joint
  RcsJoint* next;      ///< Next joint
  RcsJoint* coupledTo; ///< Joint to which this one is coupled
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
  RCSSHAPE_GPISF     = 9,   ///< Implicite surface
  RCSSHAPE_TORUS     = 10,  ///< Torus
  RCSSHAPE_OCTREE    = 11,  ///< Octree
  RCSSHAPE_POINT     = 12,  ///< Point
  RCSSHAPE_MARKER    = 13,  ///< Marker
  RCSSHAPE_SHAPE_MAX = 14   ///< max. number of shape types

} RCSSHAPE_TYPE;



typedef enum
{
  RCSSHAPE_COMPUTE_DISTANCE     = 1,   ///< Distance computation
  RCSSHAPE_COMPUTE_PHYSICS      = 2,   ///< Physics simulation
  RCSSHAPE_COMPUTE_GRAPHICS     = 4,   ///< Graphics visualization
  RCSSHAPE_COMPUTE_CONTACT      = 8,   ///< Contact simulation
  RCSSHAPE_COMPUTE_SOFTPHYSICS  = 16,  ///< Soft physics simulation
  RCSSHAPE_COMPUTE_DEPTHBUFFER  = 32   ///< Depth buffer simulation

} RCSSHAPE_COMPUTE_TYPE;

struct _RcsShape
{
  int type;             ///< SSL, SSR or other
  HTr A_CB;             ///< Relative transformation from body
  double extents[3];    ///< Geometrical parameters
  double scale;         ///< Scale factor for meshes (applied after parsing)
  bool resizeable;      ///< For visualization and copying
  char computeType;     ///< Bitmask, see RCSSHAPE_COMPUTE_TYPE

  char* meshFile;       ///< Name of a mesh file (if any)
  char* textureFile;    ///< File holding texture data
  char* color;          ///< Color of shape primitives
  char* material;       ///< Material of the shape (for physics simulation)

  void* userData;       ///< For user extensions
};



struct _RcsPair
{
  const RcsBody* b1;     ///< First body
  const RcsBody* b2;     ///< Second body
  const RcsGraph* graph; ///< Graph to which bodies belong
  double weight;         ///< Weighting factor for distance
  double dThreshold;     ///< Distance threshold
  double distance;       ///< Shortest distance between b1 and b2
  double* cp1;           ///< Closest point on body 1
  double* cp2;           ///< Closest point on body 2
  double* n1;            ///< Unit normal on body 1 (pointing outwards)
};



struct _RcsCollisionMdl
{
  const RcsGraph* graph;   ///< Graph to which bodies belong
  RcsPair** pair;          ///< List of collision pairs
  MatNd* cp;               ///< Closest points in alternating order
  MatNd* n1;               ///< Unit normal on body 1 (pointing outwards)
  double sMixtureCost;     ///< Factor that multiplies the center distances
  double penetrationSlope; ///< Steepness of the cost at zero distance
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
  double m;                         ///< Body mass
  bool rigid_body_joints;           ///< Has 6 rigid body dof
  int physicsSim;                   ///< see RCSBODY_PHYSICS_SIMULATION_TYPE
  double x_dot[3];                  ///< Bodie's lin. velocity in world coords
  double omega[3];                  ///< Bodie's ang. velocity in world coords
  double confidence;                ///< Obsolete
  char bdyName[RCS_MAX_NAMELEN];    ///< Fully qualified name including suffix
  char bdyXmlName[RCS_MAX_NAMELEN]; ///< Name of the body from xml file
  char bdySuffix[RCS_MAX_NAMELEN];  ///< Group suffix of the body

  HTr A_BP;                         ///< Relative transformation
  HTr A_BI;                         ///< Absolute transformation
  HTr Inertia;                      ///< Inertia tensor and local COG vector

#ifdef OLD_TOPO
  RcsBody* parent;        ///< Parent body
  RcsBody* firstChild;    ///< First child body
  RcsBody* lastChild;     ///< Last child body
  RcsBody* next;          ///< Next sibling body
  RcsBody* prev;          ///< Previous sibling body
#endif

  int id;                 ///< Body id
  int parentId;           ///< Parent body
  int firstChildId;       ///< First child body
  int lastChildId;        ///< Last child body
  int nextId;             ///< Next sibling body
  int prevId;             ///< Previous sibling body

  RcsShape** shape;       ///< Shapes of the body for collision detection
  RcsJoint* jnt;          ///< Joint to which body is attached

  void* extraInfo;        ///< For generic bodies
};


typedef enum
{
  RCSSENSOR_CUSTOM = 0,      ///< Use this type for self made sensors
  RCSSENSOR_LOAD_CELL,       ///< Measuring forces and torques
  RCSSENSOR_JOINT_TORQUE,    ///< Retrieve the current torque acting in a joint
  RCSSENSOR_CONTACT_FORCE,   ///< Artificial skin sensor
  RCSSENSOR_PPS              ///< Artificial skin sensor array
} RCSSENSOR_TYPE;

struct _RcsTexel
{
  double position[3];    ///< Texel position in sensor frame
  double normal[3];      ///< Texel unit normal in sensor frame
  double extents[3];     ///< Texel side lengths xyz
};

struct _RcsSensor
{
  RCSSENSOR_TYPE type;   ///< Sensor type, see enum RCSSENSOR_TYPE
  char* name;            ///< Name of the sensor
  RcsSensor* next;       ///< Pointer to next sensor, NULL otherwise
  int bodyId;            ///< Id of the body the sensor is attached to
  HTr* offset;           ///< Relative transformation of the sensor mount point
  char* extraInfo;       ///< Pressure array data
  RcsTexel** texel;      ///< Array of texels for PPS sensors
  MatNd* rawData;        ///< Raw sensor data array
};



struct _RcsGraph
{
  RcsBody* root;          ///< Pointer to root body
  RcsBody* bodies;        ///< Array of bodies
  RcsBody gBody[10];      ///< Generic bodies
  unsigned int dof;       ///< Number of degrees of freedom
  unsigned int nJ;        ///< Number of unconstrained degrees of freedom
  unsigned int nBodies;   ///< Number of bodies in the graph
  MatNd* q;               ///< Array of joint values
  MatNd* q_dot;           ///< Array of joint velocity values
  char* xmlFile;          ///< Configuration file name (full path)
  RcsSensor* sensor;      ///< Pointer to the first registered sensor

  void* userData;         ///< To append extensions
};


#ifdef __cplusplus
}
#endif

#endif   // RCS_TYPEDEF_H
