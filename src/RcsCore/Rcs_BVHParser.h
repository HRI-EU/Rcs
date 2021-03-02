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

#ifndef RCS_BVHPARSER_H
#define RCS_BVHPARSER_H

#include <Rcs_graph.h>


#ifdef __cplusplus
extern "C" {
#endif


/*! \ingroup RcsGraphFunctions
 *  \brief Creates a graph from a given BVH file. BVH stands for "Bounding
 *         Volume Hierarchy" and is a Motion Capture format.
 *
 *  \param[in]  fileName         Fully qualified name of BVH file
 *  \param[in]  linearScaleToSI  Value that scales the BVH-file's translations
 *                               to SI units (meters).
 *  \param[in] Z_up_x_forward    When false, the root frame is kept in BVH-
 *                               conventions (y-up), otherwise it is transformed
 *                               into z-up x-forward convention.
 *  \return                      Graph according to BVH file, or NULL if loading
 *                               failed. In the latter case, debug messages are
 *                               printed to the console on debug levels 1 and
 *                               higher.
 */
RcsGraph* RcsGraph_createFromBVHFile(const char* fileName,
                                     double linearScaleToSI,
                                     bool Z_up_x_forward);

/*! \ingroup RcsGraphFunctions
 *  \brief Creates a matrix of animation poses from a given BVH file.
 *
 *  \param[in]  graph             Graph corresponding to BVH file. If graph is
 *                                NULL, the first 3 rows of the data array is
 *                                scaled by linearScaleToSI, and the remaining
 *                                ones by angularScaleToSI. If graph is not
 *                                NULL, all values corresponding to a
 *                                rotational joint are scaled by
 *                                angularScaleToSI, and the linear ones by
 *                                linearScaleToSI.
 *  \param[in]  fileName          Fully qualified name of BVH file
 *  \param[out]  dt               Time step between two consecutive frames. If
 *                                it is NULL, it will be ignored.
 *  \param[in]  linearScaleToSI   Value that scales the BVH-file's translations
 *                                to SI units (meters).
 *  \param[in] angularScaleToSI   Value that scales the BVH-file's rotations
 *                                to SI units (radians).
 *  \return Created trajectory, or NULL in case of failure.
 */
MatNd* RcsGraph_createTrajectoryFromBVHFile(const RcsGraph* graph,
                                            const char* fileName,
                                            double* dt,
                                            double linearScaleToSI,
                                            double angularScaleToSI);

/*! \ingroup RcsGraphFunctions
 *  \brief Adds shapes for feet, hands and head if the BVH model contains the
 *         following bodies:
 *         - Head
 *         - RightToe
 *         - LeftToe
 *         - RightWrist
 *         - LeftWrist
 *
 *  \param[in]  graph             Graph corresponding to BVH file.
 *  \param[in]  linearScaleToSI   Value that scales the BVH-file's translations
 *                                to SI units (meters).
 *  \return True for success, false otherwise. In case of failure, the graph
 *          remains unchanged.
 */
bool RcsGraph_beautifyHumanModelBVH(RcsGraph* graph,
                                    double linearScaleToSI);

/*! \ingroup RcsGraphFunctions
 *  \brief Computes the REBA score according to text book with a few
 *         assumptions. The graph must contain the bodies according to a
 *         BVH file in the correct conventions. The arm scores will be
 *         calculated for both arms, and the REBA score is computed for
 *         both of them. The function returns the higher score.
 *         The coupling and activity scores are currently not considered.
 *
 *         These bodies must exist:
 *         - Hips
 *         - Chest
 *         - Neck
 *         - Base
 *         - LeftToe
 *         - RightToe
 *         - LeftHip
 *         - LeftKnee
 *         - RightHip
 *         - RightKnee
 *
 *         The bodie's z-axis is pointing forward, the y-axis is pointing up.
 *
 *         These joints must exist:
 *         - Neck_jnt_Xrotation (tilt)
 *         - Neck_jnt_Yrotation (twist)
 *         - Neck_jnt_Zrotation (pan)
 *         - RightShoulder_jnt_Yrotation (flexion)
 *         - LeftShoulder_jnt_Yrotation
 *         - RightShoulder_jnt_Zrotation (abduction)
 *         - LeftShoulder_jnt_Zrotation
 *         - RightWrist_jnt_Zrotation (flexion)
 *         - LeftWrist_jnt_Zrotation
 *         - RightWrist_jnt_Yrotation (abduction)
 *         - LeftWrist_jnt_Yrotation
 *         - RightWrist_jnt_Xrotation (twist)
 *         - LeftWrist_jnt_Xrotation
 *
 *  \param[in]  graph  Graph corresponding to BVH file.
 *  \return REBA score.
 */
int RcsGraph_computeREBA(const RcsGraph* graph);


#ifdef __cplusplus
}
#endif


#endif   // RCS_BVHPARSER_H
