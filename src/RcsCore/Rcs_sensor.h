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

#ifndef RCS_SENSOR_H
#define RCS_SENSOR_H

#include <Rcs_graph.h>

#include <libxml/tree.h>



#ifdef __cplusplus
extern "C" {
#endif



/*!
 * \defgroup RcsSensorFunctions Functions related to sensors
 *
 */


/*! \ingroup RcsSensorFunctions
 *  \brief Returns pointer to a newly allocated sensor object.
 */
RcsSensor* RcsSensor_create(unsigned int type, const char* name,
                            RcsBody* parent_body, HTr* offset,
                            const char* extraInfo);

/*! \ingroup RcsSensorFunctions
 *  \brief Creates a RcsSensor from an xml node.
 */
RcsSensor* RcsSensor_createFromXML(xmlNode* node, RcsBody* parentBody);

/*! \ingroup RcsSensorFunctions
 *  \brief Clone a sensor. The mount body of the cloned sensor points to the
 *         elements of dstGraph. The next meber is set to NULL. It is
 *         connected in the RcsGraph_addSensor() function. The reason is the
 *         sensor naming: Other than for the RcsBodies, we don't enforce a
 *         unique naming of sensors. Therefore the RcsGraph_getSensorByName()
 *         is ont guaranteed to find the sensor we are looking for in case
 *         of several sensors having the same name.
 *
 *  \param[in] src    Sensor to be cloned. If it is NULL, the function
 *                    returns NULL.
 *  \param[in] graph  Graph the sensor refers to. Must be valid.
 *  \return Pointer to cloned sensor, or NULL if src is NULL.
 */
RcsSensor* RcsSensor_clone(const RcsSensor* src, const RcsGraph* graph);

/*! \ingroup RcsSensorFunctions
 *  \brief Shallow copy of all members except for these pointers:
 *         - next
 *         - body
 *         - extraInfo
 *         They are left unchanged
 */
void RcsSensor_copy(RcsSensor* dst, const RcsSensor* src);

/*! \ingroup RcsSensorFunctions
 *  \brief Adds the given sensor to the linked list of sensors. If newSensor
 *         is NULL, the function returns without doing anything.
 */
void RcsGraph_addSensor(RcsGraph* self, RcsSensor* newSensor);

/*! \ingroup RcsSensorFunctions
 *  \brief Deletes the sensor and frees all memory if given pointer is not
 *         NULL
 */
void RcsSensor_destroy(RcsSensor* self);

/*! \ingroup RcsSensorFunctions
 *  \brief Returns the dimension of the array holding the sensor values.
 */
unsigned int RcsSensor_dim(RcsSensor* self);

/*! \ingroup RcsSensorFunctions
 *  \brief Computes the gravity force and torque at the sensor fts due to the
 *         gravity of the kinematic chain after the sensor. The first 3
 *         elements are the forces, the indices 3-5 comprise the torques, all
 *         represented in the sensor's frame of reference.
 *
 *  \param[in] fts    Load cell sensor. If the sensor is not of type
 *                    RCSSENSOR_LOAD_CELL, the function will exit with a
 *                    fatal error
 *  \param[out] S_f_gravity  Sensor force due to gravity, represented in the
 *                           sensor's frame of reference
 *  \return Compensated mass in [kg]
 */
double RcsSensor_computeStaticForceCompensation(const RcsGraph* graph,
                                                const RcsSensor* fts,
                                                double S_f_gravity[6]);

/*! \ingroup RcsSensorFunctions
 *  \brief Computes the dynamic force and torque at the sensor fts due to the
 *         acceleration of the kinematic chain after the sensor. The first 3
 *         elements are the forces, the indices 3-5 comprise the torques, all
 *         represented in the sensor's frame of reference.
 *
 *  \param[in] self   Pointer to a valid RcsGraph
 *  \param[in] fts    Load cell sensor. If the sensor is not of type
 *                    RCSSENSOR_LOAD_CELL, the function will exit with a
 *                    fatal error
 *  \param[in] qpp_curr  Current joint space accelerations (Dimension must be
 *                       RcsGraph::dof). If it is NULL, it is assumed to be 0.
 *  \param[out] S_f_dynamic  Sensor force due to accelerations, represented
 *                           in the sensor's frame of reference
 *  \return Compensated mass in [kg]
 */
double RcsSensor_computeDynamicForceCompensation(const RcsGraph* self,
                                                 const RcsSensor* fts,
                                                 const MatNd* qpp_curr,
                                                 double S_f_dynamic[6]);

/*! \ingroup RcsSensorFunctions
 *  \brief Computes the static and dynamic force and torque at the sensor fts
 *         due to the gravity and acceleration of the kinematic chain after
 *         the sensor. The first 3 elements are the forces, the indices 3-5
 *         comprise the torques, all represented in the sensor's frame of
 *         reference.
 *
 *  \param[in] self   Pointer to a valid RcsGraph
 *  \param[in] fts    Load cell sensor. If the sensor is not of type
 *                    RCSSENSOR_LOAD_CELL, the function will exit with a
 *                    fatal error
 *  \param[in] qpp_curr  Current joint space accelerations (Dimension must be
 *                       RcsGraph::dof). If it is NULL, it is assumed to be 0.
 *  \param[out] S_f_full  Sensor force due to gravity and accelerations,
 *                        represented in the sensor's frame of reference.
 *  \return Compensated mass in [kg]
 */
double RcsSensor_computeFullForceCompensation(const RcsGraph* self,
                                              const RcsSensor* fts,
                                              const MatNd* qpp_curr,
                                              double S_f_full[6]);

/*! \ingroup RcsSensorFunctions
 *  \brief Prints out the sensor's information to out. The function accepts
 *         if s is NULL.
 */
void RcsSensor_fprint(FILE* out, const RcsSensor* s);

/*! \ingroup RcsSensorFunctions
 *  \brief Prints the sensor's xml representation to the given file
 *         descriptor. Both arguments are assumed to be not NULL. Otherwise,
 *         the function exits with a fatal error.
 */
void RcsSensor_fprintXML(FILE* out, const RcsSensor* self);

/*! \ingroup RcsSensorFunctions
 *  \brief Computes the pressure distribution of the PPS sensor self. If the
 *         sensor is NULL ot the type doesn't match RCSSENSOR_PPS, or the
 *         mount body is NULL, the function issues a warning on debug level
 *         1 and returns false without changing the array ppsResult. Otherwise,
 *         the array ppaResult is reshaped to the sensor's dimension (see
 *         \ref RcsSensor_dim()), and filled with the calculated pressure
 *         values, and true is returned.
 */
#ifdef OLD_TOPO
bool RcsSensor_computePPS(const RcsSensor* self, MatNd* ppsResult,
                          const double contactForce[3]);
#else
bool RcsSensor_computePPS(RcsGraph* graph, const RcsSensor* self, MatNd* ppsResult,
                          const double contactForce[3]);
#endif

/*! \ingroup RcsSensorFunctions
 *  \brief Returns the name of the sensor (see enum RCSSENSOR_TYPE)
 *         as char pointer for debugging purposes. If no corresponding
 *         sensor type exists, "Unknown sensor type" is returned.
 */
const char* RcsSensor_name(int shapeType);



#ifdef __cplusplus
}
#endif

#endif   // RCS_SENSOR_H
