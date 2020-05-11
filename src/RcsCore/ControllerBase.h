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

#ifndef RCS_CONTROLLERBASE_H
#define RCS_CONTROLLERBASE_H

/*!
 *  \defgroup RcsController Controller classes
 *
 *  A library for various controller implementations
 */


#include "Task.h"
#include "Rcs_collisionModel.h"



namespace Rcs
{
/*! \ingroup RcsController
 * \brief Controller Base Class. This class implements a number of generic
 *        methods to compute kinematic and dynamic magnitudes, such as
 *        Jacobians, Hessians, velocities etc. The class holds a pointer to a
 *        graph, which is representing the kinematic state considered. None of
 *        the methods will change the graph, with the exception of destroying
 *        it when it is owned by the class itself (see constructor
 *        documentation). The core of this class is the task vector, which can
 *        conveniently be parsed from an xml file. Some tasks allow for another
 *        construction, see details in the respective classes.
 *
 */
class ControllerBase
{
public:

  /*! \brief Constructor based on xml parsing. The file xmlDescription must
   *         contain the file name of the controller definition, or a string
   *         containing the xml description itself. The flag
   *         xmlParsingFinished indicates if the xml file should be closed in
   *         this class, or left open. The latter is needed for derieved classes
   *         that have to parse additional data from the xml file. If you are
   *         instantiating this class directly, it should be set to true (the
   *         default).
   */
  ControllerBase(const std::string& xmlDescription,
                 bool xmlParsingFinished=true);

  /*! \brief Constructor based on a graph. The new class takes ownership of the
   *         given graph. Use this if you only want to add tasks manually using
   *         addTask. If this constructor is used, the readXYFromXML methods
   *         will NOT work.
   */
  ControllerBase(RcsGraph* graph);

  /*! \brief Copy constructor doing deep copying. The new class owns a deep
   *         copy of the graph and the collision model (if any).
   */
  ControllerBase(const ControllerBase& copyFromMe);

  /*! \brief Assignment operator. The new class owns a deep copy of the graph
   *         and the collision model (if any).
   */
  ControllerBase& operator= (const ControllerBase& copyFromMe);

  /*! \brief Deletes all tasks and frees all memory.
   */
  virtual ~ControllerBase();

  virtual void initFromXmlNode(xmlNodePtr xmlNodeController);

  /*! \brief Returns the name of a task with the given ID. If id is out
   *         of range, the function will exit with a fatal error.
   *
   *  \param[in] id   Task index
   *  \return Reference to the task name string
   */
  virtual const std::string& getTaskName(size_t id) const;

  /*! \brief Returns the class name of a task with the given ID. If id is
   *         out of range, the function will exit with a fatal error.
   *
   *  \param[in] id   Task index
   *  \return Reference to the task type string.
   */
  virtual std::string getTaskType(size_t id) const;

  /*! \brief Returns the full dimension of all tasks.
  *
  *  \return Overall task space dimension for id=-1, or the dimension of
  *          the task with the given id.
  */
  virtual size_t getTaskDim() const;

  /*! \brief Returns the full dimension of a task with the given ID. If
   *         id is out of range, the function will exit with a fatal error.
   *
   *  \param[in] id   Task index
  *  \return Dimension of the task with the given id.
   */
  virtual size_t getTaskDim(size_t id) const;

  /*! \brief Returns the dimension of all active tasks. The number of rows
   *         of array activation must match the number of tasks.
   */
  size_t getActiveTaskDim(const MatNd* activation) const;

  /*! \brief Returns the index of the task. If a task with the given name
   *         is not found, or name is NULL, the function returns -1.
   *
   *  \param[in] name   Task name.
   *  \return Task index in the vector, or -1, if none found.
   */
  virtual int getTaskIndex(const char* name) const;

  /*! \brief Returns the index of the task. If a task with the given pointer
   *         is not found, or the argument is NULL, the function returns -1.
   *
   *  \param[in] task   Pointer to task.
   *  \return Task index in the vector, or -1, if none found.
   */
  virtual int getTaskIndex(const Task* task) const;

  /*! \brief Returns the task's index to its entries in x_curr, x_des, and
  *         xp_des vectors. If id is larger than the number of tasks, the
  *         function will exit with a fatal error.
  */
  virtual size_t getTaskArrayIndex(size_t id) const;

  /*! \brief Returns the index of the task vector element. If a task with
   *         the given name is not found, or name is NULL, the function
   *         returns -1 and issues a warning on debug level 4.
   *
   *  \param[in] name   Task name.
   *  \return Task array index in the vector, or -1, if none found.
   */
  virtual int getTaskArrayIndex(const char* name) const;

  /*! \brief Returns the index of the task vector element. If the task is not
   *         found, or is NULL, the function returns -1 and issues a warning on
   *         debug level 4.
   *
   *  \param[in] task   Task
   *  \return Task array index in the vector, or -1, if none found.
   */
  virtual int getTaskArrayIndex(const Task* task) const;

  /*! \brief Returns a pointer to the task id. If the id is out of range, the
   *         function will exit with a fatal error.
   */
  const Task* getTask(size_t id) const;

  /*! \brief Returns a const pointer to the task id. If the id is out of range,
   *         the function will exit with a fatal error.
   */
  Task* getTask(size_t id);

  /*! \brief Returns a pointer to the task name. If a task with the specified
   *         name doesn't exist, the function returns NULL.
   */
  const Task* getTask(const std::string& name) const;

  /*! \brief Returns a const pointer to the task name. If a task with the
   *         specified name doesn't exist, the function returns NULL.
   */
  Task* getTask(const std::string& name);

  /*! \brief Return the number of registered tasks (this->tasks.size())
   */
  virtual size_t getNumberOfTasks() const;

  /*! \brief Returns a vector of all active tasks.
   */
  virtual std::vector<Task*> getTasks(const MatNd* activation=NULL) const;

  /*! \brief Return a pointer to the controller's underlying graph
   */
  RcsGraph* getGraph() const;

  /*! \brief Return the graph file name (e.g., gScenario.xml)
   */
  std::string getGraphFileName() const;

  /*! \brief Return a pointer to the underlying collision model.
   */
  RcsCollisionMdl* getCollisionMdl() const;

  /*! \brief Reads the activation vector from the xml file. If the xml field
   *         of the task has a tag "active", the activation will be set to
   *         1 for true, 0 for false. If the xml field has a tag
   *         "activation", the activation of the task will be set to the
   *         value of this tag. The tag "activation" overwrites the value
   *         of the tag "active", if both exist. If none of the tags
   *         exists, the activation value in a_init is set to 0. Vector
   *         activation is reshaped to nTasks x 1.
   */
  void readActivationsFromXML(MatNd* activation) const;

  /*! \brief Reads the activation vector from the xml file. If the xml field
   *         of the task has the given tag, the activation will be set to
   *         the corresponding value. Otherwise, the corresponding element of
   *         vector x remains unchanged. Vector x is reshaped to nTasks x 1.
   */
  void readActivationVectorFromXML(MatNd* x, const char* tag) const;

  /*! \brief Reads the task vector from the xml file. If the xml field
   *         of the task has he given tag, the corresponsing values will
   *         be copied to x.
   */
  void readTaskVectorFromXML(MatNd* x, const char* tag) const;

  /*! \brief Return the vector of tasks
   */
  const std::vector<Task*>& taskVec() const;

  /*! \brief Computes the task Jacobian J.
   *
   *         Vector a_des is of dimension [allTasks x 1]
   *         If a_des == NULL, the complete Jacobian is returned, else
   *         the Jacobian is composed of only the rows that correspond to
   *         active tasks (with entries of a that are > 0). The function
   *         checks if memory is written over the bounds of J.
   */
  virtual void computeJ(MatNd* J, const MatNd* a_des=NULL) const;

  /*! \brief Computes the task Hessian H.
   *
   *         Vector a_des is of dimension [allTasks x 1]
   *         If a_des == NULL, the complete Hessian is returned, else
   *         the Hessian is composed of only the rows that correspond to
   *         active tasks (with entries of a that are > 0). The function
   *         checks if memory is written over the bounds of H. The dimension
   *         of H is nx x (nx*nq)
   */
  virtual void computeH(MatNd* H, const MatNd* a_des=NULL) const;

  /*! \brief Computes the product dot(J)*qp.
   *
   *         If a_des == NULL, the complete JdotQdot is returned, else
   *         the vector JdotQdot will only comprise elements for the active
   *         tasks. Vector a_des is assumed to hold activations for all
   *         tasks
   *         Vector qp is the joint velocity vector (read from the graph),
   *         not comprising elements for constrained dofs.
   */
  void computeJdotQdot(MatNd* JdotQdot,
                       const MatNd* a_des=NULL) const;

  /*! \brief Computes the vector of task variables according to the graph's
   *         current state.
   *
   *         Vector a_des is of dimension [allTasks x 1]. If it is NULL, the
   *         vector x will be computed with all task elements.
   *         The task vector x is composed of only the parts that correspond
   *         to active tasks (activation > 0). The function checks if
   *         memory is written over the bounds of x.
   *
   *  \param[out] x      Task vector according to current graph's state. It
   *                     must provide memory for the task values of all active
   *                     tasks (according to a_des). Vector x is reshaped to
   *                     the correct dimensions by this function.
   *  \param[in]  a_des  Activation vector of size getTaskDim() x 1. If it is
   *                     NULL, it is assumed to be a vector holding all
   *                     entries of 1. The corresponding entry of x is computed
   *                     if the element of a is > 0.
   */
  virtual void computeX(MatNd* x, const MatNd* a_des=NULL) const;

  /*! \brief Computes the delta in task space for the differential
   *         kinematics. The function checks if memory is written over the
   *         bounds of dx.
   */
  virtual void computeDX(MatNd* dx, const MatNd* x_des,
                         const MatNd* a_des=NULL) const;

  /*! \brief Computes the tracking feedback
   *         \f[
   *         a_x = \ddot{x}_{des} + k_p \Delta x(x_{des},x_{curr})
   *               + k_d \Delta \dot{x} (\dot{x}_{des}, \dot{x}_{curr} )
   *         \f]
   *
   *  \param[out] ax       Resulting tracking acceleration.
   *  \param[in]  a_des    Full activation vector (including inactive
   *                       dimensions)
   *  \param[in]  x_des    Full (including inactive dimensions) vector of
   *                       desired task coordinates.
   *  \param[in]  xp_des   Full (including inactive dimensions) vector of
   *                       desired task velocities.
   *  \param[in]  xpp_des  Full (including inactive dimensions) vector of
   *                       desired task accelerations. If it is NULL, it
   *                       will be ignored.
   *  \param[in]  kp       Task space position gain vector. Its dimensions must
   *                       be [nTasks x 1].
   *  \param[in]  kd       Task space damping gain vector. Its dimensions must
   *                       be [nTasks x 1].
   */
  void computeAx(MatNd* ax, const MatNd* a_des, const MatNd* x_des,
                 const MatNd* xp_des, const MatNd* xpp_des,
                 const MatNd* kp, const MatNd* kd) const;

  /*! \brief Computes the vector of task velocities according to the graph's
   *         current velocity state.
   *
   *         Vector a_des is of dimension [allTasks x 1]
   *         The task velocity vector is composed of only the parts that
   *         correspond  to active tasks (activation > 0). The function
   *         checks if memory is written over the bounds of xp.
   */
  virtual void computeXp(MatNd* xp, const MatNd* a_des=NULL) const;

  /*! \brief Computes the vector of task velocities according to the graph's
   *         current velocity state. They are represented in the velocity
   *         coordinates.
   *
   *         Vector a_des is of dimension [allTasks x 1]
   *         The task velocity vector is composed of only the parts that
   *         correspond  to active tasks (activation > 0). The function
   *         checks if memory is written over the bounds of xp.
   */
  virtual void computeXp_ik(MatNd* xp, const MatNd* a_des=NULL) const;

  /*! \addtogroup TaskLinearDerivative
   *  \brief Adds a velocity to the given state.
   *
   *  \param[out] x_res    State after adding velocity increment
   *  \param[in]  x        State before adding velocity increment
   *  \param[in]  x_dot    Velocity in task velocity space
   *  \param[in]  dt       Time interval
   */
  virtual void integrateXp_ik(MatNd* x_res, const MatNd* x,
                              const MatNd* x_dot, double dt,
                              const MatNd* a_des=NULL) const;

  /*! \brief Computes the vector of task velocity increments according to
   *         the graph's current velocity state.
   *
   *         Vector a_des is of dimension [allTasks x 1]
   *         The task velocity vector is composed of only the parts that
   *         correspond  to active tasks (activation > 0). The function
   *         checks if memory is written over the bounds of dxp.
   */
  virtual void computeDXp(MatNd* dxp, const MatNd* xp_des,
                          const MatNd* a_des=NULL) const;

  /*! \brief Converts the acceleration vector x_ddot into the coordinates the
   *         Jacobian is represented in.
   *
   *  \param[out] x_ddot_ik Resulting acceleration.
   *  \param[in]  x_ddot    Vector of task acceleration. It is represented in
   *                        the task-relevant coordinates. If it is NULL, the
   *                        acceleration is considered to be zero.
   *  \param[in] a_des  The activation vector is NULL, or of dimension
   *                    [allTasks x 1]. If it is NULL, the function returns the
   *                    acceleration vector for all tasks. If it is not NULL,
   *                    only the dimensions of the tasks are counted that have
   *                    a larger activation than 0.
   */
  void computeFfXpp(MatNd* x_ddot_ik, const MatNd* x_ddot,
                    const MatNd* a_des=NULL) const;

  /*! \brief Computes the vector of task accelerations according to the
   *         graph's current acceleration state.
   *
   *         Vector a_des is of dimension [allTasks x 1]
   *         The task acceleration vector is composed of only the parts that
   *         correspond  to active tasks (activation > 0). The function
   *         checks if memory is written over the bounds of xpp.
   *         qpp is not stored in the graph and must hence be passed.
   */
  virtual void computeXpp(MatNd* xpp, const MatNd* qpp,
                          const MatNd* a_des=NULL) const;

  /*! \brief Augments a vector xc of only the task elements that correspond
   *         to activations larger than 0.
   */
  virtual void compressToActive(MatNd* xc,
                                const MatNd* x,
                                const MatNd* activations) const;

  /*! \brief In-place version of compressToActive().
   */
  virtual void compressToActiveSelf(MatNd* x,
                                    const MatNd* activations) const;

  /*! \brief Inverse operation of \ref compressToActive, unknown elements
   *         are set to 0
   */
  void decompressFromActive(MatNd* x, const MatNd* xc,
                            const MatNd* activations) const;

  /*! \brief In-place version of \ref decompressFromActive.
   */
  void decompressFromActiveSelf(MatNd* xc, const MatNd* activations) const;

  /*! \brief Given a vector of activations a with the number of rows
   *         corresponding to the number of tasks, the function copies
   *         the activation value to all task elements of the corresponding
   *         task.
   *
   * \param[out] x   Task vector, will be reshaped to the correct dimensions.
   * \param[in]  a   Activation vector, must be [num tasks x 1]. If this is not
   *                 the case, the function will exit fatally.
   */
  void decompressActivationToTask(MatNd* x, const MatNd* a) const;

  /*! \brief In-place version of the above \ref decompressActivationToTask
   *         function.
   *
   * \param[in,out] a_to_x   Activation vector of size [num tasks x 1]. It must
   *                         have enough memory for task-dim elements,
   *                         otherwise the function exits fatally.
   */
  void decompressActivationToTask(MatNd* a_to_x) const;

  /*! \brief Computes the joint limit costs. This function uses the
   *         method RcsGraph_jointLimitCost().
   */
  virtual double computeJointlimitCost() const;

  /*! \brief Gradient for the function computeJointlimitCost(). The gradient
   *         is an 1 x dof vector, where dof is the number of unconstrained
   *         degrees of freedom of the system.
   */
  virtual void computeJointlimitGradient(MatNd* grad) const;

  /*! \brief Computes the joint limit costs. This function uses the
   *         method RcsGraph_jointLimitBorderCost(). The variable
   *         borderRatio indicates the joint range in which the cost is
   *         zero. Only when the joint value is beyond the given
   *         borderRatio, a quadratic penalty is computed. See
   *         RcsGraph_jointLimitBorderCost() for details.
   */
  virtual double computeJointlimitBorderCost(double borderRatio) const;

  /*! \brief Gradient for the function computeJointlimitCost(). The gradient
   *         is an 1 x dof vector, where dof is the number of unconstrained
   *         degrees of freedom of the system.
   */
  virtual void computeJointlimitBorderGradient(MatNd* grad,
                                               double borderRatio) const;

  /*! \brief Computes the collision model. This function traverses the
   *         collision pairs that are defined in the underlying collision
   *         model (member cMdl), and calculatse the proximities between
   *         them.
   */
  virtual void computeCollisionModel();

  /*! \brief Computes the collision costs. This function updates the
   *         collision model with \ref computeCollisionModel(), and then
   *         calculates the cost contribution of each pair to an overall
   *         cost.
   */
  virtual double computeCollisionCost();

  /*! \brief Computes the collision costs. This function assumes an updated
   *         collision model that has been computed with
   *         \ref computeCollisionModel() before. It calculates the cost
   *         contribution of each pair to an overall cost.
   */
  virtual double getCollisionCost() const;

  /*! \brief Gradient for the function computeCollisionCost(). The function
   *         computes the collision model using \ref computeCollisionModel()
   *
   *  \param[out] grad Collision gradient as a 1 x dof vector, where dof is
   *              the number of unconstrained (RcsGraph::nJ) degrees of
   *              freedom of the system.
   */
  virtual void computeCollisionGradient(MatNd* grad);

  /*! \brief Gradient for the function computeCollisionCost(). The gradient
   *         is an 1 x dof vector, where dof is the number of unconstrained
   *         degrees of freedom of the system. This function does not
   *         compute the collision model, but retrieves it from the member
   *         \ref cMdl.
   *
   *  \param[out] grad Collision gradient as a 1 x dof vector, where dof is
   *              the number of unconstrained (RcsGraph::nJ) degrees of
   *              freedom of the system.
   */
  virtual void getCollisionGradient(MatNd* grad) const;

  /*! \brief Calculates the squared distance to the target in task
   *         coordinates as
   *         \f[
   *         cost = \mathbf{ \Delta x^T W \Delta x }
   *         \f]
   *         where \f$ \mathbf{\Delta x} \f$ is being computed with the
   *         function computeDX().
   *
   *  \param[in] xDes Vector of desired task coordinates, comprising all
   *                  tasks (also the inactive ones).
   *  \param[in] W Optional diagonal weighting vector. If it is NULL, each
   *               task is weighted uniformly. If W is not NULL, it must be
   *               a vector of dimension dim(active tasks) x 1.
   *  \param[in] activation The activation vector is NULL, or of dimension
   *                        [allTasks x 1]. If it is NULL, the function
   *                        returns the task dimension of all tasks that
   *                        are labelled for the respective task priority.
   *                        If it is not NULL, only the dimensions of the
   *                        tasks are counted that have a larger activation
   *                        than 0.
   *  \return Squared distance to the target in task coordinates.
   */
  virtual double computeTaskCost(const MatNd* xDes,
                                 const MatNd* W=NULL,
                                 const MatNd* activation=NULL) const;

  /*! \brief Gradient for the function \ref computeTaskCost() according to
   *         \f[
   *         \frac{\partial c}{\partial \mathbf{q}} = \mathbf{\Delta x^T W J}
   *         \f]
   *         where \f$ \mathbf{\Delta x} \f$ is being computed with the
   *         function computeDX().
   *
   *  \param[out] grad Collision gradient as a 1 x dof vector, where dof is
   *              the number of unconstrained (RcsGraph::nJ) degrees of
   *              freedom of the system.
   *  \param[in] xDes Vector of desired task coordinates, comprising all
   *                  tasks (also the inactive ones).
   *  \param[in] W Optional diagonal weighting vector. If it is NULL, each
   *               task is weighted uniformly. If W is not NULL, it must be
   *               a vector of dimension dim(active tasks) x 1.
   *  \param[in] activation The activation vector is NULL, or of dimension
   *                        [allTasks x 1]. If it is NULL, the function
   *                        returns the task dimension of all tasks that
   *                        are labelled for the respective task priority.
   *                        If it is not NULL, only the dimensions of the
   *                        tasks are counted that have a larger activation
   *                        than 0.
   */
  virtual void computeTaskGradient(MatNd* grad,
                                   const MatNd* xDes,
                                   const MatNd* W=NULL,
                                   const MatNd* activation=NULL) const;

  /*! \brief Computes the Manipulability index according to Yoshikawa:
   *         w = sqrt(det(J*W*J^T)).
   *
   *  \param[in] a_des  The activation vector is NULL, or of dimension
   *                    [allTasks x 1]. If it is NULL, the function returns the
   *                    manipulability index for all tasks. If it is not NULL,
   *                    only the dimensions of the tasks are counted that have
   *                    a larger activation than 0.
   *  \param[in] W   n x 1 weight vector with n being the dimension of the
   *                 Jacobian's number of rows. If it is NULL, an identity
   *                 weighting is applied.
   *  \return sqrt(det(J*W*J^T))
   */
  double computeManipulabilityCost(const MatNd* a_des=NULL,
                                   const MatNd* W=NULL) const;

  /*! \brief Computes the gradient of the Manipulability index according to
   *         \ref computeManipulabilityCost(). For more details, please see
   *         \ref MatNd_computeManipulabilityIndexGradient().
   *
   *  \param[out] grad  Gradient of dimension 1 x nJ. It is reshaped in the
   *                    function.
   *                    tasks according to the activation vector a_des. If it is
   *                    NULL, an identity matrix is used.
   *  \param[in] a_des  The activation vector is NULL, or of dimension
   *                    [allTasks x 1]. If it is NULL, the function returns the
   *                    manipulability index for all tasks. If it is not NULL,
   *                    only the dimensions of the tasks are counted that have
   *                    a larger activation than 0.
   *  \param[in] W   n x 1 weight vector with n being the dimension of the
   *                 Jacobian's number of rows. If it is NULL, an identity
   *                 weighting is applied.
   *  \return sqrt(det(J*W*J^T))
   */
  double computeManipulabilityGradient(MatNd* grad,
                                       const MatNd* a_des=NULL,
                                       const MatNd* W=NULL) const;

  /*! \brief Computes the current task forces based on the values of all
   *         FT sensors:
   *         f_task = J_task (J_sensor1# f_sensor1 + J_sensor2# f_sensor2 ...
   *
   *  \param[in] ft_task   Force in task coordinates.
   *  \param[in] a_des  The activation vector is NULL, or of dimension
   *                    [allTasks x 1]. If it is NULL, the function returns the
   *                    ft_task vector for all tasks. If it is not NULL,
   *                    only the dimensions of the tasks are counted that have
   *                    a larger activation than 0.
  */
  virtual void computeTaskForce(MatNd* ft_task, const MatNd* a_des=NULL) const;

  /*! \brief Computes the displacement of a compliant frame based on an
   *         admittance control law.
   *
   *  \param[out] compliantVec Vector of compliant task coordinates. The
   *                           function adds a small displacement to it.
   *                           The caller has to make sure it is consistent
   *                           over time. One can imagine this vector as the
   *                           task space equivalent to a compliance frame.
   *  \param[in] ft_task   Desired force in task coordinates.
   *  \param[in] Kp        Position gain for admittance
   *  \param[in] a_des  The activation vector is NULL, or of dimension
   *                    [allTasks x 1]. If it is NULL, the function returns the
   *                    compliant displacements for all tasks. If it is not
   *                    NULL, only the dimensions of the tasks are counted
   *                    that have a larger activation than 0.
   */
  virtual void computeAdmittance(MatNd* compliantVec,
                                 const MatNd* ft_task,
                                 const MatNd* Kp,
                                 const MatNd* a_des);

  /*! \brief Returns the name of the controller as indicated in the xml
   *         file.
   *
   *  \return Reference to name string.
   */
  virtual const std::string& getName() const;

  /*! \brief Returns the name of the controller's xml file.
   *
   *  \return Reference to  string.
   */
  virtual const std::string& getXmlFileName() const;

  /*! \brief Runs through all tasks and performs the task's test function.
   *         Returns true if all tests have succeeded, false otherwise.
   */
  virtual bool test(bool verbose=false);

  /*! \brief Closes the parsed xml file and destroys all xml memory.
   */
  virtual void closeXmlFile();

  /*! \brief Returns the xml root node of the controller definition file.
   *
   *  \return Controller file root xml node, or NULL if the xml file is not
   *          open.
   */
  virtual xmlNodePtr getXmlNode() const;

  virtual bool add(const ControllerBase& other, const char* suffix,
                   const HTr* A_BP);

  /*! \brief Adds the task to the controller's task vector and updates the
   *         task array indices. If other is NULL, the function returns without
   *         doing anything (just complains on debug level 4).
   */
  virtual void add(Task* other);

  /*! \brief Prints out a task vector x with task names and additional
   *         information. Only active tasks (a_des of the corresponding
   *         task > 0) are printed.
   *
   *  \param[in] x       Task vector to be printed. Must be of size
   *                     getTaskDim() x 1 (larger is ok).
   *  \param[in] a_des   Activation. Only task vector elements with the task
   *                     activation >0 are considered. If a_des is NULL, all
   *                     tasks are printed.
   */
  virtual void printX(const MatNd* x, const MatNd* a_des = NULL) const;

  /*! \brief See \ref RcsGraph_getModelStateFromXML();
   */
  bool getModelState(MatNd* q, const char* modelStateName, int timeStamp=0);

  /*! \brief Performs various checks on the controller. All margins are in
   *         SI units.
   *
   *  \param checkJointLimits     Perform joint limit check for all joints.
   *  \param checkCollisions      Perform collision checks for collision model.
   *  \param checkJointVelocities Check if joint velocities are within range.
   *  \param jlMarginAngular      Joint limit safety margin for angular joints.
   *  \param jlMarginLinear       Joint limit safety margin for linear joints.
   *  \param collMargin           Safety margin for collision check.
   *  \param speedMarginAngular   Speed limit safety margin for angular joints.
   *  \param speedMarginLinear    Speed limit safety margin for linear joints.
   *  \return True for no violation, false otherwise.
   */
  bool checkLimits(bool checkJointLimits=true,
                   bool checkCollisions=true,
                   bool checkJointVelocities=true,
                   double jlMarginAngular=0.0,
                   double jlMarginLinear=0.0,
                   double collMargin=0.0,
                   double speedMarginAngular=0.0,
                   double speedMarginLinear=0.0) const;

  /*! \brief Swaps the task vector newTasks with the one of the class.
   *
   *  \param[in] newTasks   Tasks to be swapped. The vector newTasks will
   *                        contain the class's task vector afterwards.
   *  \param[in] recomputeArrayIndices   The class's array indices will be
   *                                     recomputed if this argument is true.
   */
  void swapTaskVec(std::vector<Task*>& newTasks,
                   bool recomputeArrayIndices=false);

  /*! \brief Prints information about tasks to console.
   */
  virtual void print() const;

  /*! \brief Prints whatever is written in the xml-file under the node tag
   *         usage to the console
   */
  static void printUsage(const std::string& xmlFile);

  /*! \brief Computes the inverse dynamics for joint-space tracking:
   *         \f[
   *         T_{des} = M a_q + h + g
   *         \f]
   *
   *         with
   *         \f$
   *         a_q = \ddot{q} -k_p (q-q_{des}) - k_d (\dot{q}-\dot{q}_{des})
   *         \f$
   *
   *  \param[out] T_des   Desired joint torque. It will be reshaped to the
   *                      full dimension (RcsGraph::dof x 1)
   *  \param[in] graph    Underlying graph
   *  \param[in]  q_des   Desired joint angles. They must be of the full
   *                      dimension (RcsGraph::dof x 1)
   *  \param[in]  qp_des  Desired joint velocities. If qp_des is NULL, they
   *                      are assumed to be zero. Otherwise they must be of
   *                      the full dimension (RcsGraph::dof x 1)
   *  \param[in]  qpp_des Desired joint angles. If qpp_des is NULL, they
   *                      are assumed to be zero. Otherwise they must be of
   *                      the full dimension (RcsGraph::dof x 1)
   *  \param[in]  positionGain   Position gain for PD control
   *  \param[in]  velocityGain   Velocity gain for PD control. If it is -1, the
   *                             damping is set to asymptotic behavior
   *                             (velocityGain = 0.5*sqrt(4.0*positionGain))
   */
  static void computeInvDynJointSpace(MatNd* T_des,
                                      const RcsGraph* graph,
                                      const MatNd* q_des,
                                      const MatNd* qp_des,
                                      const MatNd* qpp_des,
                                      double positionGain,
                                      double velocityGain);

  /*! \brief Computes the inverse dynamics for joint-space tracking:
   *         \f[
   *         T_{des} = M a_q + h + g
   *         \f]
   *
   *         with
   *         \f$
   *         a_q = -k_p (q-q_{des}) - k_d \dot{q}
   *         \f$
   *
   *  \param[out] T_des   Desired joint torque. It will be reshaped to the
   *                      full dimension (RcsGraph::dof x 1)
   *  \param[in] graph    Underlying graph
   *  \param[in]  q_des   Desired joint angles. They must be of the full
   *                      dimension (RcsGraph::dof x 1)
   *  \param[in]  positionGain   Position gain for PD control
   *  \param[in]  velocityGain   Velocity gain for PD control. If it is -1, the
   *                             damping is set to asymptotic behavior
   *                             (velocityGain = 0.5*sqrt(4.0*positionGain))
   */
  static void computeInvDynJointSpace(MatNd* T_des,
                                      const RcsGraph* graph,
                                      const MatNd* q_des,
                                      double positionGain,
                                      double velocityGain=-1.0);

protected:

  RcsGraph* graph;                   //!< Underlying graph
  bool ownsGraph;                    //!< True if controller needs to destroy it
  std::vector<Task*> tasks;          //!< List of tasksof the controller

private:

  xmlNodePtr xmlRootNode;            //!< XML root node during ctors
  xmlDocPtr xmlDoc;                  //!< Pointer for XML parsing context
  RcsCollisionMdl* cMdl;             //!< Collision model
  std::string name;                  //!< Name of the controller
  std::string xmlFile;               //!< Configuration file name
  std::vector<size_t> taskArrayIdx;  //!< List of indices in task vector
};


}   // namespace Rcs


#endif // RCS_CONTROLLERBASE_H
