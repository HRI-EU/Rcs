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

#ifndef RCS_TASK_H
#define RCS_TASK_H

#include "Rcs_graph.h"

#include <libxml/tree.h>

#include <vector>
#include <string>
#include <cstdio>


/*!
 *  \defgroup RcsTask Task classes
 *
 *  A library for various task implementations. The base class \ref Rcs::Task
 *  implements a few get and set methods. It also contains an interface to the
 *  specializations to be derieved from it. For convenience, the derieved class
 *  \ref Rcs::TaskGenericIK implements methods that assume the same task
 *  coordinate  representation on position and velocity levels. This is for
 *  instance the case for the cartesian position. In other cases the
 *  representation on velocity level is chosen differently in order to avoid
 *  singularities etc. For instance the Euler angles task represents the
 *  task-level velocities as angular velocities, and not Euler velocities.
 *  The class \ref Rcs::Task also implements methods to compute the
 *  kinematics of end effectors. It is possible to define three different
 *  rigid bodies: the effector, the refBody, and the refFrame. The
 *  transformation of the effector is represented relative to the refBody. If
 *  the refBody wasn't defined, the effector is represented in world
 *  coordinates. If both a refFrame and a refBody have been specified, the
 *  coordinates of the effector are represented in the refBodie's frame, but
 *  rotated into the refFrame.
 *
 *  Here is what to do for implementing a new task:
 *
 *  Case 1: Positions, velocities and accelerations are represented in the same
 *          coordinate frame. The class can inherit from
 *          \ref Rcs::TaskGenericIK, and just these methods need to be
 *          implemented:
 *          - Rcs::Task::clone()
 *          - Rcs::Task::computeX()
 *          - Rcs::Task::computeJ()
 *          - Rcs::Task::computeH()
 *
 *
 *  Case 2: Positions are represented in a different coordinate frame
 *          than velocities and accelerations (e.g. Rcs::TaskEuler3D). In
 *          addition to case 1, the following methods need to be implemented:
 *          - Rcs::Task::computeDX()
 *          - Rcs::Task::computeXp()
 *          - Rcs::Task::computeDXp()
 *          - Rcs::Task::integrateXp_ik()
 *          - Rcs::Task::computeXpp()
 *          - Rcs::Task::computeFfXpp()
 *
 *
 *  The following methods are automatically computed (see implementation in
 *  Task.cpp):
 *         - Rcs::Task::computeJdot()           uses Rcs::Task::computeH()
 *         - Rcs::Task::computeJdotQdot()       uses Rcs::Task::computeJdot()
 *         - Rcs::Task::computeTaskCost()       uses Rcs::Task::computeDX()
 *         - Rcs::Task::computeTaskGradient()   uses Rcs::Task::computeDX(),
 *                                                   Rcs::Task::computeJ()
 *         - Rcs::Task::computeAX()             uses Rcs::Task::computeDX(),
 *                                                   Rcs::Task::computeDXp(),
 *                                                   Rcs::Task::computeXp(),
 *                                                   Rcs::Task::computeXpp()
 *         - Rcs::Task::computeXp_ik()    not efficient (x_dot = J q_dot)
 *         - Rcs::Task::computeXpp_ik()   not efficient (x_ddot = J q_ddot +
 *                                                                J_dot q_dot)
 *
 *  The complete interface to the \ref Rcs::Task class is
 *
 *  - Rcs::Task::clone()
 *  - Rcs::Task::computeX()
 *  - Rcs::Task::computeJ()
 *  - Rcs::Task::computeH()
 *
 *  - Rcs::Task::computeDX()
 *  - Rcs::Task::computeXp()
 *  - Rcs::Task::computeDXp()
 *  - Rcs::Task::integrateXp_ik()
 *  - Rcs::Task::computeXpp()
 *  - Rcs::Task::computeFfXpp()
 */

namespace Rcs
{
/*! \ingroup RcsTask
 *  \brief Base class for all tasks. Contains basic interface.
 *
 *         A Task should not contain any state. All calculating function
 *         have target variables as arguments, plus possible input (state)
 *         variables.
 */
class Task
{
public:

  /*! \brief A small helper class to contain information about (constant)
   *         tasks parameters
   */
  struct Parameters
  {
    Parameters(const double min, const double max,
               const double scaleFactor, const std::string& name);

    void setParameters(const double min, const double max,
                       const double scaleFactor, const std::string& name);
    double minVal;
    double maxVal;
    double scaleFactor;
    std::string name;
  };

  /*! \brief Default constructor
   */
  Task();

  /*! \brief Constructor based on xml parsing
   */
  Task(const std::string& className, xmlNode* node, RcsGraph* graph,
       int dim=0);

  /*! \brief Copy constructor doing deep copying with optional new graph
   *         pointer
   */
  Task(const Task& copyFromMe, RcsGraph* newGraph=NULL);

  /*! \brief Destructor
   */
  virtual ~Task();

  /*! \brief Virtual copy constructor with optional new graph
   */
  virtual Task* clone(RcsGraph* newGraph=NULL) const = 0;

  /*! \brief Returns the dimension of the task.
   */
  virtual unsigned int getDim() const;

  /*! \brief Sets the name of the task.
   */
  void setName(const std::string& name);

  /*! \brief Returns reference to the name of the task
   */
  const std::string& getName() const;

  /*! \brief Returns the parameters for task dimension index. If argument
   *         index is out of the range of the parameter vector, the function
   *         exits with a fatal error.
   */
  Parameters& getParameter(size_t index);

  /*! \brief Same as above, but non-modifyable pointer.
   */
  const Parameters& getParameter(size_t index) const;

  /*! \brief Returns the whole parameter list
   */
  const std::vector<Parameters>& getParameters() const;

  /*! \brief Returns the whole parameter list
   */
  std::vector<Parameters>& getParameters();

  /*! \brief Clears the vector of parameters so that none are contained.
   */
  void clearParameters();

  /*! \brief Adds a parameter instance to the vector of parameters.
   */
  void addParameter(const Parameters& newParam);

  /*! \brief Removes the parameter at the given index from the parameter
   *         vector.
   */
  bool removeParameter(size_t index);

  /*! \brief Clears all parameters and adds this one.
   */
  void resetParameter(const Parameters& newParam);

  /*! \brief Returns a copy of the string that holds the task's class
   *         name. The class name is the name to be used in the xml file's
   *         tag "controlVariable". We return a copy instead of a reference,
   *         since we don't want anybody to change the className member.
   */
  std::string getClassName() const;

  /*! \brief Sets the className member.
   */
  void setClassName(const std::string& className);

  /*! \brief Returns a pointer to the task's underlying graph.
   */
  RcsGraph* getGraph() const;

  /*! \brief Prints information about task to console.
   */
  virtual void print() const;

  /*! \brief Returns true if the task is specified correctly, false
   *         otherwise. The task is invalid if
   *         - No tag "name" exists
   *         - No tag "controlVariable" exists
   *         - The tag "controlVariable" exists, but its contents are not
   *           equal to the argument className (case insensitive)
   */
  static bool isValid(xmlNode* node, const RcsGraph* graph,
                      const char* className);

  /*! \brief Returns true if the task is specified correctly, false
   *         otherwise. The task is invalid if
   *         - No tag "name" exists
   *         - No tag "controlVariable" exists
   *         - The tag "controlVariable" exists, but its contents are not
   *           equal to one of the strings in the vector className
   *           (case insensitive)
   */
  static bool isValid(xmlNode* node, const RcsGraph* graph,
                      const std::vector<std::string>& className);



  /**
   * @name TaskSpecialization
   *
   * To be implemented in each task.
   */

  ///@{

  /*! \addtogroup TaskSpecialization
   *  \brief Computes the current value of the task variable
   *
   *  The result is written to parameter \e x_res.
   */
  virtual void computeX(double* x_res) const = 0;

  /*! \addtogroup TaskSpecialization
   *  \brief Computes current task Jacobian to parameter \e jacobian
   */
  virtual void computeJ(MatNd* jacobian) const = 0;

  /*! \addtogroup TaskSpecialization
   *  \brief Computes the current task Hessian to parameter \e hessian
   */
  virtual void computeH(MatNd* hessian) const = 0;

  ///@}



  /**
   * @name TaskLinearDerivative
   *
   * To be implemented in tasks that have the same representation of the
   * control variable on velocity level (TaskGenericIK).
   */

  /*@{*/

  /*! \addtogroup TaskLinearDerivative
   *  \brief Computes the delta in task space for the differential
   *         kinematics. The current task values are taken from x_curr (and
   *         not from the graph's state sa usual).
   */
  virtual void computeDX(double* dx, const double* x_des,
                         const double* x_curr) const = 0;

  /*! \addtogroup TaskMethodsThroughInheritance
   *  \brief Computes the current velocity in task space.
   *         It needs to be overwritten for task variables that don't
   *         have velocities and positions represented in the same frame
   *         (e.g. for Euler angles, J might be representing the angular
   *         velocities instead of the Euler velocities).
   *
   *  \param[out] x_dot Task space velocity
   */
  virtual void computeXp(double* x_dot) const = 0;

  /*! \addtogroup TaskLinearDerivative
   *  \brief Computes the delta in task space for the differential kinematics
   */
  virtual void computeDXp(double* dx_dot, const double* x_dot_des) const = 0;

  /*! \addtogroup TaskLinearDerivative
   *  \brief Computes the current acceleration in task space.
   *
   *  \param[out] x_ddot Task space acceleration
   *  \param[in]  qpp Joint space acceleration
   */
  virtual void computeXpp(double* x_ddot, const MatNd* qpp) const = 0;

  /*! \addtogroup TaskLinearDerivative
   *  \brief Computes the feed-forward acceleration. The argument x_ddot_res
   *         is expected to be in Jacobi-coordinate.
   *
   *  \param[out] x_ddot_res    Resulting acceleration. It is represented
   *                            in the IK-relevant coordinates.
   *  \param[in]  x_ddot_des    Vector of desired task acceleration. It is
   *                            represented in the task-relevant coordinates.
   */
  virtual void computeFfXpp(double* x_ddot_res,
                            const double* x_ddot_des) const = 0;

  /*! \addtogroup TaskLinearDerivative
   *  \brief Adds a velocity increment to the given state. For tasks with the
   *         same coordinate representation on position and velocity level,
   *         this is a trivial calculation. For others (e.g. rotations), some
   *         non-linear conversions need to be done. In some cases, this
   *         calculation can even not be done.
   *
   *  \param[out] x_res    State after adding velocity increment
   *  \param[in]  x        State before adding velocity increment
   *  \param[in]  x_dot    Velocity in task velocity space
   *  \param[in]  dt       Time interval
   */
  virtual void integrateXp_ik(double* x_res, const double* x,
                              const double* x_dot, double dt) const = 0;

  virtual void forceTrafo(double* ft_task) const = 0;

  virtual void selectionTrafo(double* S_trans, const double* S) const = 0;

  ///@}



  /**
   * @name TaskMethodsThroughInheritance
   *
   * These methods are generic if the other virtual functions are
   * implemented.
   */

  /*@{*/

  /*! \addtogroup TaskMethodsThroughInheritance
   *  \brief Computes the current velocity in task velocity space. In this
   *         class's implementation, it is calculated as
   *         \f$
   *         \mathbf{\dot{x} = J \dot{q} }
   *         \f$
   *         It should be overwritten to be more efficient.
   *
   *  \param[out] x_dot Task space velocity
   */
  virtual void computeXp_ik(double* x_dot) const;

  /*! \addtogroup TaskMethodsThroughInheritance
   *  \brief Computes the current acceleration in task velocity space. In this
   *         class's implementation, it is calculated as
   *         \f$
   *         \mathbf{\ddot{x} = J \ddot{q}  + \dot{J} \dot{q}}
   *         \f$
   *
   *  \param[in]  q_ddot Joint space acceleration vector of dimension
   *                     RcsGraph::nJ x 1 (Unconstrained dof only).
   *  \param[out] x_ddot Task space acceleration in task velocity space.
   */
  virtual void computeXpp_ik(double* x_ddot, const MatNd* q_ddot) const;

  /*! \addtogroup TaskMethodsThroughInheritance
   *  \brief Computes the displacement in task space
   *         \f$
   *         \mathbf{dx = x_{des} - computeX() }
   *         \f$
   *         to reach the desired value \e x_des. The current task space
   *         values are calculated from the graph's state through method
   *         \ref computeX().
   *
   *  \param[out] dx Task space displacement
   *  \param[in] x_des desired task space coordinates
   */
  virtual void computeDX(double* dx, const double* x_des) const;

  /*! \addtogroup TaskMethodsThroughInheritance
   *  \brief Computes the current derivative Jacobian as
   *         \f$
   *         \mathbf{ \dot{J} = \dot{q}^T H}
   *         \f$.
   *         Matrix H is the task's Hessian.
   */
  virtual void computeJdot(MatNd* Jdot) const;

  /*! \addtogroup TaskMethodsThroughInheritance
   *  \brief Computes a scalar penalty for reaching a given target as
   *         \f[
   *         c = \mathbf{ \Delta x^T W \Delta x }
   *         \f]
   *         where \f$ \mathbf{\Delta x} \f$ is being computed with the
   *         function computeDX(). If W is NULL, an identity weighting matrix
   *         is assumed.
   */
  virtual double computeTaskCost(const double* x_des,
                                 const double* diagW=NULL) const;

  /*! \addtogroup TaskMethodsThroughInheritance
   *  \brief Computes the gradient of the function computeTaskCost()
   *         according to
   *         \f[
   *         \frac{\partial c}{\partial \mathbf{q}} = \mathbf{\Delta x^T W J}
   *         \f]
   *         where \f$ \mathbf{\Delta x} \f$ is being computed with the
   *         function computeDX(). If W is NULL, an identity weighting matrix
   *         is assumed. The gradient dcDq is a vector of dimension 1 x nq,
   *         where nq is the number of unconstrained degrees of freedom.
   */
  virtual void computeTaskGradient(MatNd* dcDq,
                                   const double* x_des,
                                   const double* diagW=NULL) const;

  /*! \addtogroup TaskMethodsThroughInheritance
   *  \brief Computes the term
   *         \f[
   *         \mathbf{\dot{J} \dot{q} = \dot{q}^T H \dot{q}}
   *         \f]
   *         The matrix JdotQdot is of dimension nx x nq, where nx is
   *         the dimension of the task and nq is the number of
   *         unconstrained degrees of freedom.
   *         The vector qp (read from graph) is a vector of dimension nq x 1.
   *
   */
  virtual void computeJdotQdot(MatNd* JdotQdot) const;

  /*! \addtogroup TaskMethodsThroughInheritance
   *  \brief Calculate the tracking acceleration
   *         \f[
   *         a_{x} = \ddot{x}_{des} + kp \Delta x + kd \Delta \dot{x}
   *                                + k_i \int  \Delta x dt
   *         \f]
   *         with all vectors having as many elements as the task has
   *         dimensions.
   *
   *  \param[out] a_res    Resulting tracking acceleration. It is represented
   *                       in the IK-relevant coordinates (e.g. angular
   *                       and not Euler acceleration for Euler task).
   *  \param[in,out] integral_x Integral position error. If it is NULL,
   *                       it will be ignored.
   *  \param[in]  x_des    Vector of desired task coordinates.
   *  \param[in]  x_dot_des   Vector of desired task velocities.
   *  \param[in]  x_ddot_des  Vector of desired task accelerations. If it is
   *                       NULL, it will be ignored.
   *  \param[in]  S_des    Selection vector (dimension of task) for kinematic
   *                       (1) and force (0) tracking. It only has a meaning
   *                       if x_int is not NULL. If in this case S_des is
   *                       NULL, the integral term will be computed for
   *                       kinematic tracking only.
   *  \param[in]  a_des    Task activation. It is only considered in the
   *                       integral term as a scaling factor.
   *  \param[in]  kp       Position gain
   *  \param[in]  kd       Velocity gain
   *  \param[in]  ki       Integral gain
   */
  virtual void computeAX(double* a_res,
                         double* integral_x,
                         const double* x_des,
                         const double* x_dot_des,
                         const double* x_ddot_des,
                         const double* S_des,
                         const double a_des,
                         const double kp,
                         const double kd,
                         const double ki) const;

  /*! \brief This function computes the desired task space acceleration
   *         related to the desired forces. It is not a very nice function,
   *         however, the large argument list is really needed to deal
   *         component-wise with selection and the integral term. This
   *         implementation is generic for all tasks, if ft_des and ft_task
   *         are represented on task velocity level.
   *
   *         The force Control \f$a_f\f$ is composed of a feed forward, a
   *         proportional and an integral term. If argument ft_task is NULL,
   *         the result only considers the feed forward term:
   *         \f[
   *         a_f = f_{des}
   *         \f]
   *         Otherwise, the task space force is considered: The force
   *         error term is
   *         \f[
   *         f_{err} = f_{des} - f_{curr}
   *         \f]
   *         The integral term is updated as
   *         \f[
   *         f_{int} = f_{int} + k_i a_{des}(I-S)f_{err}
   *         \f]
   *         It means that if the selection vector element is set to
   *         tracking (tracking: 1, force: 0), the integral error is not
   *         being updated. The resulting force acceleration is
   *         \f[
   *         a_f = f_{des} + k_p f_{err} + f_{int}
   *         \f]
   *
   *  \param[out] ft_res    Resulting force acceleration. It is represented
   *                        in the IK-relevant coordinates (e.g. angular
   *                        and not Euler acceleration for Euler task).
   *  \param[in,out] ft_int Integral force error. If it is NULL,
   *                        it will be ignored. If not, it is integrated
   *                        according to the above equation. A wind-up limit
   *                        of 10*a_des is imposed (TODO:Make more generic)
   *  \param[in]  ft_des    Vector of desired forces.
   *  \param[in]  S_des     Selection vector (dimension of task) for kinematic
   *                        (1) and force (0) tracking. It only has a meaning
   *                        if x_int is not NULL. If in this case S_des is
   *                        NULL, the integral term will be updated for
   *                        kinematic tracking only.
   *  \param[in]  ft_task   Measured force, projected into the task space
   *                        coordinates. If it is NULL, only the feed forward
   *                        term is considered.
   *  \param[in]  a_des     Task activation. It is only considered in the
   *                        integral term as a scaling factor.
   *  \param[in]  kp        Position gain
   *  \param[in]  ki        Integral gain
   */
  virtual void computeAF(double* ft_res,
                         double* ft_int,
                         const double* ft_des,
                         const double* S_des,
                         const double* ft_task,
                         const double a_des,
                         const double kp,
                         const double ki) const;

  /*! \brief Projects the load cell sensor forces to the task space
   *         coordinates according to
   *         \f[
   *         f_{task} = J_{task} J_{sensor}^{\#} f_{sensor}
   *         \f]
   *
   *  \param[out] ft_task  Vector of task space forces of dimension
   *                       [taskDim x 1]. If loadCell is NULL, it will be set
   *                       to 0. The function will reshape ft_task to the
   *                       correct dimensions.
   *  \param[in] fts       Load cell with 6-d force wrench measurement. This
   *                       will be projected on the ft_task vector. It is
   *                       assumed that the sensor's rawData field has been
   *                       updated with the measurements.
   */
  virtual void projectTaskForce(MatNd* ft_task, const RcsSensor* fts) const;

  ///@}





  /**
   * @name TaskEffector
   *
   * These methods are related to end effector tasks
   */

  /*@{*/

  /*! \brief Returns the effector body of the task, or NULL if none exists.
   */
  virtual const RcsBody* getEffector() const;

  /*! \brief Returns the refBdy of the task, or NULL if none exists.
   */
  virtual const RcsBody* getRefBody() const;

  /*! \brief Returns the refFrame of the task, or NULL if none exists.
   */
  virtual const RcsBody* getRefFrame() const;

  /*! \brief Overwrites the effector body of the task.
   */
  virtual void setEffector(const RcsBody* effector);

  /*! \brief Overwrites the reference body of the task.
   */
  virtual void setRefBody(const RcsBody* referenceBody);

  /*! \brief Overwrites the reference frame of the task.
   */
  virtual void setRefFrame(const RcsBody* referenceFrame);

  /*! \brief Computes the relative rotation matrix between effector and
   *         reference body according to
   *         \f$
   *         \mathbf{ A_{ER} = A_{EI} A_{RI}^T }
   *         \f$
   *         If bdyEff and / or bdyRef is NULL, the identity matrix is
   *         used.
   *
   *  \param[in]  bdyEff Effector body (may be NULL)
   *  \param[in]  bdyRef Reference body (may be NULL)
   *  \param[out] A_ER   Relative rotation matrix
   */
  static void computeRelativeRotationMatrix(double A_ER[3][3],
                                            const RcsBody* bdyEff,
                                            const RcsBody* bdyRef);

  /*! \brief Computes the current XYZ Euler angles between effector and
   *         reference body.
   */
  static void computeEulerAngles(double* ea,
                                 const RcsBody* effector,
                                 const RcsBody* referenceBody);

  /*! \brief Computes the angular velocity of the effector with respect to
   *         the refBody. If refBody is NULL, it is the angular velocity
   *         of the effector with respect to the world frame.
   */
  virtual void computeOmega(double* omega_curr) const;

  /*! \brief Checks if effector exists in the graph, if it is specified in the
   *         xml node.
   *
   *  \return True if
   *          - the string tag does not exist in the xmlNode
   *          - the string tag does exist in the xmlNode, and the corresponding
   *            body exists in the graph
   */
  static bool checkBody(xmlNode* node, const char* tag,
                        const RcsGraph* graph, const char* taskName);


  ///@}

  /**
   * @name TaskTestingMethods
   *
   * These tests are generic if the other virtual functions are
   * implemented.
   */

  /*@{*/

  /*! \brief Performs a gradient test for the task using a finite difference
   *         method. The finite difference Jacobian is constructed column-
   *         wise by computing the tasks's computeDX() function with a
   *         slightly permuted state vector.
   *
   *  \param[in] errorLimit Limit on permissible error for each component of
   *                        the gradient. It is calculated according to the
   *                        argument relativeError.
   *  \param[in] delta Finite difference step size (should be small).
   *  \param[in] relativeError Calculation mode for the error:
   *                        - absolute difference of the analytic and numeric
   *                          gradients if argument relativeError is false
   *                        - relative difference of the analytic and numeric
   *                          gradients with respect to the magnitude of the
   *                          individual element. In this mode, some scaling
   *                          is done to avoid numerical artefacts for tiny
   *                          values or zeros.
   *  \param[in] verbose If true, debug information is printed to the console.
   *  \return True for success, false otherwise.
   */
  virtual bool testJacobian(double errorLimit=1.0e-4, double delta=1.0e-6,
                            bool relativeError=false, bool verbose=false);

  /*! \brief Performs a gradient test for the task Hessian using a finite
   *         difference method. The error is calculated with the method
   *         \ref Rcs_testGradient().
   *
   *  \param[in] verbose If true, debug information is printed to the console.
   *  \return True for success, false otherwise.
   */
  virtual bool testHessian(bool verbose=false);

  /*! \brief Compares the output of the function \ref computeXp_ik with the
   *         product J*qp
   *
   *  \param[in] maxErr Maximum permissable rms error.
   *  \return Root mean square error of difference.
   */
  virtual bool testVelocity(double maxErr=1.0e-8) const;

  /*! \brief Performs a set of tests:
   *         - Jacobian finite difference test
   *         - Hessian finite difference test
   *
   *  \param[in] verbose If true, debug information is printed to the console.
   *  \return True for success, false otherwise.
   */
  virtual bool test(bool verbose=false);

  /*! \brief Writes the task's xml representation to a file desriptor.
   */
  virtual void toXML(FILE* out, bool activation = true) const;

  ///@}


protected:

  /*! \brief Sets the dimension of the task. This should not be required in
   *         most cases. However, in few exceptions (e.g. CompositeTask), it
   *         is needed.
   */
  virtual void setDim(unsigned int taskDim);

  /*! \brief Writes the task's xml representation common to all tasks into
   *         thie file descriptor: name, control variable
   */
  virtual void toXMLStart(FILE* out) const;

  /*! \brief Writes the specific task's xml representation to a file
   *         desriptor. Here it is effector, refBdy, refFrame. To be
   *         overwritten in specialized classes.
   */
  virtual void toXMLBody(FILE* out) const;

  /*! \brief Writes the active flag and the xml closing syntax.
   */
  virtual void toXMLEnd(FILE* out, bool activation) const;

  /*! \brief Graph on which the task operates. It is not a const pointer,
   *         since it is modified in some test functions. However, it is safe
   *         to assume that it is not modified during "normal" processing.
   */
  RcsGraph* graph;          //!< Underlying graph representation
  const RcsBody* ef;        //!< Effector
  const RcsBody* refBody;   //!< Reference body
  const RcsBody* refFrame;  //!< Reference frame body


private:

  /*! \brief Dimension relevant for IK calculation
   */
  unsigned int taskDim;

  /*! \brief Private assignment operator to avoid it being used
   */
  Task& operator = (const Task&);

  /*! \brief Name of the task
   */
  std::string name;

  /*! \brief Class name of the task (Xml tag "controlVariable")
   */
  std::string className;

  /*! \brief List of constant task parameters for each dimension
   */
  std::vector<Parameters> params;
};

}

#endif // RCS_TASK_H
