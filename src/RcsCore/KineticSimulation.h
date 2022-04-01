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

#ifndef RCS_KINETICSIMULATION_H
#define RCS_KINETICSIMULATION_H

#include "PhysicsBase.h"

#include <string>



namespace Rcs
{

/*! \ingroup RcsPhysics
 *  \brief Newton-Euler dynamic simulation using rigid body equations of motion
 *
 *  This library implements a dynamics simulation class that integrates the
 *  rigid body equations of motion. It also implements a point contact model
 *  based on a spring-damper model. The forward dynamics is based on an O(3)
 *  scheme that inverts the mass matrix.
 *
 *  The equations of motion can be integrated with two different integrators.
 *  The default is an Euler one-step integration. It is very efficient, but is
 *  sensitive to stiff contacts or large mass ratios. In such cases, the
 *  integration time step should be selected very small.
 *
 *  The second integrator is a Runge-Kutta-Fehlberg integrator with adaptive
 *  step size adaptation. It performs an error checking by comparing the
 *  integration results of a second- and a third-order integration, and adapts
 *  the simulation time step such as to not exceed a given error bounds. This
 *  makes it very robust, but less computationally efficient due to computing
 *  the EoM several times. In systems with stiff contacts, one might feel a
 *  slow-down in certain situations.
 *
 *  Another feature of this class is the decomposition of the EoM into free and
 *  constrained partitions. This allows to model systems with kinematically
 *  moving joints that still react dynamically to ground reaction forces.
 *  Another example is a system with mixed kinematic and dynamic joints, such
 *  as a humanoid robot with position-controlled legs and free-swinging
 *  (or torque-controlled) arms. The decomposition is performed automatically
 *  based on each joints's ctrlType variable: if this is position or velocity,
 *  the dof is in the constrained partition. If it is torque, it is in the free
 *  partition. Algorithmically, we decompose the standard EoM
 *
 *      M qdd - h = F
 *
 *  into
 *
 *      / M00   M10 \  / qdd_f \     / h_f \      / F_f \
 *      |           |  |       |  =  |     |  +   |     |
 *      \ M10   M11 /  \ qdd_c /     \ h_c /      \ F_c /
 *
 *  We then solve for the free accelerations under consideration of the
 *  constrained ones:
 *
 *      qdd_f = inv(M00) (h_f + F_f - M10 qdd_c)
 *
 *  qdd_c contains desired accelerations for joints that are position- or
 *  velocity-controlled, qdd_f contains the ones that are torque controlled
 *  and are responding to gravity and forces. If a body has the property
 *  rigid_body_joints set to true, the class will internally set all joints to
 *  torque-controlled.
 *
 */
class KineticSimulation : public PhysicsBase
{
public:

  /*!
   * @name CreationAndDestruction
   *
   * Creation and destruction
   */

  ///@{

  /*! \brief Empty default constructor. All members are initialized to NULL.
   *         The initialize() function needs to be called to properly
   *         initialize the class.
   */
  KineticSimulation();

  /*! \brief Base class constructor.
   *
   *  \param[in] graph   Graph that the simulation should be build on.
   */
  KineticSimulation(const RcsGraph* graph);

  /*! \brief Copy constructor
   *
   *  \param[in] copyFromMe   Simulation instance to be copied from.
   */
  KineticSimulation(const KineticSimulation& copyFromMe);

  /*! \brief Copy constructor. The configuration state vector will not be set
   *         from the graph of copyFromMe, but will be the one of newGraph.
   *         If they must be equal, this needs to be ensured by the caller. The
   *         graph will never be modified within the physics simulation.
   *
   *  \param[in] copyFromMe   Simulation instance to be copied from.
   *  \param[in] newGraph     RcsGraph to refer to.
   */
  KineticSimulation(const KineticSimulation& copyFromMe,
                    const RcsGraph* newGraph);

  /*! \brief Assignment operator for deep copying everything.
   */
  KineticSimulation& operator = (const KineticSimulation&);

  /*! \brief Virtual destructor to allow overloading.
   */
  virtual ~KineticSimulation();


  ///@}



  /*!
   * @name PureVirtualMethods
   *
   * Pure virtual methods inherited from parent class.
   */

  ///@{

  virtual void step(double dt);

  virtual void simulate(double dt, MatNd* q=NULL, MatNd* q_dot=NULL,
                        MatNd* q_ddot=NULL, MatNd* T=NULL,
                        bool control=false);

  virtual void reset();

  virtual const char* getClassName() const;

  virtual void setGravity(const double gravity[3]);

  virtual void setForce(const RcsBody* body, const double F[3],
                        const double p[3]);

  virtual void applyImpulse(const RcsBody* body, const double F[3],
                            const double r[3]);

  virtual void applyForce(const RcsBody* body, const double F[3],
                          const double r[3]);

  virtual void applyTransform(const RcsBody* body, const HTr* A_BI);

  virtual void applyLinearVelocity(const RcsBody* body, const double v[3]);

  virtual void applyAngularVelocity(const RcsBody* body,
                                    const double omega[3]);

  virtual void getLinearVelocity(const RcsBody* body, double v[3]) const;

  virtual void getAngularVelocity(const RcsBody* body,
                                  double omega[3]) const;

  virtual void setJointTorque(const MatNd* T_des);

  virtual void getJointTorque(MatNd* T_curr,
                              RcsStateType type=RcsStateFull) const;

  virtual void getJointAngles(MatNd* q,
                              RcsStateType type=RcsStateFull) const;

  virtual void getJointVelocities(MatNd* q_dot,
                                  RcsStateType type=RcsStateFull) const;

  virtual void setMassAndInertiaFromPhysics(RcsGraph* graph);

  virtual void getPhysicsTransform(HTr* A_BI, const RcsBody* body) const;

  virtual const HTr* getPhysicsTransformPtr(const RcsBody* body) const;

  virtual void disableCollision(const RcsBody* b0, const RcsBody* b1);

  virtual Contacts getContacts();

  virtual void setJointLimits(bool enable);

  virtual KineticSimulation* clone(RcsGraph* newGraph=NULL) const;

  virtual void setJointCompliance(const MatNd* stiffness,
                                  const MatNd* damping=NULL);

  virtual void getJointCompliance(MatNd* stiffness,
                                  MatNd* damping=NULL) const;

  virtual bool initialize(const RcsGraph* g, const PhysicsConfig* config);

  virtual bool addBody(const RcsGraph* graph, const RcsBody* body);

  ///@}



  /*!
   * @name OverwrittenMethods
   *
   * Methods overwriting parent class methods.
   */

  ///@{

  virtual void setControlInput(const MatNd* q_des, const MatNd* q_dot_des,
                               const MatNd* T_des);

  /*! \brief Function for setting physics parameter. The function is thread-
   *         safe. All strings are interpreted case insensitive. The following
   *         parmeter types are supported:
   *
   *         - category "Simulation":
   *           - type: Integrator
   *           - name: Fehlberg or Euler
   *           - value: Doesn't matter (use 0.0 for instance)
   *
   *  \param[in] category   See enum ParameterCategory
   *  \param[in] name   Parameter name, such as "SoftMaterial".
   *  \param[in] type   Parameter type, such as "Restitution".
   *  \param[in] value  Value the parameter should be assigned with
   *  \return true for success, false otherwise. There is no failure case yet.
   */
  virtual bool setParameter(ParameterCategory category,
                            const char* name, const char* type, double value);


  ///@}



  /*!
   * @name ClassSpecificMethods
   *
   * Methods for this class only.
   */

  ///@{

  /*! \brief Returns the overall (kinetic and potential) energy of the system.
   *         If the system has no dissipating terms (e.g. contacts with damping
   *         or joint velocity damping), the total energy should be constant.
   */
  virtual double getEnergy() const;

  /*! \brief Returns the intetrator currently used in the simulate() method.
   *         Currently, Euler and Fehlberg are supported.
   */
  virtual std::string getIntegrator() const;

  /*! \brief Returns the adapted integration time step. It is only adapted when
   *         the Fehlberg integrator is used. Otherwise, it will correspond to
   *         the dt passed to the simulate() method.
   */
  virtual double getAdaptedDt() const;

  /*! \brief Returns a pointer to the i-th contact point position.
   */
  virtual const double* getContactPositionPtr(size_t i) const;

  /*! \brief Returns the number of contact points.
   */
  virtual size_t getNumContacts() const;

  ///@}



private:

  static double integrationStep(const double* x, void* param, double* xp,
                                double dt);
  static double integrationStepSimple(const double* x, void* param, double* xp,
                                      double dt);
  double dirdyn(const RcsGraph* graph, const MatNd* F_ext,
                MatNd* q_ddot, MatNd* b) const;

  void addContactForces(MatNd* M_contact, MatNd* xp_contact, MatNd* f_contact,
                        const double* x) const;

  void addSpringForces(MatNd* M_spring, const MatNd* q_dot) const;

  void addConstraintForces(MatNd* M_constraint, const MatNd* M, const MatNd* b) const;



  struct FrictionContactPoint
  {
    FrictionContactPoint(int bdyId, int shapeIdx,
                         double mu=1.0, double k_p=2.0e4, double z0=0.0);

    void computeContactForce(double f[3],
                             const double x_contact[3],
                             const double x_attach[3],
                             const double xp_attach[3]) const;

    void computeContacts(double dx_contact[3],
                         double f_contact[3],
                         const double x_contact[3],
                         const double x_attach[3],
                         const double xp_attach[3]) const;

    int bdyId;                // Contact body
    int shapeIdx;             // Index of bodie's contacting shape
    double mu;                // Coulomb friction coefficient
    double k_p;               // Spring coefficient
    double k_v;               // Damping coefficient
    double z0;                // Height of ground plane
  };



  struct KinematicConstraint
  {

    typedef enum
    {
      Pos,
      Ori,
      PosAndOri

    } ConstraintType;

    KinematicConstraint(ConstraintType type, int bdyId, int refBdyId,
                        std::vector<double> x_des, double kp);
    void appendJacobian(MatNd* J, const RcsGraph* graph) const;
    void appendDotJacobian(MatNd* J_dot, const RcsGraph* graph,
                           const MatNd* q_dot) const;
    void appendStabilization(MatNd* ax, const RcsGraph* graph) const;
    static void computeX(const RcsGraph* graph, int bdy_id, int refbdy_id,
                         double x_curr[3]);
    static void computeEulerAngles(const RcsGraph* graph,
                                   int bdy_id,
                                   int refbdy_id,
                                   double ea_curr[3]);

    void computeXp(const RcsGraph* graph, double x_dot[3]) const;
    std::string name(const RcsGraph* graph) const;

    size_t dim() const;
    void print(const RcsGraph* graph) const;

    ConstraintType type;
    int bdyId;
    int refBdyId;
    std::vector<double> x_des;
    double kp;
  };


  std::vector<FrictionContactPoint> contact;
  std::vector<KinematicConstraint> constraint;

  MatNd* draggerTorque;
  MatNd* jointTorque;
  MatNd* contactForces;
  MatNd* contactPositions;
  std::string integrator;
  double energy;
  double dt_opt;
  double lastDt;   // Most recent dt when calling simulate()
  double gravity[3];
};

}   // namespace Rcs

#endif // RCS_KINETICSIMULATION_H
