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
 *  based on a spring-damper model.
*/
class KineticSimulation : public PhysicsBase
{
public:

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

  virtual void setControlInput(const MatNd* q_des, const MatNd* q_dot_des,
                               const MatNd* T_des);

  virtual bool setParameter(ParameterCategory category,
                            const char* name, const char* type, double value);

  // protected:

  struct FrictionContactPoint
  {
    FrictionContactPoint(const RcsBody* bdy, int shapeIdx,
                         double mu=1.0, double k_p=2.0e4, double z0=0.0);

    void computeContactForce(double f[3],
                             const double x_contact[3],
                             const double x_attach[3],
                             const double xp_attach[3]) const;

    void computeContactSpeed(double xp_contact[3],
                             const double dt,
                             const double x_contact[3],
                             const double x_attach[3],
                             const double xp_attach[3]) const;

    int bdyId;                // Contact body
    int shapeIdx;             // Index of bodie's contacting shape
    double mu;                // Coulomb friction coefficient
    double k_p;               // Spring coefficient
    double k_v;               // Damping coefficient
    double z0;                // Height of ground plane
    double x_contact[3];      // Contact point position in world coordinates
    double f_contact[3];      // Contact force in world coordinates
  };

  static void integrationStep(const double* x, void* param, double* xp,
                              double dt);

  MatNd* draggerTorque;
  std::vector<FrictionContactPoint> contact;
  std::string integrator;
  double energy;
  double dt_opt;
};

}   // namespace Rcs

#endif // RCS_KINETICSIMULATION_H
