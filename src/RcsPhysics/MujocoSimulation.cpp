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

#include "MujocoSimulation.h"
#include "PhysicsFactory.h"

#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_dynamics.h>
#include <Rcs_kinematics.h>
#include <Rcs_math.h>
#include <Rcs_body.h>
#include <Rcs_shape.h>
#include <Rcs_joint.h>
#include <Rcs_mujocoParser.h>
#include <Rcs_quaternion.h>

#include <algorithm>
#include <cmath>

/*

TODO:

- Mutex arounf debug window updates (was however not issue yet)

Some findings:

- Mujoco rotation matrices are transposed with respect to Rcs ones (A_BI is A_IB)

*/

namespace Rcs
{

static const char className[] = "Mujoco";
static PhysicsFactoryRegistrar<MujocoSimulation> physics(className);

/*******************************************************************************
 * Constructor.
 ******************************************************************************/
MujocoSimulation::MujocoSimulation() :
  PhysicsBase(), sim(NULL), simData(NULL), debugWindow(NULL)
{
}

/*******************************************************************************
 * Constructor.
 ******************************************************************************/
MujocoSimulation::MujocoSimulation(const RcsGraph* graph_) :
  PhysicsBase(graph_), sim(NULL), simData(NULL), debugWindow(NULL)
{
  bool success = initialize(graph_, NULL);
  RCHECK(success);
}

/*******************************************************************************
 * Copy constructor.
 ******************************************************************************/
MujocoSimulation::MujocoSimulation(const MujocoSimulation& copyFromMe) :
  PhysicsBase(copyFromMe), sim(NULL), simData(NULL), debugWindow(NULL)
{
}

/*******************************************************************************
 * Copy constructor.
 ******************************************************************************/
MujocoSimulation::MujocoSimulation(const MujocoSimulation& copyFromMe,
                                   const RcsGraph* newGraph) :
  PhysicsBase(copyFromMe, newGraph), sim(NULL), simData(NULL), debugWindow(NULL)
{
}

/*******************************************************************************
 * Assignment operator.
 ******************************************************************************/
MujocoSimulation& MujocoSimulation::operator= (const MujocoSimulation& other)
{
  // check for self-assignment by comparing the address of the
  // implicit object and the parameter
  if (this == &other)
  {
    return *this;
  }

  PhysicsBase::operator =(other);
  return *this;
}

/*******************************************************************************
 * Destructor.
 ******************************************************************************/
MujocoSimulation::~MujocoSimulation()
{
  mj_deleteModel(sim);
  mj_deleteData(simData);
  delete debugWindow;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool MujocoSimulation::initialize(const RcsGraph* g, const PhysicsConfig* cfg)
{
  RCHECK_MSG(sizeof(mjtNum) == sizeof(double),
             "Mujoco seems to be compiled for float, but only double is supported");
  initGraph(g);

  bool success = RcsGraph_toMujocoFile("mujoco.xml", g);
  RcsGraph_copy(getGraph(), g);

  if (!success)
  {
    RLOG(1, "Couldn't convert graph to mujoco xml file");
    RFATAL("Couldn't convert graph to mujoco xml file");
    return false;
  }
  else
  {
    RLOG(5, "Mujoco xml file successfully written");
  }

  char error[1024] = "";
  this->sim = mj_loadXML("mujoco.xml", NULL, error, 1024);

  if (!this->sim)
  {
    RLOG(1, "Couldn't create mujoco simulation from mujoco.xml");
    RFATAL("Couldn't create mujoco simulation from mujoco.xml: %s", error);
    return false;
  }
  else
  {
    RLOG(5, "Mujoco simulation successfully created");
  }

  this->simData = mj_makeData(this->sim);

  if (!this->simData)
  {
    RLOG(1, "Couldn't create mujoco data");
    RFATAL("Couldn't create mujoco data");
    return false;
  }
  else
  {
    RLOG(5, "Mujoco data successfully created");
  }

  simData->userdata = (mjtNum*) this;
  // This enables the computation of the kinetic and potential energies
  // that will be returned through the getEnergy() function. According to
  // the documentation, it leads to a bit of run-time overhead.
  sim->opt.enableflags |= mjENBL_ENERGY;
  setJointAngles(g->q);
  mj_forward(sim, simData);
  // install control callback
  mjcb_control = controlCallback;

  A_BI.resize(sim->nbody);
  for (int i = 0; i < sim->nbody; ++i)
  {
    HTr_setIdentity(&A_BI[i]);
  }

  REXEC(1)
  {
    print();
    RLOG(5, "Done construction");
  }

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
void MujocoSimulation::print() const
{
  // Print joints
  RLOG(0, "Joints: nq=%d nv=%d njnt=%d nu=%d", sim->nq, sim->nv, sim->njnt, sim->nu);
  RLOG(0, "Number of joints: %d %d", sim->njnt, getGraph()->dof);
  for (int i = 0; i < sim->njnt; i++)
  {
    fprintf(stdout, "Joint %d: %s %s\n",
            i, sim->names + sim->name_jntadr[i], getGraph()->joints[i].name);
    //RCHECK(STREQ(sim->names + sim->name_jntadr[i], getGraph()->joints[i].name));
  }

  // Print bodies. Mujoco has an additional worldbody.
  RLOG(0, "Number of bodies: %d %d", sim->nbody, getGraph()->nBodies);
  for (int i = 1; i < sim->nbody; i++)
  {
    fprintf(stdout, "Body %d: %s %s\n",
            i, sim->names + sim->name_bodyadr[i], getGraph()->bodies[i-1].name);
    //RCHECK(STREQ(sim->names + sim->name_bodyadr[i], getGraph()->bodies[i-1].name));
  }

  mj_printFormattedModel(sim, "mujoco_formatted.txt", "%f");
  char error[1000] = "";
  mj_saveLastXML("mujoco_parsed.xml", sim, error, 1000);
}

/*******************************************************************************
 *
 ******************************************************************************/
void MujocoSimulation::step(double dt)
{
  simulate(dt);
}

/*******************************************************************************
 *
 ******************************************************************************/
void MujocoSimulation::simulate(double dt, MatNd* q, MatNd* q_dot,
                                MatNd* q_ddot, MatNd* T, bool control)
{
  if (dt <= 0.0)
  {
    return;
  }

  incrementTime(dt);

  mj_step(sim, simData);
  getJointAngles(getGraph()->q);
  getJointVelocities(getGraph()->q_dot);

  if (q)
  {
    MatNd_copy(q, getGraph()->q);
  }

  if (q_dot)
  {
    MatNd_copy(q_dot, getGraph()->q_dot);
  }

  if (q_ddot)
  {
    getJointAccelerations(q_ddot);
  }

  RcsGraph_setState(getGraph(), getGraph()->q, getGraph()->q_dot);

  for (size_t i = 0; i < A_BI.size(); ++i)
  {
    HTr A_MI = getMujocoTransform(i);
    HTr_copy(&A_BI[i], &A_MI);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MujocoSimulation::reset()
{
  RcsGraph_setDefaultState(getGraph());
  mj_resetData(sim, simData);
  setJointAngles(getGraph()->q);
}

/*******************************************************************************
 *
 ******************************************************************************/
const char* MujocoSimulation::getClassName() const
{
  return className;
}

/*******************************************************************************
 *
 ******************************************************************************/
void MujocoSimulation::setGravity(const double newGravity[3])
{
  RCHECK(sim);
  sim->opt.gravity[0] = newGravity[0];
  sim->opt.gravity[1] = newGravity[1];
  sim->opt.gravity[2] = newGravity[2];
}

/*******************************************************************************
 *
 ******************************************************************************/
void MujocoSimulation::setForce(const RcsBody* body, const double F[3],
                                const double p[3])
{
  RFATAL("FIXME");
}

/*******************************************************************************
 *
 ******************************************************************************/
void MujocoSimulation::applyImpulse(const RcsBody* body, const double F[3],
                                    const double r[3])
{
  RFATAL("FIXME");
}

/*******************************************************************************
 *
 ******************************************************************************/
void MujocoSimulation::applyForce(const RcsBody* body, const double f[3],
                                  const double r[3])
{
  RLOG(1, "FIXME");
}

/*******************************************************************************
 *
 ******************************************************************************/
void MujocoSimulation::applyTransform(const RcsBody* body, const HTr* A_BI)
{
  RFATAL("FIXME");
}

/*******************************************************************************
 *
 ******************************************************************************/
void MujocoSimulation::applyLinearVelocity(const RcsBody* body,
                                           const double v[3])
{
  RFATAL("FIXME");
}

/*******************************************************************************
 *
 ******************************************************************************/
void MujocoSimulation::applyAngularVelocity(const RcsBody* body,
                                            const double omega[3])
{
  RFATAL("FIXME");
}

/*******************************************************************************
 *
 ******************************************************************************/
void MujocoSimulation::getLinearVelocity(const RcsBody* body,
                                         double v[3]) const
{
  RFATAL("FIXME");
}

/*******************************************************************************
 *
 ******************************************************************************/
void MujocoSimulation::getAngularVelocity(const RcsBody* body,
                                          double omega[3]) const
{
  RFATAL("FIXME");
}

/*******************************************************************************
 *
 ******************************************************************************/
void MujocoSimulation::setJointTorque(const MatNd* T)
{
  MatNd_copy(this->T_des, T);
}

/*******************************************************************************
 *
 ******************************************************************************/
void MujocoSimulation::getJointTorque(MatNd* T_curr, RcsStateType type) const
{
  RFATAL("FIXME");
}

/*******************************************************************************
 *
 ******************************************************************************/
void MujocoSimulation::getJointAngles(MatNd* q, RcsStateType type) const
{
  int i_rcs = 0, i_mcj = 0;

  for (int jnt=0; jnt<sim->njnt; ++jnt)
  {
    switch (sim->jnt_type[jnt])
    {
      case mjJNT_FREE:    // 7 dof position and quaternion
      {
        const double* pos = &simData->qpos[i_mcj];
        const double* quat = &simData->qpos[i_mcj+3];
        Vec3d_copy(&q->ele[i_rcs], pos);
        Quat_toEulerAngles(&q->ele[i_rcs+3], quat);
        i_rcs += 6;
        i_mcj += 7;
        break;
      }

      case mjJNT_BALL:    // 4 dof quaternion
      {
        const double* quat = &simData->qpos[i_mcj];
        Quat_toEulerAngles(&q->ele[i_rcs], quat);
        i_rcs += 3;
        i_mcj += 4;
        break;
      }

      case mjJNT_SLIDE:   // 1 dof
      case mjJNT_HINGE:   // 1 dof
      {
        MatNd_set(q, i_rcs, 0, simData->qpos[i_mcj]);
        i_rcs += 1;
        i_mcj += 1;
        break;
      }

      default:
        RFATAL("Joint type %d at joint index %d of %d unknown",
               sim->jnt_type[jnt], i_mcj, sim->njnt);
    }

  }

  // MatNd qPos_ = MatNd_fromPtr(sim->nq, 1, simData->qpos);
  // MatNd_printCommentDigits("qpos", &qPos_, 6);
  // qPos_ = MatNd_fromPtr(sim->nq, 1, sim->qpos0);
  // MatNd_printCommentDigits("qpos0", &qPos_, 6);

  RCHECK(i_rcs == (int)getGraph()->dof);
  RCHECK(i_mcj == sim->nq);
}

/*******************************************************************************
 *
 ******************************************************************************/
void MujocoSimulation::setJointAngles(MatNd* q)
{
  int i_rcs = 0, i_mcj = 0;

  for (int jnt=0; jnt<sim->njnt; ++jnt)
  {
    switch (sim->jnt_type[jnt])
    {
      case mjJNT_FREE:    // 7 dof position and quaternion
      {
        double* pos = &simData->qpos[i_mcj];
        double* quat = &simData->qpos[i_mcj+3];
        Vec3d_copy(pos, &q->ele[i_rcs]);
        Quat_fromEulerAngles(quat, &q->ele[i_rcs+3]);
        i_rcs += 6;
        i_mcj += 7;
        break;
      }

      case mjJNT_BALL:    // 4 dof quaternion
      {
        double* quat = &simData->qpos[i_mcj];
        Quat_fromEulerAngles(quat, &q->ele[i_rcs+3]);
        i_rcs += 3;
        i_mcj += 4;
        break;
      }

      case mjJNT_SLIDE:   // 1 dof
      case mjJNT_HINGE:   // 1 dof
      {
        simData->qpos[i_mcj] = q->ele[i_rcs];
        i_rcs += 1;
        i_mcj += 1;
        break;
      }

      default:
        RFATAL("Joint type %d at joint index %d of %d unknown",
               sim->jnt_type[jnt], i_mcj, sim->njnt);
    }

  }

  RCHECK_MSG(i_mcj == sim->nq, "%d %d", i_mcj, sim->nq);
  RCHECK_MSG(i_rcs == (int)getGraph()->dof, "%d %d", i_rcs, getGraph()->dof);
}

/*******************************************************************************
 *
 ******************************************************************************/
void MujocoSimulation::getJointVelocities(MatNd* q_dot,
                                          RcsStateType type) const
{
  int i = 0;
  for (int jnt = 0; jnt < sim->njnt; ++jnt)
  {
    switch (sim->jnt_type[jnt])
    {
      case mjJNT_FREE:    // 6 dof position and omega
      {
        VecNd_copy(&q_dot->ele[i], &simData->qvel[i], 6);
        i += 6;
        break;
      }

      case mjJNT_BALL:    // 3 dof omega
      {
        VecNd_copy(&q_dot->ele[i], &simData->qvel[i], 3);
        i += 3;
        break;
      }

      case mjJNT_SLIDE:   // 1 dof
      case mjJNT_HINGE:   // 1 dof
      {
        MatNd_set(q_dot, i, 0, simData->qvel[i]);
        i += 1;
        break;
      }

      default:
        RFATAL("Joint type %d at joint index %d of %d unknown",
               sim->jnt_type[jnt], i, sim->njnt);
    }

  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MujocoSimulation::getJointAccelerations(MatNd* q_ddot,
                                             RcsStateType type) const
{
  int i = 0;
  for (int jnt = 0; jnt < sim->njnt; ++jnt)
  {
    switch (sim->jnt_type[jnt])
    {
      case mjJNT_FREE:    // 6 dof position and omega_dot
      {
        VecNd_copy(&q_ddot->ele[i], &simData->qacc[i], 6);
        i += 6;
        break;
      }

      case mjJNT_BALL:    // 3 dof omega_dot
      {
        VecNd_copy(&q_ddot->ele[i], &simData->qacc[i], 3);
        i += 3;
        break;
      }

      case mjJNT_SLIDE:   // 1 dof
      case mjJNT_HINGE:   // 1 dof
      {
        MatNd_set(q_ddot, i, 0, simData->qacc[i]);
        i += 1;
        break;
      }

      default:
        RFATAL("Joint type %d at joint index %d of %d unknown",
               sim->jnt_type[jnt], i, sim->njnt);
    }

  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void MujocoSimulation::setMassAndInertiaFromPhysics(RcsGraph* graph)
{
  // Nothing to do here
}

/*******************************************************************************
 *
 ******************************************************************************/
void MujocoSimulation::getPhysicsTransform(HTr* A_bdyI,
                                           const RcsBody* body) const
{
  int id = mj_name2id(sim, mjOBJ_BODY, body->name);

  if (id != -1)
  {
    HTr_copy(A_bdyI, &A_BI[id]);
  }
  else
  {
    RLOG(1, "Transform for body% s not found", body->name);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
HTr MujocoSimulation::getMujocoTransform(int mj_id) const
{
  RCHECK(mj_id != -1);

  HTr A_MI;
  Mat3d_transpose(A_MI.rot, (double(*)[3]) & simData->xmat[mj_id * 9]);
  Vec3d_copy(A_MI.org, &simData->xpos[mj_id * 3]);

  return A_MI;
}

/*******************************************************************************
 *
 ******************************************************************************/
const HTr* MujocoSimulation::getPhysicsTransformPtr(const RcsBody* body) const
{
  int id = mj_name2id(sim, mjOBJ_BODY, body->name);

  if ((id==-1) || (!STREQ(sim->names+sim->name_bodyadr[id], body->name)))
  {
    return &body->A_BI;
  }

  return &A_BI[id];
}

/*******************************************************************************
 *
 ******************************************************************************/
void MujocoSimulation::disableCollision(const RcsBody* b0, const RcsBody* b1)
{
  // Only predefined pairs
  sim->opt.collision = mjCOL_PAIR;
}

/*******************************************************************************
 *
 ******************************************************************************/
PhysicsBase::Contacts MujocoSimulation::getContacts()
{
  Contacts contacts;

  return contacts;
}

/*******************************************************************************
 *
 ******************************************************************************/
void MujocoSimulation::setJointLimits(bool enable)
{
  RFATAL("FIXME");
}

/*******************************************************************************
 *
 ******************************************************************************/
MujocoSimulation* MujocoSimulation::clone(RcsGraph* newGraph) const
{
  return new MujocoSimulation(*this, newGraph);
}

/*******************************************************************************
 *
 ******************************************************************************/
void MujocoSimulation::setJointCompliance(const MatNd* stiffness,
                                          const MatNd* damping)
{
  RFATAL("FIXME");
}

/*******************************************************************************
 *
 ******************************************************************************/
void MujocoSimulation::getJointCompliance(MatNd* stiffness,
                                          MatNd* damping) const
{
  RFATAL("FIXME");
}

/*******************************************************************************
 *
 ******************************************************************************/
bool MujocoSimulation::addBody(const RcsGraph* graph, const RcsBody* body_)
{
  RFATAL("FIXME");
  return false;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool MujocoSimulation::setParameter(ParameterCategory category,
                                    const char* name, const char* type,
                                    double value)
{
  switch (category)
  {
    case Simulation:
      if (STREQ(type, "Integrator"))
      {
        RLOG_CPP(5, "Changed integrator to " << std::string(name));
      }
      break;

    default:
      break;
  }

  return true;
}

/*******************************************************************************
 * 0: potential, 1: kinetic energy
 ******************************************************************************/
double MujocoSimulation::getEnergy() const
{
  return simData->energy[0] + simData->energy[1];
}

/*******************************************************************************
 *
 ******************************************************************************/
std::string MujocoSimulation::getIntegrator() const
{
  return std::string();
}

/*******************************************************************************
 *
 ******************************************************************************/
void MujocoSimulation::setControlInput(const MatNd* q_des_,
                                       const MatNd* q_dot_des_,
                                       const MatNd* T_des_)
{
  PhysicsBase::setControlInput(q_des_, q_dot_des_, T_des_);
}

/*******************************************************************************
 *
 ******************************************************************************/
MujocoDebugWindow* MujocoSimulation::createDebugWindow()
{
  if (debugWindow)
  {
    RLOG(4, "Debug window already running");
    return debugWindow;
  }

  debugWindow = new MujocoDebugWindow(sim, simData);
  debugWindow->start();

  return debugWindow;
}

/*******************************************************************************
 *
 ******************************************************************************/
MujocoDebugWindow* MujocoSimulation::getDebugWindow()
{
  return debugWindow;
}

/*******************************************************************************
 *
 ******************************************************************************/
void MujocoSimulation::destroyDebugWindow()
{
  if (!debugWindow)
  {
    RLOG(4, "Debug window already destroyed");
    return;
  }

  delete debugWindow;
  debugWindow = NULL;
}

/*******************************************************************************
 *
 ******************************************************************************/
void MujocoSimulation::toggleDebugWindow()
{
  if (!debugWindow)
  {
    createDebugWindow();
  }
  else
  {
    destroyDebugWindow();
  }

}

/*******************************************************************************
 * simple controller applying damping to each dof
 ******************************************************************************/
void MujocoSimulation::controlCallback(const mjModel* sim, mjData* simData)
{
  MujocoSimulation* self = (MujocoSimulation*)simData->userdata;

  for (int i = 0; i < sim->nu; i++)
  {
    const char* jntName = sim->names + sim->name_actuatoradr[i];
    const RcsJoint* jnt = RcsGraph_getJointByName(self->getGraph(), jntName);
    if (jnt)
    {
      simData->ctrl[i] = 0.0*self->q_des->ele[jnt->jointIndex];
      RLOG(1, "Actuator %d: %s %.2f", i, jntName, simData->ctrl[i] * 180.0 / M_PI);
    }
  }



  //if (m->nu == m->nv)
  //  mju_scl(d->ctrl, d->qvel, -0.1, m->nv);
}

}   // namespace Rcs
