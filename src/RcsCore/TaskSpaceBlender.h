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

#ifndef RCS_TASKSPACEBLENDER_H
#define RCS_TASKSPACEBLENDER_H

#include <ControllerBase.h>



namespace Rcs
{

class TaskSpaceBlender
{
public:

  typedef enum
  {
    Binary = 0,
    Linear,
    Approximate,
    IndependentTasks,
    DependentTasks

  } BlendingMode;



  /*! \brief Constructs a TaskSpaceBlender class instance based on the
  *         given controller. All tasks from the controller's task list can
  *         be used. The state of the system will be taken from the
  *         controller's graph. This class will not change any internals of
  *         the controller.
  */
  TaskSpaceBlender(const ControllerBase* controller);

  /*! \brief Deletes the class instance and frees all memory. The pased
   *         controller is not considered to be owned, so it will not be
   *         deleted.
   */
  virtual ~TaskSpaceBlender();

  /*! \brief Computes the task blending matrix Wx. We compute it as a
   *         vector, since the matrix is diagonal. The vector only
   *         comprises the rows that have a non-zero activation. Wx will
   *         be reshaped by this function.
   *
   *  \param[out]  Wx      Weight vector of dimension nx x 1
   *  \param[out]  ax_curr True activation vector of dimension nx x 1. If it
   *                       is NULL, it is ignored.
   *  \param[in]   a_des   Full activation vector (including inactive
   *                       dimensions) of dimension nTasks x 1.
   *  \param[in]   J       Task Jacobian comprisiong only active task elements.
   *  \param[in]   invWq   Configuration space regularization for
   *                       pseudo-inverse. It is of dimension 1 x 1 or nq x 1,
   *                       where nq is the number of columns of the Jacobian.
   */
  void compute(MatNd* Wx,
               MatNd* ax_curr,
               const MatNd* a_des,
               const MatNd* J,
               const MatNd* invWq) const;

  /*! \brief Same as
   *         \ref TaskSpaceBlender::compute(MatNd* Wx, MatNd* ax_curr,
   *              const MatNd* a_des, const MatNd* J, const MatNd* invWq) const
   *
   *         Argument lambda is a scalar and will be applied to all task
   *         elements.
   */
  void compute(MatNd* Wx,
               MatNd* ax_curr,
               const MatNd* a_des,
               const MatNd* J,
               double lambda) const;

  /*! \brief Sets the mode how the blending matrix is computed (see enum
   *         BlendingMode). Here is what the modes mean:
   *         - Binary: Weight matrix is identity matrix, activation vector
   *           toggles task between active (a>0) and inactive (a=0)
   *         - Approximate: Heuristic that computes very fast, but has
   *           non-linear behavior
   *         - IndependentTasks: Iterative and accurate method that iteratively
   *           computes the solution. The number of iterations can be set with
   *           the \ref setMaxIterations() method. A good number is 10. This
   *           mode assumes that all tasks are independent.
   *         - DependentTasks: Same as IndependentTasks, but without the
   *           assumption that all tasks are independent. It therefore takes
   *           much longer. This method has not yet been extensively tested.
   *
   *  \param[in]  newMode   Blending mode, see enum BlendingMode
   */
  void setBlendingMode(BlendingMode newMode);

  /*! \brief Returns the current blending mode.
   *
   *  \return Blending mode, see enum BlendingMode
   */
  BlendingMode getBlendingMode() const;

  /*! \brief Sets the maximum number of iterations for the blending modes
   *         IndependentTasks and DependentTasks. The default is 10. A larger
   *         number improves the convergence behavior, at the cost of a longer
   *         calculation time. In case the class is set to incremental weight
   *         update, the maxIterations value can be selected very low.
   *
   *  \param[in] maxIterations   Max. number of iterations.
   */
  void setMaxIterations(unsigned int maxIterations);

  /*! \brief Returns the maximum number of iterations for the blending modes
   *         IndependentTasks and DependentTasks.
   *
   *  \return Max. number of iterations
   */
  unsigned int getMaxIterations() const;

  /*! \brief Enables updating the passed Wx matrix incrementally. This often
   *         leads to a faster convergence if the configuration of the system
   *         or the desired activations did not change much. In case the
   *         number of active elements changed, this option is ignored and the
   *         weight matrix diagonal is initialized with the desired
   *         activations. This function only has an effect for the modes
   *         IndependentTasks and DependentTasks.
   *
   *  \param[in]  enable   True for incremental weigh updates, false otherwise.
   */
  void setIncrementalWeightUpdate(bool enable);

  /*! \brief Returns if the weight matrix is incrementally updated or not.
   *
   *  \return True for incremental update, false otherwise.
   */
  bool getIncrementalWeightUpdate() const;

protected:

  void computeActivation(MatNd* ax_curr,
                         const MatNd* Wx,
                         const MatNd* a_des,
                         const MatNd* J,
                         const MatNd* invWq) const;

  double getActivationScaling(MatNd* scaling,
                              const MatNd* J,
                              const MatNd* a_des,
                              const double tol) const;

  /*! \brief Computes the task blending matrix Wx. We compute it as a
   *         vector, since the matrix is diagonal. The vector only
   *         comprises the rows that have a non-zero activation. Wx will
   *         be reshaped by this function.
   *
   *  \param[out]  Wx      Weight vector of dimension nx x 1
   *  \param[out]  ax_curr True activation vector of dimension nx x 1. If it
   *                       is NULL, it is ignored.
   *  \param[in]   a_des   Full activation vector (including inactive
   *                       dimensions) of dimension nTasks x 1.
   *  \param[in]   J       Task Jacobian comprisiong only active task elements.
   *  \param[in]   invWq   Configuration space regularization for
   *                       pseudo-inverse. It is of dimension 1 x 1 or nq x 1,
   *                       where nq is the number of columns of the Jacobian.
   *  \param[in]   rankActivation Experimental and expensive method to
   *               calculate the maximally possible activations
   */
  void computeIterative(MatNd* Wx,
                        MatNd* ax_curr,
                        const MatNd* a_des,
                        const MatNd* J,
                        const MatNd* invWq,
                        bool rankActivation=false) const;

  void computeApproximate(MatNd* Wx,
                          MatNd* ax_curr,
                          const MatNd* a_des,
                          const MatNd* J,
                          const MatNd* invWq,
                          const bool useInnerProduct=true) const;

  void computeBinary(MatNd* Wx,
                     MatNd* ax_curr,
                     const MatNd* a_des) const;

  void computeLinear(MatNd* Wx,
                     MatNd* ax_curr,
                     const MatNd* a_des) const;

private:

  const ControllerBase* controller;

  BlendingMode mode;
  unsigned int maxIterations;
  bool incrementalWeightUpdate;
};

}   // namespace Rcs

#endif   // RCS_TASKSPACEBLENDER_H
