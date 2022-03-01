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

#ifndef RCS_VIAPOINTSEQUENCE_H
#define RCS_VIAPOINTSEQUENCE_H

#include "Rcs_MatNd.h"

#include <vector>

/*
 * Here's the format of the MatNd to describe the via point array:
 *
 *   time   values           flag
 *   t    x xp xpp      pos | vel | acc
 *
 * It means that the first row corresponds to the initial condition,
 * and the last row to the final condition.
 *
 * This is what the flags mean:
 * 0 nothing
 * 1 pos
 * 2 vel
 * 3 pos + vel
 * 4 acc
 * 5 pos + acc
 * 6 vel + acc
 * 7 pos + vel + acc
 */
#define VIA_POS (0)
#define VIA_VEL (1)
#define VIA_ACC (2)



namespace Rcs
{
class ViaPointSequence
{
  friend class ViaPointSequencePlotter;

public:

  enum ViaPointType
  {
    FifthOrderPolynomial = 0,
    LinearAcceleration
  };

  /*! \brief Constructor without initialization. All member pointers are
   *         initialized to NULL, and no heap memory is allocated. It is safe
   *         to call the destructor for an instance that has been constructed
   *         with this constructor.
   */
  ViaPointSequence();

  /*! \brief Constructor with descriptor. The polynomial parameters will will
   *         be computed in the constructor.
   */
  ViaPointSequence(const MatNd* viaDescr,
                   ViaPointType type=FifthOrderPolynomial);

  /*! \brief Convenience constructor with string version of descriptor. If no
   *         descriptor can be destructed from it, the behavior is undefined.
   */
  ViaPointSequence(const char* viaString,
                   ViaPointType type=FifthOrderPolynomial);

  /*! \brief Copy constructor. It does only member copying, no additional
   *         initialization.
   */
  ViaPointSequence(const ViaPointSequence& copyFromMe);

  /*! \brief Assignment operator. It does only member copying, no additional
   *         initialization.
   */
  ViaPointSequence& operator=(const ViaPointSequence& rhs);

  /*! \brief Destroys the class instance and frees all internal memory.
   */
  virtual ~ViaPointSequence();

  /*! \brief Clones the ViaPointSequence.
   *
   *  \return Deep copy of a ViaPointSequence
   */
  ViaPointSequence* clone() const;

  /*! \brief Computes the polynomial parameters of the descriptor. This is an
   *         expensive operation. It will go through each row of the descriptor
   *         and create a row for the corresponsing linear equation system that
   *         describes the relation of constraints to polynomial parameters.
   *         Rows with flag zero will be ignored. The equation system is then
   *         solved.
   *
   *  \param[in] viaDescr   Via point descriptor to be analysed
   *  \return True for success, false otherwise.
   */
  virtual bool init(const MatNd* viaDescr);

  /*! \brief Adds a via point to the sequence. It is just added to the classes
   *         descriptor. To recompute the polynomial, the \ref init() function
   *         needs to be called.
   *
   *  \param[in] t        Insertion time point. May be outside the current
   *                      time interval, however then the flag must be 7
   *  \param[in] x        Via point position, considered only of the bit
   *                      VIA_POS in flag is set
   *  \param[in] x_dot    Via point velocity, considered only of the bit
   *                      VIA_VEL in flag is set
   *  \param[in] x_ddot   Via point acceleration, considered only of the bit
   *                      VIA_ACC in flag is set
   *  \param[in] flag     Bit mask, see general class description for details.
   *
   *  \return True for success, false otherwise. False will only be returned
   *          if a via point with a flag other than 7 is added before t0 or
   *          after t1. In this case, the function does not modify the
   *          descriptor.
   */
  virtual bool addViaPoint(double t, double x, double x_dot, double x_ddot,
                           int flag);

  /*! \brief Prints out all kind of helpful internal information about the
   *         via point sequence.
   */
  void print() const;

  /*! \brief Checks if the via sequence is valid or not. Several checks are
   *         performed:
   *         - Member variables are not NULL
   *         - Dimension checks of all member arrays
   *         - Time points in sequence are increasing
   *         - Matrix B is invertible and B*inv(B) = I
   *         - All given constraints are matched
   *
   *  \return True for success, false otherwise. On debug level 1 or higher,
   *          the reason of a possible failure is logged to stderr.
   */
  bool check() const;

  /*! \brief Plots the via sequence between initial and final time to Gnuplot.
   *  \param[in] dt Time tic for data points to be displayed. For instance if
   *                the duration os 1 sec, and dt is 0.01, 100 points will be
   *                displayed.
   *
   *  \param[in] flag Indicator which level of derivative to show:
   *                  - 1: position
   *                  - 2: velocity
   *                  - 4: acceleration
   *                  - 7: all in the same plot window
   */
  void gnuplot(double dt=0.01, int flag=7) const;

  /*! \brief Plots the via sequence between t0 and t1 to Gnuplot. This function
   *         is otherwise identical to \ref gnuplot.
   */
  void gnuplot(double t0, double t1, double dt=0.01, int flag=7) const;

  /*! \brief Time of the initial constraint
   *
   *  \return Initial via point time
   */
  double t0() const;

  /*! \brief Time of the final constraint
   *
   *  \return Last via point time
   */
  double t1() const;

  /*! \brief Duration of the via point sequence
   *
   *  \return Time between initial and final via point
   */
  double duration() const;

  /*! \brief Computes a trajectory for a given ViaPointSequence.
   *
   *  \param[out] traj Trajectory. Must have memory for 4*round((t1-t0)/dt)
   *                   double values. The function will reshape this array to
   *                   4 rows and round((t1-t0)/dt) columns.
   *  \param[in] t0    Start time in seconds
   *  \param[in] t1    End time in seconds
   *  \param[in] dt    Time between two samples
   */
  void computeTrajectory(MatNd* traj, double t0, double t1,
                         double dt=0.01) const;

  /*! \brief Convenience version of the \ref computeTrajectory function. The
   *         start and end time is taken from the class instance
   *
   *  \param[out] traj Trajectory. Must have memory for 4*round((t1-t0)/dt)
   *                   double values. The function will reshape this array to
   *                   4 rows and round((t1-t0)/dt) columns.
   *  \param[in] dt    Time between two samples
   */
  void computeTrajectory(MatNd* traj, double dt=0.01) const;

  /*! \brief Calculates the position, velocity and acceleration at time t
   *
   *  \param[out] xt      Position at time t
   *  \param[out] xt_dot  Velocity at time t
   *  \param[out] xt_ddot Acceleration at time t
   *  \param[in] t        Time to evaluate the position, velocity and
   *                      acceleration.
   */
  void computeTrajectoryPoint(double& xt, double& xt_dot, double& xt_ddot,
                              double t) const;
  void computeTrajectoryPoint_poly5(double& xt, double& xt_dot, double& xt_ddot,
                                    double t) const;
  void computeTrajectoryPoint_linAcc(double& xt, double& xt_dot, double& xt_ddot,
                                     double t) const;

  /*! \brief Calculates the position at time t
   *
   *  \param[in] t Time to evaluate the position
   *  \return Position at time point t
   */
  double computeTrajectoryPos(double t) const;

  /*! \brief Calculates the velocity at time t
   *
   *  \param[in] t Time to evaluate the velocity
   *  \return Velocity at time point t
   */
  double computeTrajectoryVel(double t) const;

  /*! \brief Calculates the acceleration at time t
   *
   *  \param[in] t Time to evaluate the acceleration
   *  \return Acceleration at time point t
   */
  double computeTrajectoryAcc(double t) const;

  /*! \brief Calculates the jerk at time t
   *
   *  \param[in] t Time to evaluate the jerk
   *  \return Jerk at time point t
   */
  double computeTrajectoryJerk(double t) const;

  /*! \brief Not yet implemented.
   */
  double getMaxVelocity(double& t_vmax) const;

  /*! \brief Constructs a randomized via sequence descriptor and tests it
   *         with the \ref check() method. On debug level 1 or higher, the
   *         sequence is plotted with Gnuplot.
   *
   *  \return True for success, false otherwise. This function should always
   *          return success.
   */
  static bool test();

  /*! \brief Computes the number of constraints that are present in the via
   *         point descriptr. This corresponds to the dimension of the square
   *         matrix B.
   *
   *  \param[in] desc   Via point descriptor to be analysed
   *  \return Number of constraints in the descriptor
   */
  static size_t computeNumberOfConstraints(const MatNd* desc);

  /*! \brief Enables the turbo mode: Computes the polynomial parameters up to
   *         the first full constraint in the descriptor. Any point after the
   *         first constraint queried will return an undefined result. This
   *         function is essentially setting the computeHorizon to max.
   *
   *  \param[in] enable   True for activating turbo more, false otherwise.
   */
  void setTurboMode(bool enable);

  /*! \brief Sets the time horizon until which the polynomial coefficients are
   *         computed. Calling this function with t_hor=0 will lead to the
   *         polynomials only computed up to the first full constraint. This is
   *         the most computationall efficient calculation.
   *
   *  \param[in] t_hor Time horizon up to which the polynomials are computed.
   */
  void setComputeHorizon(double t_hor);

  /*! \brief See setTurboMode().
   *
   *  \return True for turbo mode being enabled, false otherwise.
   */
  bool getTurboMode() const;

  /*! \brief See setComputeHorizon(double t_hor).
   *
   *  \return Returns the time point up to which the polynomial coefficients
   *          deliver a valid query.
   */
  bool getComputeHorizon() const;

  /*! \brief This class supports different types of polynomial coefficient
   *         calculations. The default is FifthOrderPolynomial and will compute
   *         coefficients according to a minimum jerk model. The
   *         LinearAcceleration mode will lead to linear acceleration profiles.
   *         It is experimental only and has some issues.
   *
   *  \param[in] type   Polynomial computation type, see enum ViaPointType.
   */
  void setViaPointType(ViaPointType type);

  /*! \brief See setViaPointType(ViaPointType type).
   *
   *  \return Returns the current mode of polynomial coefficient calculation.
   */
  ViaPointType getViaPointType() const;

  MatNd* cloneDescriptor() const;

  bool gradientDxDvia(MatNd* dxdvia, unsigned int row, double t0, double dt, unsigned int nSteps) const;
  bool gradientDxDvia(MatNd* dxdvia, unsigned int row, double t0, double t1, double dt) const;
  bool gradientDxDvia_a(MatNd* dxdvia, unsigned int row, double t0, double t1, double dt) const;

protected:

  /*! \brief Assembles the right hand side vector for the given time point to
   *         solve for the polynomial coefficients.
   *
   *  \param[in] rhs   Right hand side vector of time polynomials.
   *  \param[in] t     Time point at which rhs is evaluated
   */
  void computeRHS(MatNd* rhs, double t) const;

  /*! \brief Sorts the rows of desc so that the time (column 0) always
   *         increases.
   *
   *  \param[in] desc   Via point descriptor to be sorted
   */
  void sort(MatNd* desc) const;

  /*! \brief Gets the element of the vector p at the given index. If index is
   *         out of range, the function will exit with a fatal error.
   *
   *  \param[in] index Index of the desired element
   *  \return Element of the vector p at the given index
   */
  double getPolynomialParameter(size_t index) const;

  /*! \brief Finds the first flag 7 constraint after t_horizon and reshapes
   *         the descriptor so that it is the last one. This function assumes a
   *         descriptor that is sorted with respect to time.
   *
   *  \param[in] desc        Via point descriptor to be compressed
   *  \param[in] t_horizon   Time point until which all flag 7 constraints
   *                         are kept.
   */
  void compressDescriptor(MatNd* desc, double t_horizon) const;

  static int getConstraintIndex(const MatNd* desc, unsigned int row,
                                unsigned int pos_vel_or_acc);

  /*! \brief Computes the B-matrix for the linear equation system x = B*p
   *         where x are the trajectory constraints and p are the polynomial
   *         parameters. The function will not terminate in case of algorithmic
   *         issues, but return success or failure. It will only crash when
   *         severe memory allocation problems occur.
   *
   *  \param[out] B       Matrix describing x = B*p
   *  \param[in]  vDesc   Via point descriptor
   *  \return True for success, false otherwise.
   */
  static bool computeB_poly5(MatNd* B, const MatNd* vDesc);
  static void computeB_linAcc(MatNd* B, const MatNd* vDesc);

  MatNd* viaDescr;
  MatNd* B, *invB, *x, *p;
  double computeHorizon;
  ViaPointType viaType;

  // Vectors with dimension equal to the number of rows / columns of B
  std::vector<int> constraintType;
  std::vector<double> viaTime;
};

class ViaPointSequencePlotter
{
public:
  ViaPointSequencePlotter();
  virtual ~ViaPointSequencePlotter();
  virtual void plot(const ViaPointSequence& via, double t0, double t1,
                    double dt=0.01, int flag=7);
  virtual void plot2(const ViaPointSequence& via, double t0, double t1,
                     double dt=0.01, int flag=7);
  virtual void enableFixedAxes(const ViaPointSequence& via, bool enable,
                               double margin=0.25);
  virtual void setRangeY(double lowerLimit, double upperLimit);
  virtual void setRangeX(double lowerLimit, double upperLimit);

protected:
  FILE* pipe;
  bool fixAxes;
  double lowerLimitY[3];
  double upperLimitY[3];
  double lowerLimitX;
  double upperLimitX;
};

}

#endif   // RCS_VIAPOINTSEQUENCE_H
