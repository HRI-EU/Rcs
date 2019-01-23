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

#ifndef RCS_PERIODICCALLBACK_H
#define RCS_PERIODICCALLBACK_H

#include <pthread.h>
#include <string>


namespace Rcs
{

/*! \brief PeriodicCallback Class
 *
 *         This is a virtual class. Deriving and implementing a callback()
 *         function allows to have the callback function being called
 *         periodically from a thread / real-time task.
 */
class PeriodicCallback
{
public:

  /*! \brief Synchronization modes:
   *         - Synchronize_Hard: Tries to keep the time slices in sync with
   *           the computer time. In real-time mode, this is ensured by a
   *           real-time thread. In user-space mode, the \ref Timer_wait
   *           is used.
   *         - Synchronize_Soft: Makes use of the \ref Timer_waitNoCatchUp
   *           function. If one time slice takes dt longer as desired, the
   *           next callback is postponed by dt. This makes the
   *           synchronization stick to the desired duration of the time
   *           slices, but introduces some drift in case there is
   *           violations.
   *         - Synchronize_Sleep: Just sleeps for dt after calling the
   *           callback function.
   */
  typedef enum
  {
    Synchronize_Hard = 0,
    Synchronize_Soft,
    Synchronize_Sleep,
    Synchronize_OneShot,
    Synchronize_Poll

  } SynchronizeMode;

  /*! \brief Constructor. Thread-settings need to be set before calling the
   *         start() method, otherwise they don't take effect.
   *
   */
  PeriodicCallback();

  /*! \brief This destructor doesn't stop the thread. Please make sure that
   *         you do this in the class you instantiate, either by calling the
   *         \ref stop() method directly, or in the destructor.
   */
  virtual ~PeriodicCallback();

  /*! \brief This function needs to be implemented by the derieving classes.
   *         It is called periodically after the \ref start() method has been
   *         called. The update frequency needs to be specified in the \ref
   *         start() method.
   */
  virtual void callback() = 0;

  /*! \brief This function is called before entering the periodic callback
   *         loop in the thread function. In this implementation, it does
   *         nothing. It can be overwritten e.g. to perform operations that
   *         need to be carried out in the thread context.
   */
  virtual void preStart();

  /*! \brief Sets the synchronization mode. Default is \ref Synchronize_Soft
   *         for user-space mode, and \ref Synchronize_Hard for real-time
   *         mode. This function is thread-safe.
   */
  virtual void setSynchronizationMode(SynchronizeMode mode);

  /*! \brief Sets the scheduling policy:
   *         - SCHED_OTHER
   *         - SCHED_FIFO
   *         - SCHED_RR
   *         See pthread_attr_setschedpolicy for details.
   *         This function is thread-safe.
   */
  virtual void setSchedulingPolicy(int policy);

  /*! \brief Sets the thread priority. It will only take effect before starting
   *         the thread.
   *
   *  \param[in] priority   New thread priority
   *  \return True for success, false otherwise.
   */
  bool setThreadPriority(int priority);

  /*! \brief Sets the scheduling policy:
   *         - SCHED_OTHER
   *         - SCHED_FIFO
   *         - SCHED_RR
   *         See pthread_attr_setschedpolicy for details.
   *         This function is thread-safe.
   */
  int getThreadPriority() const;

  /*! \brief Returns the current synchronization mode.
   *         This function is thread-safe.
   */
  virtual SynchronizeMode getSynchronizationMode() const;

  /*! \brief Starts a thread that calls the \ref callback() function with
   *         the previously set update frequency. If the scheduling policy is
   *         set to SCHED_RR or SCHED_FIFO, then the thread priority will take
   *         effect.
   */
  virtual void start();

  /*! \brief Starts a thread that calls the \ref callback() function with
   *         the given frequency. If the scheduling policy is set to
   *         SCHED_RR or SCHED_FIFO, then the thread priority will take
   *         effect.
   *
   *  \param[in] updateFreq   Thread update frequency
   *  \param[in] prio         Thread priority
   */
  virtual void start(double updateFreq, int prio=50);

  /*! \brief Stops the thread. This joins the thread and should be called
   *         before calling the destructor. When stop() returns, the thread
   *         has successfully been joined and it is guaranteed that no
   *         thread computation is executed afterwards.
   *
   *         Note: Calling this function in the constructor will lead to
   *               runtime errors due to calling a pure virtual function.
   *               This is due to the destruction of the instance, and thus
   *               having no valid \ref callback() function in the thread
   *               loop.
   */
  virtual void stop();

  /*! \brief Prints out some internal information of the class.
   *         This function is thread-safe.
   */
  virtual void print() const;

  /*! \brief Returns true if the thread is running, false otherwise.
   *         This function is thread-safe.
   */
  virtual bool isRunning() const;

  /*! \brief Returns true if the thread has been started, false otherwise.
   *         This function is thread-safe.
   */
  virtual bool isStarted() const;

  /*! \brief Sets a class name to the instance. It will be used for debugging
   *         output, so that for classes deriving from this one, the user
   *         can make a distinction of where something is going wrong. This is
   *         for instance useful if many threads run in parallel.
   *         This function is thread-safe.
   */
  virtual void setClassName(const std::string& name);

  /*! \brief Returns the class name. For this one, it is "PeriodicCallback"
   *         This function is thread-safe.
   */
  virtual std::string getClassName() const;

  /*! \brief Returns the update frequency. This is 0 on construction, and will
   *         be set in each \ref start() call. This function is thread-safe.
   */
  virtual double getUpdateFrequency() const;

  /*! \brief Sets the update frequency. This will be overwritten if start() is
   *         called with a different frequency. This function is thread-safe.
   */
  virtual void setUpdateFrequency(double updateFrequency);

  /*! \brief Returns the loop count. This variable is incremented in each call
   *         to callback(). This function is thread-safe.
   */
  virtual unsigned long getLoopCount() const;

  /*! \brief Returns the number of timer overruns. This variable is incremented
   *         each time the callback() takes longer than its desired update time.
    *        This function is thread-safe.
   */
  virtual unsigned long getOverruns() const;



private:

  bool runLoop;             ///< Quits the thread's while() loop if false
  bool started;             ///< True if the endless loop has been started
  double updateFreq;        ///< Callback update frequency in [Hz]
  double now;               ///< last starting time of fast callback
  double duration;          ///< last duration of callback (in usec)
  double durationMin;       ///< minimum duration of callback (in usec)
  double durationMax;       ///< maximum duration fast callback (in usec)
  double durationSum;       ///< Sum of all durations of callback (in usec)
  double durationSumSqr;    ///< Sum of all durations of callback (in usec)
  unsigned long loopCount;  ///< Counts each callback function call
  unsigned long overruns;   ///< Counts each missed timing
  int schedulingPolicy;     ///< Default: SCHED_OTHER
  int threadPriority;       ///< Default: 50
  pthread_t callbackThread; ///< Handle to callback thread
  std::string className;    ///< Default: Rcs::PeriodicCallback
  SynchronizeMode syncMode; ///< See enum description
  mutable pthread_mutex_t settingsMtx;

  static void* threadFuncPosix(void* param);

  PeriodicCallback(const PeriodicCallback&);
  PeriodicCallback& operator=(const PeriodicCallback&);
};

}

#endif // RCS_PERIODICCALLBACK_H
