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
#include "PeriodicCallback.h"

#include <Rcs_utils.h>
#include <Rcs_timer.h>
#include <Rcs_macros.h>

#include <cfloat>          // DBL_MAX
#include <cmath>           // std::sqrt
#include <algorithm>       // std::min

#if !defined (_MSC_VER)
#include <sys/time.h>      // getrlimit
#include <sys/resource.h>  // getrlimit
#include <execinfo.h>      // backtrace
#include <sys/mman.h>      // mlockall
#include <errno.h>         // errno
#include <csignal>         // SIGXCPU
#endif



/*******************************************************************************
 * Posix thread wrapper function around fast callback
 ******************************************************************************/
void* Rcs::PeriodicCallback::threadFuncPosix(void* param)
{
  Rcs::PeriodicCallback* self = static_cast<Rcs::PeriodicCallback*>(param) ;
  self->preStart();

  double dtFast = 1.0/self->getUpdateFrequency();
  double dtUsec = 1.0e6*dtFast;

  RLOG(5, "Starting periodic callback with %.2f msec", 1.0e3*dtFast);
  Timer* timerFast = Timer_create(dtFast);




  while (self->isRunning() == true)
  {
    // get current time
    pthread_mutex_lock(&self->settingsMtx);
    self->now = Timer_getSystemTime();
    pthread_mutex_unlock(&self->settingsMtx);

    // call the fast callback function
    self->callback();

    // Lock when updating timing information
    pthread_mutex_lock(&self->settingsMtx);

    // get duration of mainLoopStep (in usec)
    self->duration = (Timer_getSystemTime() - self->now) * 1000000.0;

    // runtime statistics:
    self->durationMin = std::min(self->duration, self->durationMin);
    self->durationMax = std::max(self->duration, self->durationMax);
    self->durationSum += self->duration;
    self->durationSumSqr += self->duration*self->duration;

    if ((self->duration > dtUsec) &&
        (self->syncMode != PeriodicCallback::Synchronize_Sleep))
    {
      // Warn for debug levels equal or higher than 3
      RLOG(3, "overrun at loop %lu in %s: duration=%.1f usec "
           "(should be < %.1f)\n", self->loopCount,
           self->className.c_str(), self->duration, dtUsec);

      self->overruns++;
    }

    self->loopCount++;

    // Done computing timing statistics
    pthread_mutex_unlock(&self->settingsMtx);



    // wait until it's our turn again
    switch (self->getSynchronizationMode())
    {
      case Rcs::PeriodicCallback::Synchronize_Hard:
        Timer_wait(timerFast);
        break;

      case Rcs::PeriodicCallback::Synchronize_Soft:
        Timer_waitNoCatchUp(timerFast);
        break;

      case Rcs::PeriodicCallback::Synchronize_Sleep:
        Timer_waitDT(dtFast);
        break;

      case Rcs::PeriodicCallback::Synchronize_OneShot:
        pthread_mutex_lock(&self->settingsMtx);
        self->runLoop = false;
        pthread_mutex_unlock(&self->settingsMtx);
        break;

      case Rcs::PeriodicCallback::Synchronize_Poll:
        break;

      default:
        RFATAL("Unknown synchronization mode: %d",
               self->getSynchronizationMode());
    }

  }   // while(runLoop)


  REXEC(1)
  {
    printf("[%s(%d)]: exiting main loop\n", __FUNCTION__, __LINE__);
    self->print();
  }

  Timer_destroy(timerFast);

  pthread_mutex_lock(&self->settingsMtx);
  self->started = false;
  pthread_mutex_unlock(&self->settingsMtx);

  // Exit thread function to allow joining pthread
  pthread_exit(NULL);

  return NULL;
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::PeriodicCallback::PeriodicCallback() :
  runLoop(false),
  started(false),
  updateFreq(0.0),
  now(0.0),
  duration(0.0),
  durationMin(DBL_MAX),
  durationMax(0.0),
  durationSum(0.0),
  durationSumSqr(0.0),
  loopCount(0),
  overruns(0),
  schedulingPolicy(SCHED_OTHER),
  threadPriority(50),
  className("Rcs::PeriodicCallback"),
  syncMode(Synchronize_Soft)
{
  pthread_mutex_init(&this->settingsMtx, NULL);
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::PeriodicCallback::~PeriodicCallback()
{
  if (isStarted()==true)
  {
    // not good
    RLOG(0, "The PeriodicCallback  of class %s is getting destroyed while its"
         " thread is still running! This will very likely lead to problems,"
         " since the derivedclass is already destroyed. Please call stop() in"
         " your destructor.", this->className.c_str());

    // let's be on the safe side and stop anyways
    stop();
  }

  pthread_mutex_destroy(&this->settingsMtx);

  RLOGS(5, "PeriodicCallback of class \"%s\" destructor finished",
        this->className.c_str());
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::PeriodicCallback::setSynchronizationMode(SynchronizeMode mode)
{
  pthread_mutex_lock(&this->settingsMtx);
  this->syncMode = mode;
  pthread_mutex_unlock(&this->settingsMtx);
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::PeriodicCallback::SynchronizeMode
Rcs::PeriodicCallback::getSynchronizationMode() const
{
  SynchronizeMode mode;

  pthread_mutex_lock(&this->settingsMtx);
  mode = this->syncMode;
  pthread_mutex_unlock(&this->settingsMtx);

  return mode;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::PeriodicCallback::start()
{
  start(getUpdateFrequency(), getThreadPriority());
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::PeriodicCallback::start(double updateFreq_, int prio)
{
  if (isStarted() == true)
  {
    RLOG(1, "PeriodicCallback \"%s\" has already been started - skipping",
         getClassName().c_str());
    return;
  }

  if (updateFreq_ <= 0.0)
  {
    RLOG(1, "PeriodicCallback \"%s\" update frequency <= 0 (%g) - skipping",
         getClassName().c_str(), updateFreq_);
    return;
  }

  pthread_mutex_lock(&this->settingsMtx);
  this->started = true;
  this->runLoop = true;
  this->updateFreq = updateFreq_;

  pthread_attr_t* att = NULL;
  pthread_attr_t attrRT;

  this->threadPriority = prio;
  bool rtSuccess = Rcs_getRealTimeThreadAttribute(this->schedulingPolicy,
                                                  prio, &attrRT);

  if (rtSuccess == true)
  {
    att = &attrRT;
    RLOG(5, "Succeeded to get real-time thread attributes");
  }
  else
  {
    RLOG(1, "Failed to get real-time thread attributes");
  }

  pthread_mutex_unlock(&this->settingsMtx);


  int res = pthread_create(&this->callbackThread, att, &threadFuncPosix, this);

  switch (res)
  {
    case 0:
      RLOG(5, "Success to get real-time thread attributes");
      break;

    case EAGAIN:
      RLOG(1, "EAGAIN Insufficient resources to create another thread");
      break;

    case EINVAL:
      RLOG(1, "EINVAL Invalid settings in attr");
      break;

    case EPERM:
      RLOG(1, "EPERM  No permission to set the scheduling policy and "
           "parameters");
      break;

    default:
      RLOG(1, "Unknown error %d", res);
      break;
  }

  // This removes a memory issues in our memory debugger, but leads to crashes.
  //pthread_attr_destroy(&attrRT);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::PeriodicCallback::stop()
{
  if (isStarted()==false)
  {
    RLOG(4, "Thread is already stopped");
    return;
  }

  pthread_mutex_lock(&this->settingsMtx);
  this->runLoop = false;
  pthread_mutex_unlock(&this->settingsMtx);

  while (isStarted()==true)
  {
    Timer_waitDT(0.01);
  }

  pthread_join(this->callbackThread, NULL);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::PeriodicCallback::print() const
{
  pthread_mutex_lock(&this->settingsMtx);
  double dtUsec = 1.0e6/this->updateFreq;
  double mean = (double) this->durationSum / this->loopCount;
  double stdev = std::sqrt(((double) this->durationSumSqr / this->loopCount)-mean*mean);
  double durMin = this->durationMin;
  double durMax = this->durationMax;
  pthread_mutex_unlock(&this->settingsMtx);

  printf("%s callback loop statistics:\n", getClassName().c_str());
  printf("  target period = %.3f msec\n", 0.001*dtUsec);
  printf("  min           = %.3f msec\n", 0.001*durMin);
  printf("  ave           = %.3f msec\n", 0.001*mean);
  printf("  max           = %.3f msec\n", 0.001*durMax);
  printf("  stdev         = %.3f msec\n", 0.001*stdev);
  printf("  total cycles  = %lu\n", getLoopCount());
  printf("  overruns      = %lu\n", getOverruns());
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::PeriodicCallback::isRunning() const
{
  bool running;

  pthread_mutex_lock(&this->settingsMtx);
  running = this->runLoop;
  pthread_mutex_unlock(&this->settingsMtx);

  return running;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::PeriodicCallback::isStarted() const
{
  bool threadStarted;

  pthread_mutex_lock(&this->settingsMtx);
  threadStarted = this->started;
  pthread_mutex_unlock(&this->settingsMtx);

  return threadStarted;
}

/*******************************************************************************
 * Sets the class name
 ******************************************************************************/
void Rcs::PeriodicCallback::setClassName(const std::string& name)
{
  pthread_mutex_lock(&this->settingsMtx);
  this->className = name;
  pthread_mutex_unlock(&this->settingsMtx);
}

/*******************************************************************************
 * Sets the class name
 ******************************************************************************/
std::string Rcs::PeriodicCallback::getClassName() const
{
  pthread_mutex_lock(&this->settingsMtx);
  std::string name = this->className;
  pthread_mutex_unlock(&this->settingsMtx);

  return name;
}

/*******************************************************************************
 * Called before entering the callback() loop from the thread's context.
 ******************************************************************************/
void Rcs::PeriodicCallback::preStart()
{
}

/*******************************************************************************
 *
 ******************************************************************************/
double Rcs::PeriodicCallback::getUpdateFrequency() const
{
  double freq;

  pthread_mutex_lock(&this->settingsMtx);
  freq = this->updateFreq;
  pthread_mutex_unlock(&this->settingsMtx);

  return freq;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::PeriodicCallback::setUpdateFrequency(double updateFrequency)
{
  pthread_mutex_lock(&this->settingsMtx);
  this->updateFreq = updateFrequency;
  pthread_mutex_unlock(&this->settingsMtx);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::PeriodicCallback::setSchedulingPolicy(int policy)
{
  pthread_mutex_lock(&this->settingsMtx);
  this->schedulingPolicy = policy;
  pthread_mutex_unlock(&this->settingsMtx);
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::PeriodicCallback::setThreadPriority(int prio)
{
  pthread_mutex_lock(&this->settingsMtx);
  this->threadPriority = prio;
  pthread_mutex_unlock(&this->settingsMtx);

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
int Rcs::PeriodicCallback::getThreadPriority() const
{
  int prio;

  pthread_mutex_lock(&this->settingsMtx);
  prio = this->threadPriority;
  pthread_mutex_unlock(&this->settingsMtx);

  return prio;
}

/*******************************************************************************
 *
 ******************************************************************************/
unsigned long Rcs::PeriodicCallback::getLoopCount() const
{
  unsigned long lc;

  pthread_mutex_lock(&this->settingsMtx);
  lc = this->loopCount;
  pthread_mutex_unlock(&this->settingsMtx);

  return lc;
}

/*******************************************************************************
 *
 ******************************************************************************/
unsigned long Rcs::PeriodicCallback::getOverruns() const
{
  unsigned long orns;

  pthread_mutex_lock(&this->settingsMtx);
  orns = this->overruns;
  pthread_mutex_unlock(&this->settingsMtx);

  return orns;
}

/*******************************************************************************
 *
 ******************************************************************************/
double Rcs::PeriodicCallback::getLastCallbackTime() const
{
  double timeOfLastCallback;

  pthread_mutex_lock(&this->settingsMtx);
  timeOfLastCallback = this->now;
  pthread_mutex_unlock(&this->settingsMtx);

  return timeOfLastCallback;
}

/*******************************************************************************
 *
 ******************************************************************************/
double Rcs::PeriodicCallback::getLastCallbackDuration() const
{
  double tmp;

  pthread_mutex_lock(&this->settingsMtx);
  tmp = this->duration;
  pthread_mutex_unlock(&this->settingsMtx);

  return tmp;
}
