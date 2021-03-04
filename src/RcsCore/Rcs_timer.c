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

#include "Rcs_timer.h"
#include "Rcs_macros.h"

#if !defined (_MSC_VER)
#include <sys/time.h>
#include <unistd.h>
#else
#include <windows.h>
#include <time.h>
#endif


struct _Timer
{
  double t0, t, dt, t_des;
  struct timeval  tv0;
  long tCount;
  bool verbose;
};

static double RCS_TIMER_T0 = 0.0;


/******************************************************************************

   \brief See header.

******************************************************************************/

Timer* Timer_create(double dt)
{
  if (dt < 0.0)
  {
    RLOG(1, "Timer dt<=0.0 (%f) - failed to create timer", dt);
    return NULL;
  }

  Timer* self = (Timer*) calloc(1, sizeof(Timer));

  if (self == NULL)
  {
    RLOG(1, "Out of memory - failed to create timer");
    return NULL;
  }

  self->t0     = Timer_getTime(); // Initial time in seconds
  self->t      = 0.0;   // Running time in secs
  self->t_des  = dt;    // Desired time in secs
  self->dt     = dt;
  self->tCount = 0;
  self->verbose = false;

  return self;
}



/******************************************************************************

   \brief See header.

******************************************************************************/

void Timer_setVerbose(Timer* self, bool verbose)
{
  if (self == NULL)
  {
    RLOG(1, "Timer is NULL - ignoring verbosity");
    return;
  }

  self->verbose = verbose;
}



/******************************************************************************

   \brief See header.

******************************************************************************/

void Timer_reset(Timer* self)
{
  if (self == NULL)
  {
    RLOG(1, "Timer is NULL - not resetting");
    return;
  }

  self->t0     = Timer_getTime();
  self->t      = 0.0;
  self->t_des  = self->dt;
  self->tCount = 0;
}



/******************************************************************************

   \brief See header.

******************************************************************************/

double Timer_get(Timer* self)
{
  if (self == NULL)
  {
    RLOG(1, "Timer is NULL - returning 0.0");
    return 0.0;
  }

  return Timer_getTime() - self->t0;
}



/******************************************************************************

   \brief See header.

******************************************************************************/

double Timer_getTime()
{
  return Timer_getSystemTime() - RCS_TIMER_T0;
}



/******************************************************************************

   \brief See header.

******************************************************************************/

double Timer_getSystemTime()
{
  double seconds = -1.0;

#if defined (_MSC_VER)
  _int64 tCurr, ldFreq;
  QueryPerformanceCounter((LARGE_INTEGER*) &tCurr);
  QueryPerformanceFrequency((LARGE_INTEGER*) &ldFreq);
  seconds = ((double)(tCurr) / (double)ldFreq);
#else
  struct timeval  tv1;
  gettimeofday(&tv1, NULL);
  seconds = tv1.tv_sec + 1.0e-6*tv1.tv_usec;
#endif

  return seconds;
}



/******************************************************************************

   \brief See header.

******************************************************************************/

void Timer_waitDT(double dt)
{
  if (dt <= 0.0)
  {
    return;
  }

  double t0 = Timer_getSystemTime();

  do
  {
    Timer_usleep(100);
  }
  while (Timer_getSystemTime() - t0 < dt);
}



/******************************************************************************

   \brief See header.

******************************************************************************/

bool Timer_wait(Timer* self)
{
  double dt_wait;

  if (self == NULL)
  {
    RLOG(1, "Timer is NULL - not waiting");
    return false;
  }

  self->t = Timer_getTime() - self->t0;

  // Return if current time is later than desired time
  if (self->t >= self->t_des)
  {
    // Return failure if time exceeds t_des+0.25*dt
    if (self->t > self->t_des + 0.25 * self->dt && (self->verbose))
    {
      RMSG("[%ld] Timer: took %5.1f ms (dt is %5.1f ms)", self->tCount,
           1.0e3 * (self->t - self->t_des + self->dt), 1.0e3 * self->dt);
      self->t_des += self->dt;
      return false;
    }

    self->t_des += self->dt;
    return true;
  }

  // Compute remaining time interval and apply usleep. On some machines,
  // usleep returns before the time has elapsed. Therefore we check the
  // final condition in a while loop.
  do
  {
    self->t = Timer_get(self);
    dt_wait = self->t_des - self->t;
    if (dt_wait>0.0)
    {
      Timer_usleep((unsigned long)(dt_wait * 1.0e6));
    }
  }
  while (self->t < self->t_des);

  // Update next desired time and iteration counter
  self->t_des += self->dt;
  self->tCount++;

  return true;
}



/******************************************************************************

   \brief See header.

******************************************************************************/

void Timer_waitNoCatchUp(Timer* self)
{
  if (self == NULL)
  {
    RLOG(1, "Timer is NULL - not waiting");
    return;
  }

  // Compute remaining time interval and apply usleep. On some machines,
  // usleep returns before the time has elapsed. Therefore we check the
  // final condition in a while loop.
  do
  {
    self->t = Timer_get(self);
    if (self->t<self->t_des)
    {
      Timer_usleep((unsigned long)(1.0e6*(self->t_des - self->t)));
    }
  }
  while (self->t < self->t_des);

  self->t_des  = self->t + self->dt;
  self->tCount++;
}



/******************************************************************************

  \brief See header.

******************************************************************************/

long Timer_getIteration(const Timer* self)
{
  if (self == NULL)
  {
    RLOG(1, "Timer is NULL - returning iteration 0");
    return 0;
  }

  return self->tCount;
}



/******************************************************************************

  \brief See header.

******************************************************************************/

double Timer_getSamplingTime(const Timer* self)
{
  if (self == NULL)
  {
    RLOG(1, "Timer is NULL - returning sampling time 0.0");
    return 0.0;
  }

  return self->dt;
}



/******************************************************************************

  \brief See header.

******************************************************************************/

double Timer_getCurrentIterationTime(const Timer* self)
{
  return self->t0 + self->t_des - self->dt;
}



/******************************************************************************

  \brief See header.

******************************************************************************/

void Timer_destroy(Timer* self)
{
  if (self != NULL)
  {
    free(self);
  }
}



/******************************************************************************

  \brief See header.

******************************************************************************/

void Timer_usleep(unsigned long usec)
{
#if defined (_MSC_VER)
  double dt_msec = 1.0e-3*(double)usec;
  Sleep((DWORD)floor(dt_msec));  // msec resolution
#else
  usleep(usec);
#endif
}



/******************************************************************************

  \brief See header.

******************************************************************************/

void Timer_setZero()
{
  RCS_TIMER_T0 = Timer_getSystemTime();
}



/******************************************************************************

\brief See header.

******************************************************************************/

void Timer_setTo(Timer* self, double t_des)
{
  if (self == NULL)
  {
    RLOG(1, "Timer is NULL - not setting");
    return;
  }

  self->t = self->t0 + t_des;
  self->t_des = self->t + self->dt;
}



