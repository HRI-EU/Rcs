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

#ifndef RCS_TIMER_H
#define RCS_TIMER_H


#include <stdbool.h>


#ifdef __cplusplus
extern "C" {
#endif



/*!
 * \defgroup RcsTimerFunctions Timer functions
 *
 */



typedef struct _Timer Timer;

/*! \ingroup RcsTimerFunctions
 *  \brief Creates a timer instance with count down time dt (in [secs]).
 */
Timer* Timer_create(double dt);

/*! \ingroup RcsTimerFunctions
 *  \brief Resets the timer to the current time.
 */
void Timer_reset(Timer* self);

/*! \ingroup RcsTimerFunctions
 *  \brief Returns the time in [secs] that has passed since the timer
 *         has been resetted.
 */
double Timer_get(Timer* self);

/*! \ingroup RcsTimerFunctions
 *  \brief Waits until the count down has passed.
 */
bool Timer_wait(Timer* self);

/*! \ingroup RcsTimerFunctions
*  \brief Waits until the count down has passed. - no catch up
*/
void Timer_waitNoCatchUp(Timer* self);

/*! \ingroup RcsTimerFunctions
 *  \brief Returns the number of Timer_wait() - calls since the timer
 *         instantiation or the last call to Timer_reset().
 */
long Timer_getIteration(const Timer* self);

/*! \ingroup RcsTimerFunctions
 *   \brief Returns the count down time.
 */
double Timer_getSamplingTime(const Timer* self);

/*! \ingroup RcsTimerFunctions
 *  \brief Deletes the timer and frees all associated memory.
 */
void Timer_destroy(Timer* self);

/*! \ingroup RcsTimerFunctions
 *  \brief Returns the computer time in secs. This considers an offset, so
 *         that the timer can be resetted (with Timer_setZero()).
 */
double Timer_getTime();

/*! \ingroup RcsTimerFunctions
 *  \brief Returns the computer time in secs.
 */
double Timer_getSystemTime();

/*! \ingroup RcsTimerFunctions
 *  \brief Waits for a period of dt seconds. The minimum resolution is
 *         1 msec. If dt is equal or less 0, the function does nothing.
 */
void Timer_waitDT(double dt);

/*! \ingroup RcsTimerFunctions
 *  \brief Sets the timer's increment to dt [in seconds].
 */
void Timer_setDT(Timer* self, double dt);

/*! \ingroup RcsTimerFunctions
 *  \brief Makes the timer produce console output on missed timings if
 *         verbose is true. If false, no output will be generated.
 *         Per default, the timer is silent.
 */
void Timer_setVerbose(Timer* self, bool verbose);

/*! \ingroup RcsTimerFunctions
 *  \brief Please document.
 */
double Timer_getCurrentIterationTime(const Timer* self);

/*! \ingroup RcsTimerFunctions
 *  \brief Calls usleep. It's a convenience wrapper function, since for
 *         MSVC, there's no usleep function.
 */
void Timer_usleep(unsigned long usec);

/*! \ingroup RcsTimerFunctions
 *  \brief Resets the time for the Timer_getTime() calls.
 */
void Timer_setZero();

/*! \ingroup RcsTimerFunctions
*  \brief Sets the timer to the given tim.
*/
void Timer_setTo(Timer* self, double t_des);

#ifdef __cplusplus
}
#endif

#endif   // RCS_TIMER_H
