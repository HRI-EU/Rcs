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

#ifndef ATOMIC_H
#define ATOMIC_H

#include <Rcs_macros.h>

#include <pthread.h>

namespace Rcs
{

/*!
* \ingroup RcGui
* \brief This class implements a generic, templated, thread-safe container
*
* Access to the data is only possible via the get() function and the operators,
* which are all secured via a mutex.
*
* \note Usually automatic casting to the template type works well, so this class
*       can be used as a replacement for it. However, for functions that take a
*       variable-length argument list, you need to explicitely cast using
*       (T) or static_cast<T>(). Alternatively you can use the get() function.
*/
template<class T>
class Atomic
{
public:
  /*!
   * \brief Default constructor
   */
  Atomic()
  {
    pthread_mutex_init(&_mutex, NULL);

    pthread_mutex_lock(&_mutex);
    _value = T();
    pthread_mutex_unlock(&_mutex);
  }

  /*!
   * \brief Constructor with given initial value
   * \param val Value to be set initially
   */
  Atomic(const T& val)
  {
    pthread_mutex_init(&_mutex, NULL);

    pthread_mutex_lock(&_mutex);
    _value = val;
    pthread_mutex_unlock(&_mutex);
  }

  /*!
   * \brief Copy constructor
   * \param obj Object to copy
   */
  Atomic(const Atomic& obj)
  {
    pthread_mutex_init(&_mutex, NULL);

    pthread_mutex_lock(&_mutex);
    pthread_mutex_lock(&obj._mutex);

    _value = obj._value;

    pthread_mutex_unlock(&_mutex);
    pthread_mutex_unlock(&obj._mutex);
  }

  /*!
   * \brief Get function in case the automatic casting does not work
   * \return Value
   */
  T get() const
  {
    pthread_mutex_lock(&_mutex);
    T val = _value;
    pthread_mutex_unlock(&_mutex);
    return val;
  }

  /*!
   * \brief Operator for automatic casting to template type
   */
  operator T()
  {
    pthread_mutex_lock(&_mutex);
    T val = _value;
    pthread_mutex_unlock(&_mutex);
    return val;
  }

  /*!
   * \brief Assignment operator given another Atomic
   * \param rhs Value to assign
   * \return Reference to this
   */
  Atomic& operator=(const Atomic& rhs)
  {
    if (this == &rhs)
    {
      RFATAL("Cannot assign Atomic to itself");
      return *this;
    }

    pthread_mutex_lock(&_mutex);
    pthread_mutex_lock(&rhs._mutex);

    _value = rhs._value;

    pthread_mutex_unlock(&_mutex);
    pthread_mutex_unlock(&rhs._mutex);

    return *this;
  }

  /*!
   * \brief Assignment operator given another variable of the template type
   * \param rhs Value to assign
   * \return Reference to this
   */
  Atomic& operator=(const T& rhs)
  {
    pthread_mutex_lock(&_mutex);
    _value = rhs;
    pthread_mutex_unlock(&_mutex);

    return *this;
  }
private:
  T _value;
  mutable pthread_mutex_t _mutex;
};

}

#endif   // ATOMIC_H
