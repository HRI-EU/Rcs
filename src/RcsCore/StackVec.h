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

#ifndef RCS_STACKVEC_H
#define RCS_STACKVEC_H

#include <ostream>


namespace Rcs
{
template <class T, int NVAL>
class StackVec
{
public:

  // Only the memory that is used upon construction is initialized with zero.
  StackVec(int numEle) : ele(ssoStack), nEle(numEle)
  {
    if (nEle > NVAL)
    {
      ele = new T[nEle];
    }

    memset(ele, 0, nEle * sizeof(T));
  }

  StackVec(const StackVec& other) : ele(ssoStack), nEle(other.nEle)
  {

    if (other.ele != other.ssoStack)
    {
      ele = new T[nEle];
    }

    memmove(ele, other.ele, other.size()*sizeof(T));
  }

  StackVec& operator= (const StackVec& other)
  {
    if (this == &other)
    {
      return *this;
    }

    // The class to be assigned has only stack memory
    if (other.ele == other.ssoStack)
    {
      // If we have heap memory, we delete it and point to the stack
      if (ele != ssoStack)
      {
        delete [] ele;
        ele = ssoStack;
      }
      // Then do the copy
      memmove(ele, other.ele, other.size()*sizeof(T));
    }
    // The class to be assigned has heap memory
    else
    {
      // If we have stack memory, we create new heap memory
      if (ele == ssoStack)
      {
        ele = new T[other.size()];
      }
      // If we have heap memory, we make sure it is large enough
      else
      {
        if (nEle < other.size())
        {
          delete [] ele;
          ele = new T[other.size()];
        }
      }

      memmove(ele, other.ele, other.size()*sizeof(T));
    }

    // This goes last, otherwise we can't check against this classes size
    nEle = other.nEle;

    return *this;
  }

  inline size_t size() const
  {
    return nEle;
  }

  inline bool isHeap() const
  {
    return (ele==ssoStack) ? false : true;
  }

  ~StackVec()
  {
    if (ele != ssoStack)
    {
      delete[] ele;
    }
  }

  // We don't need these since we have overloaded the cast operator
  // T& operator [](size_t i) { return ele[i]; }
  // T operator [](size_t i) const { return ele[i]; }

  // We make this class behave as if it was a pointer to T. Therefore we
  // don't necessarily need to overload the brackets operator necessarily.
  operator T* () const
  {
    return ele;
  }

  bool operator==(const StackVec& other)
  {
    if (size() != other.size())
    {
      return false;
    }

    for (size_t i=0; i<size(); ++i)
    {
      if (ele[i] != other.ele[i])
      {
        return false;
      }
    }

    return true;
  }

  bool operator!=(const StackVec& other)
  {
    return ! operator==(other);
  }


private:

  T* ele;
  T ssoStack[NVAL];
  size_t nEle;
};

}

#endif // RCS_STACKVEC_H
