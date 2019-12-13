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

#if defined(_MSC_VER)
#ifndef RCS_MSVC_H
#define RCS_MSVC_H

#pragma warning(disable: 4267)   //conversion from ??? to ???, possible loss of data

#include <stdio.h>    // _snprintf
#include <stdlib.h>
#include <string.h>   // _stricmp
#include <math.h>     // _finite, _isnan
#include <malloc.h>   // alloca
#include <cfloat>
#include <limits>

#if defined(WIN_DLL)
#    ifdef RCSCORE_EXPORTS
#        define RCSCORE_API __declspec(dllexport)
#    else
#        define RCSCORE_API __declspec(dllimport)
#    endif
#endif

// http://stackoverflow.com/questions/8487986/file-macro-shows-full-path :
// According to the donator of this snipplet (I modified it to search from
// back to front), this expression is evaluated at compile time. The limit
// of the file name length is 26 characters. If there's no separator, the
// original __FILE__ macro is used. Printing out the expression can be done
// using the below three lines:
// #define STRING2(x) #x
// #define STRING(x) STRING2(x)
// #pragma message(STRING(__FILENAME__))

#define STRIPPATH(p)                                                     \
  (                                                                      \
      sizeof (p) > 2 && (p)[sizeof(p)-3] == '\\' ? (p) + sizeof(p) - 2 :    \
      sizeof (p) > 3 && (p)[sizeof(p)-4] == '\\' ? (p) + sizeof(p) - 3 :    \
      sizeof (p) > 4 && (p)[sizeof(p)-5] == '\\' ? (p) + sizeof(p) - 4 :    \
      sizeof (p) > 5 && (p)[sizeof(p)-6] == '\\' ? (p) + sizeof(p) - 5 :    \
      sizeof (p) > 6 && (p)[sizeof(p)-7] == '\\' ? (p) + sizeof(p) - 6 :    \
      sizeof (p) > 7 && (p)[sizeof(p)-8] == '\\' ? (p) + sizeof(p) - 7 :    \
      sizeof (p) > 8 && (p)[sizeof(p)-9] == '\\' ? (p) + sizeof(p) - 8 :    \
      sizeof (p) > 9 && (p)[sizeof(p)-10] == '\\' ? (p) + sizeof(p) - 9 :   \
      sizeof (p) > 10 && (p)[sizeof(p)-11] == '\\' ? (p) + sizeof(p) - 10 : \
      sizeof (p) > 11 && (p)[sizeof(p)-12] == '\\' ? (p) + sizeof(p) - 11 : \
      sizeof (p) > 12 && (p)[sizeof(p)-13] == '\\' ? (p) + sizeof(p) - 12 : \
      sizeof (p) > 13 && (p)[sizeof(p)-14] == '\\' ? (p) + sizeof(p) - 13 : \
      sizeof (p) > 14 && (p)[sizeof(p)-15] == '\\' ? (p) + sizeof(p) - 14 : \
      sizeof (p) > 15 && (p)[sizeof(p)-16] == '\\' ? (p) + sizeof(p) - 15 : \
      sizeof (p) > 16 && (p)[sizeof(p)-17] == '\\' ? (p) + sizeof(p) - 16 : \
      sizeof (p) > 17 && (p)[sizeof(p)-18] == '\\' ? (p) + sizeof(p) - 17 : \
      sizeof (p) > 18 && (p)[sizeof(p)-19] == '\\' ? (p) + sizeof(p) - 18 : \
      sizeof (p) > 19 && (p)[sizeof(p)-20] == '\\' ? (p) + sizeof(p) - 19 : \
      sizeof (p) > 20 && (p)[sizeof(p)-21] == '\\' ? (p) + sizeof(p) - 20 : \
      sizeof (p) > 21 && (p)[sizeof(p)-22] == '\\' ? (p) + sizeof(p) - 21 : \
      sizeof (p) > 22 && (p)[sizeof(p)-23] == '\\' ? (p) + sizeof(p) - 22 : \
      sizeof (p) > 23 && (p)[sizeof(p)-24] == '\\' ? (p) + sizeof(p) - 23 : \
      sizeof (p) > 24 && (p)[sizeof(p)-25] == '\\' ? (p) + sizeof(p) - 24 : \
      sizeof (p) > 25 && (p)[sizeof(p)-26] == '\\' ? (p) + sizeof(p) - 25 : \
      sizeof (p) > 26 && (p)[sizeof(p)-27] == '\\' ? (p) + sizeof(p) - 26 : p)

#define __FILENAME__ STRIPPATH(__FILE__)

#define strcasecmp _stricmp
#define strncasecmp _strnicmp
#define snprintf   _snprintf
#define copysign   _copysign
#define popen      _popen
#define pclose     _pclose

#if (_MSC_VER == 1900)
#define isfinite(x)   _finite((double)(x))
#define isnan(x)      _isnan((double)(x))
#define trunc(x)    (((x) < 0) ? ceil((x)) : floor((x)))
#else
#define _TIMESPEC_DEFINED
#endif

static inline void srand48(unsigned int seed)
{
  srand(seed);
}

static inline double drand48()
{
  return (double) rand() / RAND_MAX;
}

#if (_MSC_VER < 1900)
static inline int round(double x)
{
  int i = (int) x;

  if (x >= 0.0)
  {
    return ((x-i) >= 0.5) ? (i + 1) : (i);
  }
  else
  {
    return (-x+i >= 0.5) ? (i - 1) : (i);
  }
}
#endif

static inline double fmin(double x, double y)
{
  return (x<y?x:y);
}

static inline double fmax(double x, double y)
{
  return (x>y?x:y);
}

static inline double cbrt(double x)
{
  if (fabs(x) < DBL_EPSILON)
  {
    return 0.0;
  }

  if (x > 0.0)
  {
    return pow(x, 1.0/3.0);
  }

  return -pow(-x, 1.0/3.0);
}

#endif   // RCS_MSVC_H
#endif   // RCS_MSC_VER
