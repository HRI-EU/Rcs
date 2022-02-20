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

//#if defined(WIN_DLL)
#    ifdef RCSCORE_EXPORTS
#        define RCSCORE_API __declspec(dllexport)
#    else
#        define RCSCORE_API __declspec(dllimport)
#    endif
//#endif

// See top-level CMakeLists.txt how RCS_BASE_PATH_LENGTH is defined
/* #if defined (RCS_BASE_PATH_LENGTH) */
/* #define __FILENAME__ (__FILE__ + RCS_BASE_PATH_LENGTH) */
/* #else */
/* #define __FILENAME__ (__FILE__) */
/* #endif */

#define strcasecmp _stricmp
#define strncasecmp _strnicmp
//#define snprintf   _snprintf
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

void srand48(unsigned int seed);
double drand48();

#if (_MSC_VER < 1900)
int round(double x);
double fmin(double x, double y);
double fmax(double x, double y);
double cbrt(double x);
#endif

#endif   // RCS_MSVC_H
#endif   // RCS_MSC_VER
