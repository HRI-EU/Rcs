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

#include "Rcs_macros.h"


RCSCORE_API int RcsLogLevel = 0;

// This function is declared static inline in order to avoid the "defined but
// not used" compiler warnings
#if defined(_MSC_VER)
static void* Rcs_debugCalloc(size_t nmemb, size_t size)
{
  RLOGS(1, "Allocating %Iu variables of size %Iu", nmemb, size);
  return calloc(nmemb, size);
}
#else
static inline void* Rcs_debugCalloc(size_t nmemb, size_t size)
{
  RLOGS(1, "Allocating %zu variables of size %zu", nmemb, size);
  return calloc(nmemb, size);
}
#endif

// by default we don't have a registered callback
RLOG_CB_FUNC RCS_GLOBAL_RLOG_CB = NULL;

// by default use the system's calloc function for dynamic memory allocation
RCSCORE_API RCS_MALLOC_FUNC Rcs_calloc = calloc;

