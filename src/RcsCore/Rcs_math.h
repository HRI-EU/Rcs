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

#ifndef RCS_MATH_H
#define RCS_MATH_H

/*!
 *  \page RcsMath Mathematics functions in C
 *
 *        <h1> Mathematics functions </h1>
 *
 *        This is a library of vector and matrix functions written in C.
 *        The functions have been implemented to be efficient for small
 *        dimensions (<50). If performance is an issue, libraries like
 *        Lapack or Eigen3 are better suited for arrays of larger dimensions.
 *
 *        They functions are relatively safe in terms of indexing etc.
 *        In each function, many checks are performed. Checking can be
 *        disabled by defining the macro RCS_NO_DEBUG to increase performance.
 *
 *        The function signatures are all consistently done to have the
 *        first argument as the target argument (if applicable). The
 *        const keyword is used to indicate which arguments are not
 *        changed. This unfortunately does not work for data types
 *        with 2 indices, like double[3][3]. If you are interested why,
 *        please have a look to
 *        http://gcc.gnu.org/bugzilla/show_bug.cgi?id=47143
 *
 *        \ref RcsVec3dFunctions
 *
 *        \ref RcsVecNFunctions
 *
 *        \ref RcsMat3dFunctions
 *
 *        \ref MatNdFunctions
 *
 *        \ref RcsHTrFunctions
 *
 *        \ref RcsBasicMathFunctions
 */

#include "Rcs_Vec3d.h"
#include "Rcs_VecNd.h"
#include "Rcs_Mat3d.h"
#include "Rcs_basicMath.h"

#endif   // RCS_MATH_H
