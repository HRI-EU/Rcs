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

#ifndef RCS_GRADIENTTESTS_H
#define RCS_GRADIENTTESTS_H

#include "Rcs_graph.h"

#ifdef __cplusplus
extern "C" {
#endif



/*  \brief Array x is assumed to have dof elements. The function traverses
 *         all joints and sets the unconstrained joints to the respective
 *         value of the x array.
 */
void Rcs_setIKState(RcsGraph* self, const double* x);

/*  \brief Performs a bunch of gradient test for the given graph.
 *  \param[in] graph RcsGraph to test
 *  \param[in] q State vector: Graph will be tested in this configuration.
 *             The state vector may be of dimention RcsGraph::dof or
 *             RcsGraph::nJ
 *  \param[in] verbose Flag to indicate if function should be silent or not.
 *             If verbose is true, the function will pause at some failure
 *             locations, and print debug information to the console.
 *  \return true for success, false for failure.
 */
bool Rcs_gradientTestGraph(RcsGraph* g, const MatNd* q, bool verbose);

/*  \brief Returns the randomly selected body 1. It is assigned in the
 *         testGradient() function.
 */
RcsBody* Rcs_getRndBdy1(void);

/*  \brief Returns the randomly selected body 2. It is assigned in the
 *         testGradient() function.
 */
RcsBody* Rcs_getRndBdy2(void);

/*  \brief Sets the gradient test body 1 to b.
 */
void Rcs_setGradientTestBdy1(RcsBody* b);

/*  \brief Sets the gradient test body 2 to b.
 */
void Rcs_setGradientTestBdy2(RcsBody* b);

#ifdef __cplusplus
}
#endif

#endif   // RCS_GRADIENTTESTS_H
