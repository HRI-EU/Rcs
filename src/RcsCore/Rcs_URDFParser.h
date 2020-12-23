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

#ifndef RCS_URDFPARSER_H
#define RCS_URDFPARSER_H

#include "Rcs_graph.h"

#ifdef __cplusplus
extern "C" {
#endif


/*! \brief Creates all bodies and joints found in the configuration file. The
 *         file is assumed to exist, it is not searched in the resource path.
 *         There is no assumption on the name of the configFile, however it is
 *         expected to be a valid urdf description file.
 *
 *  \param[in,out] graph    Graph to be extended by URDF model
 *  \param[in] configFile   Full path of URDF configuration file. If it cannot
 *                          be accessed, NULL is returned, and a warning is
 *                          emitted on debug level 1.
 *  \param[in] suffix       Optional character array that is appended to all
 *                          body and joint names. If it is NULL, it is ignored.
 *  \param[in] A_BP         Optional relative transformation that is applied to
 *                          all bodies that have no parent (root level). If it
 *                          is NULL, it is ignored.
 *  \param[out] dof         Number of parsed degrees of freedom. If the pointer
 *                         is NULL, it is ignored.
 *  \return Id of root body of parsed graph. It can be different from 0 if the
 *          URDF graph is parsed as a sub-graph. In case of failure, -1 is
 *          returned.
 */
int RcsGraph_rootBodyFromURDFFile(RcsGraph* graph,
                                  const char* configFile,
                                  const char* suffix,
                                  const HTr* A_BP,
                                  unsigned int* dof);

/*! \brief Creates a complete RcsGraph given the configuration file. The file
 *         is searched in the resource path. There is no assumption on the
 *         name of the configFile, however it is expected to be a valid urdf
 *         description file.
 *
 *  \param[in] configFile   Full path of URDF configuration file. If it cannot
 *                          be accessed, NULL is returned, and a warning is
 *                          emitted on debug level 1.
 */
RcsGraph* RcsGraph_fromURDFFile(const char* configFile);


#ifdef __cplusplus
}
#endif


#endif // RCS_URDFPARSER_H
