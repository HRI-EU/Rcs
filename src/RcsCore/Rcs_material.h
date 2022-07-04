/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH

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

#ifndef RCS_MATERIAL_H
#define RCS_MATERIAL_H

#ifdef __cplusplus
extern "C" {
#endif


typedef struct
{
  double amb[4];
  double diff[4];
  double spec[4];
  double shininess;

} RcsMaterial;



/*! \ingroup RcsUtilsFunctions
 *  \brief Get a material, load it from file config/colors/colors.xml if
 *         necessary. The returned data is added to a material map for a fast
 *         look-up. Therefore, the caller must not delete it.
 *
 * Returns Pointer to material, or NULL if material could not be found.
 */
const RcsMaterial* Rcs_getMaterial(const char* materialName);


/*! \ingroup RcsUtilsFunctions
 *  \brief Returns the ambient portion of the color given by string color.
 *         The following colors are defined (case insensitive):
 *         - RED
 *         - WHITE
 *         - BLACK
 *         - GREEN
 *         - BLUE
 *         - RUBY
 *         - YELLOW
 *         - BRASS
 *         - PEWTER
 *         - BRONZE
 *         - EMERALD
 *         - LIGHT_GRAYISH_BLUE
 *         - LIGHT_GRAYISH_YELLOW
 *         - LIGHT_GRAYISH_GREEN
 *         - RED_TRANS
 *         - GREEN_TRANS
 *         - BLUE_TRANS
 *         If the string color is none of them or NULL, false will be returned,
 *         white will be copied into rgba, and a warning on debug level 4 will
 *         be displayed.
 */
bool Rcs_colorFromString(const char* color, double rgba[4]);


#ifdef __cplusplus
}
#endif

#endif   // RCS_MATERIAL_H
