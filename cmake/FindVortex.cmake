################################################################################
#
#  Copyright (c) 2017, Honda Research Institute Europe GmbH.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
#  3. All advertising materials mentioning features or use of this software
#     must display the following acknowledgement: This product includes
#     software developed by the Honda Research Institute Europe GmbH.
#
#  4. Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER "AS IS" AND ANY EXPRESS OR
#  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
#  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
#  IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
#  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
#  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
#  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
#  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
################################################################################

INCLUDE(FindPackageHandleStandardArgs)

IF( DEFINED ENV{VORTEX_DIR} )
  SET( VORTEX_DIR "$ENV{VORTEX_DIR}" )
ENDIF()

SET(VORTEX_DIR_HINT1 "$ENV{SIT}/External/Vortex/6.8.1")
SET(VORTEX_DIR_HINT2 "$ENV{SIT}/External/Vortex/6.8.1/$ENV{MAKEFILE_PLATFORM}")
SET(VORTEX_DIR_HINT3 "/home/$ENV{USER}/Software/Vortex")
SET(VORTEX_DIR_HINT4 "${PROJECT_SOURCE_DIR}/../thirdParty/Vortex")

FIND_PATH(VORTEX_INCLUDE_DIR
  Vx/VxVersion.h
  HINTS "${VORTEX_DIR_HINT1}" "${VORTEX_DIR_HINT2}" "${VORTEX_DIR_HINT3}" "${VORTEX_DIR_HINT4}"
  ENV VORTEX_DIR
  PATH_SUFFIXES include
  )

FIND_PATH(VORTEX_ESSENTIALS_DIR
  lib/libVxCore.so
  HINTS "${VORTEX_DIR_HINT1}" "${VORTEX_DIR_HINT2}" "${VORTEX_DIR_HINT3}" "${VORTEX_DIR_HINT4}"
  )

FIND_LIBRARY(VORTEX_CORE_LIBRARY
  NAMES VxCore VxCore.lib
  HINTS "${VORTEX_DIR_HINT1}" "${VORTEX_DIR_HINT2}" "${VORTEX_DIR_HINT3}" "${VORTEX_DIR_HINT4}"
  ENV VORTEX_DIR
  PATH_SUFFIXES lib
  )

FIND_LIBRARY(VORTEX_DYNAMICS_LIBRARY
  NAMES VxDynamics VxDynamics.lib
  HINTS "${VORTEX_DIR_HINT1}" "${VORTEX_DIR_HINT2}" "${VORTEX_DIR_HINT3}" "${VORTEX_DIR_HINT4}"
  ENV VORTEX_DIR
  PATH_SUFFIXES lib
  )

IF (WIN32)
  
  FIND_LIBRARY(VORTEX_MATH_LIBRARY
    NAMES VxMath.lib
    HINTS "${VORTEX_DIR_HINT1}" "${VORTEX_DIR_HINT2}" "${VORTEX_DIR_HINT3}" "${VORTEX_DIR_HINT4}"
    ENV VORTEX_DIR
    PATH_SUFFIXES lib
    )
  
  FIND_LIBRARY(VORTEX_PLATFORM_LIBRARY
    NAMES VxPlatform.lib
    HINTS "${VORTEX_DIR_HINT1}" "${VORTEX_DIR_HINT2}" "${VORTEX_DIR_HINT3}" "${VORTEX_DIR_HINT4}"
    ENV VORTEX_DIR
    PATH_SUFFIXES lib
    )
  
  FIND_LIBRARY(VORTEX_FOUNDATION_LIBRARY
    NAMES VxFoundation.lib
    HINTS "${VORTEX_DIR_HINT1}" "${VORTEX_DIR_HINT2}" "${VORTEX_DIR_HINT3}" "${VORTEX_DIR_HINT4}"
    ENV VORTEX_DIR
    PATH_SUFFIXES lib
    )
  
ENDIF(WIN32)

SET(VORTEX_INCLUDE_DIR ${VORTEX_INCLUDE_DIR})
SET(VORTEX_ESSENTIALS_DIR ${VORTEX_ESSENTIALS_DIR})
SET(VORTEX_LIBRARIES
  ${VORTEX_CORE_LIBRARY}
  ${VORTEX_DYNAMICS_LIBRARY}
  ${VORTEX_MATH_LIBRARY}
  ${VORTEX_PLATFORM_LIBRARY}
  ${VORTEX_FOUNDATION_LIBRARY})

IF(UNIX)
  SET(VORTEX_DEFINITIONS LINUX)
ENDIF(UNIX)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(VORTEX DEFAULT_MSG
  VORTEX_INCLUDE_DIR
  VORTEX_LIBRARIES
  )

SET(VORTEX_FOUND ${VORTEX_FOUND})
