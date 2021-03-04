################################################################################
#
#  Copyright (c) 2017, Honda Research Institute Europe GmbH.
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
#  3. Neither the name of the copyright holder nor the names of its
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

IF( DEFINED ENV{WM5_DIR} )
  SET( WM5_DIR "$ENV{WM5_DIR}" )
ENDIF()

SET(WM5_DIR_HINT "$ENV{SIT}/External/GeometricTools/5.10/")
SET(WM5_DIR_HINT2 "${PROJECT_SOURCE_DIR}/../thirdParty/WildMagic5/SDK/Library/ReleaseDynamic/")
SET(WM5_DIR_HINT3 "${PROJECT_SOURCE_DIR}/../thirdParty/WildMagic5/SDK/Include")

FIND_PATH(WM5_INCLUDE_DIR
  Wm5Vector3.h
  HINTS "${WM5_DIR_HINT}" "${WM5_DIR_HINT2}" "${WM5_DIR_HINT3}"
  ENV WM5_DIR
  PATH_SUFFIXES include
  )

FIND_LIBRARY(WM5_CORE_LIBRARY
  NAMES Wm5Core
  HINTS "${WM5_DIR_HINT}" "${WM5_DIR_HINT2}" "${WM5_DIR_HINT3}"
  ENV WM5_DIR
  PATH_SUFFIXES lib/$ENV{MAKEFILE_PLATFORM}
  )

FIND_LIBRARY(WM5_MATHEMATICS_LIBRARY
  NAMES Wm5Mathematics
  HINTS "${WM5_DIR_HINT}" "${WM5_DIR_HINT2}" "${WM5_DIR_HINT3}"
  ENV WM5_DIR
  PATH_SUFFIXES lib/$ENV{MAKEFILE_PLATFORM}
  )

SET(WM5_INCLUDE_DIR ${WM5_INCLUDE_DIR})
SET(WM5_LIBRARIES ${WM5_MATHEMATICS_LIBRARY} ${WM5_CORE_LIBRARY})
IF(UNIX)
  SET(WM5_FLAGS "-isystem ${WM5_INCLUDE_DIR}")
ENDIF(UNIX)
SET(WM5_DEFINITIONS USE_WM5)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(WM5 DEFAULT_MSG
  WM5_INCLUDE_DIR
  WM5_LIBRARIES
  )

SET(WM5_FOUND ${WM5_FOUND})
