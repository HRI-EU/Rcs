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

IF( DEFINED ENV{QPOASES_DIR} )
  SET( QPOASES_DIR "$ENV{QPOASES_DIR}" )
ENDIF()

SET(QPOASES_DIR_HINT1 "$ENV{SIT}/External/qpOASES/3.2/$ENV{MAKEFILE_PLATFORM}")
SET(QPOASES_DIR_HINT2 "$ENV{SIT}/External/qpOASES-3.2.1")

FIND_PATH(QPOASES_INCLUDE_DIR
  qpOASES.hpp
  HINTS "${QPOASES_DIR_HINT1}" "${QPOASES_DIR_HINT2}"
  ENV QPOASES_DIR
  PATH_SUFFIXES include
  )

FIND_LIBRARY(QPOASES_LIBRARIES
  NAMES qpOASES-shared qpOASES qpOASES.lib
  HINTS "${QPOASES_DIR_HINT1}" "${QPOASES_DIR_HINT2}"
  ENV QPOASES_DIR
  PATH_SUFFIXES lib
  )

SET(QPOASES_INCLUDE_DIR ${QPOASES_INCLUDE_DIR})
SET(QPOASES_LIBRARIES ${QPOASES_LIBRARIES})

# The __USE_LONG_INTEGERS__ __USE_LONG_FINTS__ are set within the qpOASES
# build. We need to consider them here to match the function signatures.
# Otherwise, we get weird linker errors about non-matching rvalues.
IF(UNIX)
  SET(QPOASES_DEFINITIONS USE_QPOASES LINUX __USE_LONG_INTEGERS__ __USE_LONG_FINTS__)
ELSE()
  SET(QPOASES_DEFINITIONS USE_QPOASES)
ENDIF(UNIX)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(QPOASES DEFAULT_MSG
  QPOASES_INCLUDE_DIR
  QPOASES_LIBRARIES
  )

SET(QPOASES_FOUND ${QPOASES_FOUND})
