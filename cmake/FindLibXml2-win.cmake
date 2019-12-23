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

IF (WIN32)
  
  INCLUDE(FindPackageHandleStandardArgs)

  IF(DEFINED ENV{LIBXML2_DIR})
    SET(LIBXML2 "$ENV{LIBXML2_DIR}")
  ENDIF()

  SET(LIBXML2_DIR_HINT "$ENV{SIT}/External/libxml2-win/2.78")
  SET(LIBXML2_INCDIR_HINT "$ENV{SIT}/External/libxml2-win/2.78")
  SET(LIBXML2_LIBDIR_HINT "$ENV{SIT}/External/libxml2-win/2.78/lib/$ENV{MAKEFILE_PLATFORM}")

  FIND_PATH(LIBXML2_INCLUDE_DIR
    libxml/xmlversion.h
    HINTS "${LIBXML2_INCDIR_HINT}"
    PATH_SUFFIXES include
    )

  FIND_LIBRARY(LIBXML2_LIBRARIES
    NAMES RcsLibXml2.lib
    HINTS "${LIBXML2_DIR_HINT}"
    PATH_SUFFIXES lib/$ENV{MAKEFILE_PLATFORM}
    )

  SET(LIBXML2_INCLUDE_DIR ${LIBXML2_INCLUDE_DIR})
  SET(LIBXML2_LIBRARIES ${LIBXML2_LIBRARIES})

  FIND_PACKAGE_HANDLE_STANDARD_ARGS(LIBXML2 DEFAULT_MSG
    LIBXML2_INCLUDE_DIR
    LIBXML2_LIBRARIES
    )

  SET(LIBXML2_FOUND ${LIBXML2_FOUND})

ENDIF(WIN32)
