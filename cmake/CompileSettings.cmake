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

# Global C++11 settings
#IF(ENABLE_C++11)
#  SET(CMAKE_CXX_STANDARD 11)
#  SET(CMAKE_CXX_STANDARD_REQUIRED ON)
#ELSEIF(UNIX)
#  SET(CMAKE_CXX_FLAGS "-std=c++0x")
#ENDIF()

# Some special treatment for 32-bit and 64-bit machines
#IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
#  ADD_DEFINITIONS(-D__64BIT__)
#ELSE()
#  ADD_DEFINITIONS(-D__32BIT__)
#ENDIF()



# Create shared libraries for Linux and Windows: see http://www.kitware.com/blog/home/post/939
#IF(WIN32)
#  ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE -D_CRT_NONSTDC_NO_DEPRECATE)
#  SET(BUILD_SHARED_LIBS ON)
#  SET(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
#  SET(CREATE_WIN_DLL TRUE)
#  ADD_DEFINITIONS(-DWIN_DLL)
#ELSEIF(UNIX)
#  SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -Wall -pedantic -fPIC -Wno-long-long -Wno-variadic-macros -std=c99")
#  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -fPIC -Wno-format -Wno-long-long -Wno-variadic-macros")
#  ADD_DEFINITIONS(-D__linux__)

#  # Flag -rdynamic needed for backtrace()
#  IF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
#    SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -rdynamic")
#    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -rdynamic")
#  ENDIF()
  
#  IF(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
#    # setting march to core2 to enable valgrind debugging (also good for Xeon)
#    # Low level optimization for debug mode, flag for checking stack corruption, flag for debug output
#    SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -march=core2 -O0 -fstack-protector-all -ggdb")
#    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=core2 -O0 -fstack-protector-all -ggdb")
#  ELSE()
#    # setting march to native for optimal performance on local machine
#    # Strong optimization for release mode
#    SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -march=native -ggdb")
#    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -ggdb")
#  ENDIF()
#ENDIF()

