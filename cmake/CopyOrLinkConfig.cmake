################################################################################
#
#  Copyright (c) Honda Research Institute Europe GmbH.
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
#  CopyOrLinkConfig.cmake
#  A reusable function to copy or link the contents of a directory
# 
################################################################################

FUNCTION(RCS_COPY_OR_LINK_CONFIG SUBDIR_FROM SUBDIR_TO)

  # Ensure the SUBDIR_TO directory exists
  FILE(MAKE_DIRECTORY ${SUBDIR_TO})

  # Process subdirectories and contents
  FILE(GLOB SUBDIRS ${SUBDIR_FROM}/*)
  FOREACH(SUBDIR ${SUBDIRS})
    IF(IS_DIRECTORY ${SUBDIR})
      GET_FILENAME_COMPONENT(SUBDIR_NAME ${SUBDIR} NAME)
      SET(TARGET_SUBDIR ${SUBDIR_TO}/${SUBDIR_NAME})

      # Create the subdirectory if it doesn't exist
      IF(NOT EXISTS ${TARGET_SUBDIR})
        FILE(MAKE_DIRECTORY ${TARGET_SUBDIR})
        MESSAGE(STATUS "Created target subdirectory ${TARGET_SUBDIR}")
      ELSE()
        #MESSAGE(STATUS "Target subdirectory ${TARGET_SUBDIR} already exists")
      ENDIF()

      # Link or copy contents of the subdirectory
      FILE(GLOB SUBDIR_CONTENTS ${SUBDIR}/*)
      FOREACH(CONTENT ${SUBDIR_CONTENTS})
        GET_FILENAME_COMPONENT(ITEM_NAME ${CONTENT} NAME)
        SET(TARGET_ITEM ${TARGET_SUBDIR}/${ITEM_NAME})

        IF(NOT EXISTS ${TARGET_ITEM})
          IF(UNIX)
            #MESSAGE(STATUS "Linking ${CONTENT} to ${TARGET_ITEM}")
            EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CONTENT} ${TARGET_ITEM})
          ELSE()
            #MESSAGE(STATUS "Copying ${CONTENT} to ${TARGET_ITEM}")
            EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E copy ${CONTENT} ${TARGET_ITEM})
          ENDIF()
        ELSE()
          #MESSAGE(STATUS "Skipping existing item ${TARGET_ITEM}")
        ENDIF()
      ENDFOREACH()
    ENDIF()
  ENDFOREACH()
ENDFUNCTION()
