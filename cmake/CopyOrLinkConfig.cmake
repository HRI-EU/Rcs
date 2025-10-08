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

function(RCS_COPY_OR_LINK_CONFIG SUBDIR_FROM SUBDIR_TO)
  # Normalize to absolute to keep relative math stable across recursion
  get_filename_component(SRC_ABS "${SUBDIR_FROM}" ABSOLUTE)
  get_filename_component(DST_ABS "${SUBDIR_TO}"   ABSOLUTE)

  file(MAKE_DIRECTORY "${DST_ABS}")

  # React to new files/dirs being added later; include dotfiles if needed
  file(GLOB CHILDREN
       CONFIGURE_DEPENDS
       LIST_DIRECTORIES true
       "${SRC_ABS}/*" "${SRC_ABS}/.*")  # drop the second pattern if you don't want dotfiles

  foreach(CHILD IN LISTS CHILDREN)
    get_filename_component(NAME "${CHILD}" NAME)
    set(TARGET_PATH "${DST_ABS}/${NAME}")

    if(IS_DIRECTORY "${CHILD}")
      RCS_COPY_OR_LINK_CONFIG("${CHILD}" "${TARGET_PATH}")
    else()
      if(UNIX)
        # Compute relative target for the symlink
        file(RELATIVE_PATH REL_SRC "${DST_ABS}" "${CHILD}")

        # If an entry exists, replace it if it's a different link/regular file
        if(EXISTS "${TARGET_PATH}")
          # Replace if it's not a symlink to the expected target
          if(NOT IS_SYMLINK "${TARGET_PATH}")
            file(REMOVE "${TARGET_PATH}")
          else()
            file(READ_SYMLINK "${TARGET_PATH}" CUR_TARGET)
            if(NOT CUR_TARGET STREQUAL "${REL_SRC}")
              file(REMOVE "${TARGET_PATH}")
            endif()
          endif()
        endif()

        if(NOT EXISTS "${TARGET_PATH}")
          if(POLICY CMP0133) # implies CMake >= 3.24 typically; guard not strictly needed
            # Preferred: native link creation
            file(CREATE_LINK "${REL_SRC}" "${TARGET_PATH}" SYMBOLIC)
          else()
            # Fallback
            execute_process(COMMAND "${CMAKE_COMMAND}" -E create_symlink "${REL_SRC}" "${TARGET_PATH}")
          endif()
        endif()
      else()
        # Windows / others: copy only if changed
        if(NOT EXISTS "${TARGET_PATH}")
          execute_process(COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${CHILD}" "${TARGET_PATH}")
        endif()
      endif()
    endif()
  endforeach()
endfunction()

















FUNCTION(RCS_COPY_OR_LINK_CONFIG_ABS SUBDIR_FROM SUBDIR_TO)

  # Ensure the SUBDIR_TO directory exists
  FILE(MAKE_DIRECTORY ${SUBDIR_TO})

  # Process subdirectories and contents
  FILE(GLOB SUBDIRS ${SUBDIR_FROM}/*)
  FOREACH(SUBDIR ${SUBDIRS})
    IF(IS_DIRECTORY ${SUBDIR})
      GET_FILENAME_COMPONENT(SUBDIR_NAME ${SUBDIR} NAME)
      SET(TARGET_SUBDIR ${SUBDIR_TO}/${SUBDIR_NAME})

      # Recursively call the function for subdirectories
      RCS_COPY_OR_LINK_CONFIG(${SUBDIR} ${TARGET_SUBDIR})
    ELSE()
      # Handle files in the current directory
      GET_FILENAME_COMPONENT(ITEM_NAME ${SUBDIR} NAME)
      SET(TARGET_ITEM ${SUBDIR_TO}/${ITEM_NAME})

      IF(NOT EXISTS ${TARGET_ITEM})
        IF(UNIX)
	  # Compute relative path from link location to source
          FILE(RELATIVE_PATH REL_SRC ${SUBDIR_TO} ${SUBDIR})
          # Use the relative path when creating the symlink
          EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E create_symlink ${REL_SRC} ${TARGET_ITEM})

	  # Compute absolute path from link location to source
          #MESSAGE(STATUS "Linking ${SUBDIR} to ${TARGET_ITEM}")
          #EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E create_symlink ${SUBDIR} ${TARGET_ITEM})
        ELSE()
          #MESSAGE(STATUS "Copying ${SUBDIR} to ${TARGET_ITEM}")
          EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E copy ${SUBDIR} ${TARGET_ITEM})
        ENDIF()
      ELSE()
        #MESSAGE(STATUS "Skipping existing item ${TARGET_ITEM}")
      ENDIF()
    ENDIF()
  ENDFOREACH()
ENDFUNCTION()
