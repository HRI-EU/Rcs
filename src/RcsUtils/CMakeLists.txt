###############################################################################
#
# CMakeLists file for the RcsCore library
#
###############################################################################
SET(RCS_UTILS_SRCS
URDFGenerator/URDFAttributes.cpp
URDFGenerator/URDFElement.cpp
URDFGenerator/URDFGenerator.cpp
)

ADD_LIBRARY(RcsUtils ${LIBRARY_LINKING_TYPE} ${RCS_UTILS_SRCS})

IF(WIN32)
  SET_SOURCE_FILES_PROPERTIES(${RCS_UTILS_SRCS} PROPERTIES LANGUAGE CXX)
  TARGET_COMPILE_DEFINITIONS(RcsUtils PRIVATE -DRCSCORE_EXPORTS)
ELSE()
  TARGET_COMPILE_DEFINITIONS(RcsUtils PRIVATE -D_GNU_SOURCE)
ENDIF()

TARGET_INCLUDE_DIRECTORIES(RcsUtils PUBLIC
    # Header source location
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    
    # Header install location
    $<INSTALL_INTERFACE:include/RcsUtils>
)

TARGET_LINK_LIBRARIES(RcsUtils RcsGraphics)

# copy the .dll file to the same folder as the executable
IF (WIN32)
ADD_CUSTOM_COMMAND(
    TARGET RcsUtils POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
    ${LIBRARY_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}/RcsUtils.dll
    ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}/RcsUtils.dll)
ENDIF()

# Mark for export
RCS_ADD_EXPORT_LIBRARY(RcsUtils)

# Installation
# Headers
INSTALL(
  DIRECTORY . 
  DESTINATION include/RcsUtils
  FILES_MATCHING PATTERN "*.h"
)

# Install the binaries
INSTALL(TARGETS RcsUtils EXPORT RcsExport DESTINATION lib)
