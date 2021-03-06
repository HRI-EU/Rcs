# Global C++11 settings
IF(ENABLE_C++11)
  SET(CMAKE_CXX_STANDARD 11)
  SET(CMAKE_CXX_STANDARD_REQUIRED ON)
ELSEIF(UNIX)
  SET(CMAKE_CXX_FLAGS "-std=c++0x")
ENDIF()

# Some special treatment for 32-bit and 64-bit machines
IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
  ADD_DEFINITIONS(-D__64BIT__)
ELSE()
  ADD_DEFINITIONS(-D__32BIT__)
ENDIF()



IF(WIN32)

  ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE -D_CRT_NONSTDC_NO_DEPRECATE)
  IF (NOT (MSVC_VERSION LESS 1900))
    ADD_DEFINITIONS(-DHAVE_STRUCT_TIMESPEC)
  ENDIF()
  
  # Create shared libraries for Linux and Windows
  # For CMake 3.4 and higher, there is an easy way to create windows dlls. For
  # details, see http://www.kitware.com/blog/home/post/939
  IF (CMAKE_VERSION VERSION_LESS 3.4)
    MESSAGE(FATAL_ERROR "CMake version too old, at least 3.4 is needed")
  ELSE()
    SET(BUILD_SHARED_LIBS ON)
    SET(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
    SET(CREATE_WIN_DLL TRUE)
    ADD_DEFINITIONS(-DWIN_DLL)
  ENDIF()

ELSEIF(UNIX)

  SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -Wall -pedantic -fPIC -Wno-long-long -Wno-variadic-macros -std=c99 -ggdb")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -fPIC -Wno-format -Wno-long-long -Wno-variadic-macros -ggdb")
  ADD_DEFINITIONS(-D__linux__)

  IF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -rdynamic")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -rdynamic")
  ENDIF()

  IF(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    # setting march to core2 to enable valgrind debugging (also good for Xeon)
    # Low level optimization for debug mode, flag for checking stack corruption, flag for debug output
    SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -march=core2 -O0 -fstack-protector-all")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=core2 -O0 -fstack-protector-all")
  ELSE()
    # setting march to native for optimal performance on local machine
    # Strong optimization for release mode
    SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -march=native -O2")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -O2")
  ENDIF()

ENDIF()

