################################################################################
#
# Locate a MuJoCo installation and provide the imported target
#
#   mujoco::mujoco
#
# The module is named after the package name that MuJoCo itself uses for its
# cmake package configuration file, so that find_package(mujoco) picks it up
# in module mode. It first tries that configuration file, and only assembles
# the target itself if there is none. This is needed because the macOS release
# of MuJoCo ships a framework without any cmake support.
#
# The search is platform-independent, no path is hard-coded. These prefixes
# are considered, in decreasing priority:
#
#   - the cache variable MUJOCO_DIR (cmake -DMUJOCO_DIR=<prefix>)
#   - the environment variable MUJOCO_DIR
#   - the SIT (HRI-internal installations)
#   - the locations where the MuJoCo releases are commonly unpacked or
#     installed, see MUJOCO_RELEASE_HINTS below
#
# On top of these hints, cmake searches CMAKE_PREFIX_PATH and the system
# default prefixes, which covers installations done through a package manager.
#
# Output variables:
#
#   mujoco_FOUND        - True if MuJoCo was found
#   mujoco_ORIGIN       - Human readable description of what was found
#   MUJOCO_LIBRARY      - The library (or framework), if found without config
#   MUJOCO_INCLUDE_DIR  - The include directory, if found without config
#
################################################################################

INCLUDE(FindPackageHandleStandardArgs)

SET(MUJOCO_DIR "$ENV{MUJOCO_DIR}" CACHE PATH
    "Root directory of a MuJoCo installation")

# Globs that do not match anything contribute nothing, so all of these can be
# listed irrespective of the platform we are building on. The versioned
# directories are reversed, so that the most recent version comes first.
FILE(GLOB MUJOCO_RELEASE_HINTS
     "$ENV{HOME}/.mujoco/mujoco*"      # MuJoCo releases, traditional location
     "$ENV{HOME}/mujoco*"
     "/opt/mujoco*"
     "$ENV{HOMEBREW_PREFIX}/Caskroom/mujoco/*"   # brew install mujoco (cask)
     "/opt/homebrew/Caskroom/mujoco/*"
     "/usr/local/Caskroom/mujoco/*"
     "C:/Program Files/mujoco*")
LIST(REVERSE MUJOCO_RELEASE_HINTS)

# MKPLT is set by the top-level CMakeLists file. It is empty if this module is
# used from outside the Rcs build, which is fine - the entry then simply does
# not match anything.
SET(MUJOCO_SEARCH_PREFIXES
    ${MUJOCO_DIR}
    $ENV{SIT}/External/MuJoCo/1.0/${MKPLT}
    ${MUJOCO_RELEASE_HINTS})

SET(mujoco_ORIGIN "")

################################################################################
# The preferred way: the Linux and Windows releases, as well as most package
# managers, ship a cmake package configuration file in lib/cmake/mujoco. It
# defines mujoco::mujoco with all its usage requirements. Requesting CONFIG
# mode explicitly makes sure that we do not recurse into this module.
################################################################################
FIND_PACKAGE(mujoco CONFIG QUIET HINTS ${MUJOCO_SEARCH_PREFIXES})

IF (TARGET mujoco::mujoco)
  SET(mujoco_ORIGIN "${mujoco_DIR} (package configuration file)")
ENDIF()

################################################################################
# No configuration file: search for the library and the headers, and assemble
# the imported target ourselves, so that users of this module do not need to
# distinguish the two cases. The target is created GLOBAL, so that it can also
# be used from a different directory than the one calling find_package().
################################################################################
IF (NOT TARGET mujoco::mujoco)

  FIND_LIBRARY(MUJOCO_LIBRARY NAMES mujoco
               HINTS ${MUJOCO_SEARCH_PREFIXES}
               PATH_SUFFIXES lib bin
               DOC "MuJoCo library or framework")

  IF (MUJOCO_LIBRARY AND MUJOCO_LIBRARY MATCHES "\\.framework$")

    # macOS framework. The compiler resolves <mujoco/mujoco.h> through the
    # framework search path, so there is no include directory to add. The
    # library has an @rpath install name, hence the additional rpath entry.
    GET_FILENAME_COMPONENT(MUJOCO_FRAMEWORK_DIR ${MUJOCO_LIBRARY} DIRECTORY)

    ADD_LIBRARY(mujoco::mujoco INTERFACE IMPORTED GLOBAL)
    SET_TARGET_PROPERTIES(mujoco::mujoco PROPERTIES
      INTERFACE_COMPILE_OPTIONS "-F${MUJOCO_FRAMEWORK_DIR}"
      INTERFACE_COMPILE_FEATURES "cxx_std_17"
      INTERFACE_LINK_LIBRARIES "${MUJOCO_LIBRARY};-Wl,-rpath,${MUJOCO_FRAMEWORK_DIR}")

    SET(mujoco_ORIGIN "${MUJOCO_LIBRARY} (framework)")

  ELSEIF (MUJOCO_LIBRARY)

    # Plain <prefix>/lib + <prefix>/include layout.
    GET_FILENAME_COMPONENT(MUJOCO_LIBRARY_DIR ${MUJOCO_LIBRARY} DIRECTORY)

    FIND_PATH(MUJOCO_INCLUDE_DIR NAMES mujoco/mujoco.h
              HINTS ${MUJOCO_SEARCH_PREFIXES} ${MUJOCO_LIBRARY_DIR}/..
              PATH_SUFFIXES include
              DOC "Directory containing mujoco/mujoco.h")

    IF (MUJOCO_INCLUDE_DIR)
      ADD_LIBRARY(mujoco::mujoco INTERFACE IMPORTED GLOBAL)
      SET_TARGET_PROPERTIES(mujoco::mujoco PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}"
        INTERFACE_COMPILE_FEATURES "cxx_std_17"
        INTERFACE_LINK_LIBRARIES "${MUJOCO_LIBRARY}")

      SET(mujoco_ORIGIN "${MUJOCO_LIBRARY}")
    ENDIF()

  ENDIF()

ENDIF(NOT TARGET mujoco::mujoco)

################################################################################
# This handles the REQUIRED and QUIET arguments, prints the standard message,
# and sets mujoco_FOUND. The MuJoCo configuration file may already have set
# mujoco_FOUND above - we deliberately let it be determined here again, so
# that both discovery paths end up with the same semantics.
################################################################################
FIND_PACKAGE_HANDLE_STANDARD_ARGS(mujoco
  REQUIRED_VARS mujoco_ORIGIN
  FAIL_MESSAGE "Could not find MuJoCo. Point cmake to an installation with -DMUJOCO_DIR=<prefix>, where <prefix> is its root directory")

MARK_AS_ADVANCED(MUJOCO_LIBRARY MUJOCO_INCLUDE_DIR)
