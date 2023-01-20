#!/bin/bash
#
#  Copyright (c) 2020, Honda Research Institute Europe GmbH.
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

set -euo pipefail

RCS_DIR=$(cd "$(dirname "$0")" && pwd)

ALLOWED_EXTENTIONS="h c cpp hpp"
IGNORE_LIST=""
#IGNORE_LIST="ShmDataTypeMacros.h"

# If someone switches to a newer astyle version
# just change this string to match the start of astyle version string.
ASTYLE_VERSION="Artistic Style Version 3.1"
ASTYLE_CMD=astyle


# read filename(s) from parameters
if [ -z "$1" ]; then
  echo "Please pass the file(s) you wish to apply the coding style to."
  echo "Only C and C++ files (.h, .c, .cpp) are processed."
fi

function notInIgnoreList()
{
  for i in ${IGNORE_LIST}; do
#     echo "testing $1 for ignores"
    if [[ "$1" == *"${i}" ]]; then
      echo "$1 is in ignore list, skipping..."
      return 1
    fi
  done
  return 0
}

FILES=""
for GIVEN_FILE in "$@"; do
  if [ -f "${GIVEN_FILE}" ]; then
    if notInIgnoreList "${GIVEN_FILE}"; then
      EXTENTION=${GIVEN_FILE##*.}
      for ALLOWED_EXTENTION in ${ALLOWED_EXTENTIONS}; do
        if [ "${ALLOWED_EXTENTION}" = "${EXTENTION}" ]; then
          FILES="${FILES} ${GIVEN_FILE}"
        fi
      done
    fi
  else
    echo "Could not find file \"${GIVEN_FILE}\""
  fi
done
if [ "${FILES}" = "" ]; then
  echo "You did not pass any source code files."
  exit
fi

if [[ "$(${ASTYLE_CMD} --version 2>&1)" != ${ASTYLE_VERSION}* ]]; then
        echo "You should use: ${ASTYLE_VERSION}";
        exit 1;
fi

style=" --style=allman \
       --indent=spaces=2 \
       --attach-extern-c \
       --indent-switches \
       --indent-preprocessor \
       --indent-col1-comments \
       --max-instatement-indent=70 \
       --min-conditional-indent=0 \
       --break-closing-brackets \
       --add-brackets \
       --convert-tabs \
       --align-pointer=type \
       --align-reference=type \
       --pad-header \
       --unpad-paren"
options=" --lineend=linux --formatted --suffix=.before_astyle"

# finally, check all given files
for FILE in ${FILES}; do
  EXE_CMD="${ASTYLE_CMD} ${style} ${options} ${FILE}"
  ${EXE_CMD}
done
