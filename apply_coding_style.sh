#!/bin/bash
RCS_DIR=$(cd `dirname $0` && pwd)

ALLOWED_EXTENTIONS="h c cpp hpp"
IGNORE_LIST=""
#IGNORE_LIST="ShmDataTypeMacros.h"

# If someone switches to a newer astyle version
# just change this string to match the start of astyle version string.
ASTYLE_VERSION="Artistic Style Version 3.1"
ASTYLE_CMD=astyle


# read filename(s) from parameters
if [ -z $1 ]; then
  echo "Please pass the file(s) you wish to apply the coding style to."
  echo "Only C and C++ files (.h, .c, .cpp) are processed."
fi

function notInIgnoreList()
{
  for i in $IGNORE_LIST; do
#     echo "testing $1 for ignores"
    if [[ "$1" == *"$i" ]]; then
      echo "$1 is in ignore list, skipping..."
      return 1
    fi
  done
  return 0
}

FILES=""
for GIVEN_FILE in $@; do
  if [ -f $GIVEN_FILE ]; then
    if notInIgnoreList $GIVEN_FILE; then
      EXTENTION=${GIVEN_FILE##*.}
      for ALLOWED_EXTENTION in $ALLOWED_EXTENTIONS; do
        if [ "$ALLOWED_EXTENTION" = "$EXTENTION" ]; then
          FILES="$FILES $GIVEN_FILE"
        fi
      done
    fi
  else
    echo "Could not find file \"$GIVEN_FILE\""
  fi
done
if [ "$FILES" = "" ]; then
  echo "You did not pass any source code files."
  exit
fi

if [[ "`$ASTYLE_CMD --version 2>&1`" != ${ASTYLE_VERSION}* ]]; then
        echo "You should use: ${ASTYLE_VERSION}";
        exit 1;
fi

style="--style=allman \
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
options="--lineend=linux --formatted --suffix=.before_astyle"

# finally, check all given files
for FILE in $FILES; do
  $ASTYLE_CMD $style $options $FILE
done
