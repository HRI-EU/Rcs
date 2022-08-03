#!/bin/bash
ORG_DIR=$PWD
RCS_DIR=$(cd `dirname $0` && pwd)
STYLE_CMD="$RCS_DIR/apply_coding_style.sh"

# all source files in src and projects:
#FILES_TO_CHECK=`find bin src projects -type f -name *.c -o -name *.h -o -name *.cpp | xargs`
FILES_TO_CHECK=`find . -type f -name *.c -o -name *.h -o -name *.cpp | xargs`

if [ "$FILES_TO_CHECK" != "" ]; then
  $STYLE_CMD $FILES_TO_CHECK
  #echo $STYLE_CMD $FILES_TO_CHECK
  #echo "Disabled for now"
fi
cd $ORG_DIR
