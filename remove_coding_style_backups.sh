#!/bin/bash
RCS_DIR=$(cd `dirname $0` && pwd)
echo "Deleting astyle backup files from: $RCS_DIR"
find $RCS_DIR -name "*.before_astyle" -delete
