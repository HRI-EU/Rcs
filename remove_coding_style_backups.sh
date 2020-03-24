#!/bin/bash
RCS_DIR=$(cd `dirname $0` && pwd)
find $RCS_DIR -name "*.before_astyle" -delete
