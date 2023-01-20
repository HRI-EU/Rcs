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

function testMath()
{
    build/"${MAKEFILE_PLATFORM}"/bin/TestMath -m -1 -numTests 10 -dl 1 &> UnitTestResults.txt
    testMathResult=$?
}

function testGraph()
{
    build/"${MAKEFILE_PLATFORM}"/bin/Rcs -m 3 -iter 10 -dl 1 -valgrind &>> UnitTestResults.txt
    testGraphResult=$?
}

function testController()
{
    build/"${MAKEFILE_PLATFORM}"/bin/Rcs -m 6 -valgrind -nTests 10 -dl 1 &>> UnitTestResults.txt
    testControllerResult=$?
}

echo -n "Testing mathematics functions ... "
testMath

if [ "${testMathResult}" -eq 0 ]
then
  echo "succeeded"
elif [ "${testMathResult}" -eq 255 ]
then
  echo "failed with more than 255 errors"
else
  echo "failed with ${testMathResult} errors" 
fi



echo -n "Testing graph functions       ... "
testGraph

if [ "${testGraphResult}" -eq 0 ]
then
  echo "succeeded"
elif [ "${testGraphResult}" -eq 255 ]
then
  echo "failed with more than 255 errors"
else
  echo "failed with ${testGraphResult} errors" 
fi



echo -n "Testing controller functions  ... "
testController

if [ "${testControllerResult}" -eq 0 ]
then
  echo "succeeded"
elif [ "${testControllerResult}" -eq 255 ]
then
  echo "failed with more than 255 errors"
else
  echo "failed with ${testControllerResult} errors" 
fi
