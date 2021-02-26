# -*- coding: utf-8 -*-
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
name = "Rcs"

version = "1.0"

category = "Libraries"

sqLevel = "basic"

SQ_12 = [ 'build/${MAKEFILE_PLATFORM}/bin/TestMath -m -1 -numTests 10 -dl 1',
          'build/${MAKEFILE_PLATFORM}/bin/Rcs -m 3 -iter 10 -dl 1 -valgrind',
          'build/${MAKEFILE_PLATFORM}/bin/Rcs -m 6 -valgrind -nTests 10 -dl 1']

copyright        = { '.': 'Copyright (c) 2017, Honda Research Institute Europe GmbH.',
                     './src/RcsCore/SegFaultHandler.h': 'This code is in the public domain.' }

gitOriginForCIA  = 'git@dmz-gitlab.honda-ri.de:GeneralRobotics/Rcs.git'

gitBranchForCIA = 'develop'


# EOF
