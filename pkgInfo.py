# -*- coding: utf-8 -*-
#
#  Custom package settings
#
#  Copyright (C)
#  Honda Research Institute Europe GmbH
#  Carl-Legien-Str. 30
#  63073 Offenbach/Main
#  Germany
#
#  UNPUBLISHED PROPRIETARY MATERIAL.
#  ALL RIGHTS RESERVED.
#
#
name = "Rcs"

category="Libraries"

sqLevel = "basic"

sqOptOutRules    = [ 'GEN04', 'PY05' ]

sqComments       = { 'GEN04': 'Copyright header with BSD license not recognized by quality checker',
                     'PY05': 'No python code contained. ' }

SQ_12 = [ 'build/bionic64/bin/TestMath -m -1' ]




# EOF
