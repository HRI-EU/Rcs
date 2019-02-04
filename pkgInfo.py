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

sqOptOutRules    = [ 'GEN04', 'C12', 'PY05', 'DOC04' ]

sqComments       = { 'C01': 'exits stop system / robot before getting into dangerous situations.',
                     'C10': 'Some of them are false positives, unclear how to eliminate the remaining ones',
                     'DOC04': 'Should not be public outside institute',
                     'GEN04': 'Software partially licensed under BSD licenses',
                     'PY05': 'No python code contained' }


# EOF
