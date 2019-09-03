# -*- coding: utf-8 -*-
#
# Copyright (c) 2017, Honda Research Institute Europe GmbH.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. All advertising materials mentioning features or use of this software
#    must display the following acknowledgement: This product includes
#    software developed by the Honda Research Institute Europe GmbH.
#
# 4. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER "AS IS" AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
# OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

# GDB pretty printers for Rcs structs.
# Allows to view MatNd values without extra effort.
# This has been derived from Eigen's pretty printers.

# To use it:
#
# * Create a directory and put the file as well as an empty __init__.py in 
#   that directory.
# * Create a ~/.gdbinit file, that contains the following:
#      python
#      import sys
#      sys.path.insert(0, '/path/to/rcs/printer/directory')
#      from rcs_printers import register_rcs_printers
#      register_rcs_printers (None)
#      end

import re
import gdb


class MatNdPrinter:
    "Print Rcs MatNd"
    
    def __init__(self, val):
        self.val = val
    
    def children(self):
        # yield entries
        rows = self.val['m']
        cols = self.val['n']
        for row in range(rows):
            for col in range(cols):
                item = self.val['ele'][row * cols + col] 
                if (cols == 1): #if it's a column vector
                    yield '[%d]' % (row,), item
                elif (rows == 1): #if it's a row vector
                    yield '[%d]' % (col,), item
                else:
                    yield '[%d,%d]' % (row, col), item

    def to_string(self):
        return "MatNd[%s,%s] (data ptr: %s)" % (self.val['m'], self.val['n'], self.val['ele'])


def register_rcs_printers(obj):
    "Register rcs pretty-printers with objfile Obj"

    if obj == None:
        obj = gdb
    obj.pretty_printers.append(rcs_printer_lookup)


def rcs_printer_lookup(val):
    "Look-up and return a pretty-printer that can print va."
    
    type = val.type
    
    if type.code == gdb.TYPE_CODE_REF:
        type = type.target()
    
    type = type.unqualified().strip_typedefs()
    
    typename = type.tag
    if typename == None:
        return None
    
    if typename == '_MatNd':
        return MatNdPrinter(val)
    
    return None

