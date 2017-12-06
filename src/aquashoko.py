#!/usr/bin/env python
# /* -*-  iopyright (c) 2017, Daniel M. Lofaro
# Copyright (c) 2017, Alberto Perez Nunez
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# /*
# Copyright (c) 2017, Daniel M. Lofaro
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from ctypes import Structure,c_uint16,c_double,c_ubyte,c_uint32,c_int32,c_int16# */
import ach



AQUASHOKO_LEG_NUM = 4
AQUASHOKO_LEG_JOINT_NUM = 3
AQUASHOKO_CHAN_REF_NAME = "aquashoko_chan_ref"

class AQUASHOKO_JOINT(Structure):
  _pack_   = 1
  _fields_ = [
              ("ref", c_double),
              ("pos", c_double)]

class AQUASHOKO_LEG(Structure):
  _pack_   = 1
  _fields_ = [
              ("joint", AQUASHOKO_JOINT*AQUASHOKO_LEG_JOINT_NUM)]

class AQUASHOKO_REF(Structure):
  _pack_   = 1
  _fields_ = [
              ("leg", AQUASHOKO_LEG*AQUASHOKO_LEG_NUM)]



aquashoko_ref = AQUASHOKO_REF()
aquashoko_chan_ref = None


def aquashoko_init():
  global aquashoko_ref, aquashoko_chan_ref
  aquashoko_chan_ref = ach.Channel(AQUASHOKO_CHAN_REF_NAME)
  aquashoko_ref = AQUASHOKO_REF()
  return 0     

def aquashoko_get(leg, joint):
  global aquashoko_ref, aquashoko_chan_ref
  return aquashoko_ref.leg[leg].joint[joint].ref

def aquashoko_set(leg, joint, value):
  global aquashoko_ref, aquashoko_chan_ref
  aquashoko_ref.leg[leg].joint[joint].ref = value
  return 0

def aquashoko_put():
  global aquashoko_ref, aquashoko_chan_ref
  aquashoko_chan_ref.put(aquashoko_ref)
  return 0


def aquashoko_pull():
  global aquashoko_ref, aquashoko_chan_ref
  aquashoko_chan_ref.get(aquashoko_ref, wait=False, last=True)
  return 0
