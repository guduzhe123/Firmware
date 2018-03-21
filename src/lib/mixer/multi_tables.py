#!/usr/bin/env python
############################################################################
#
#   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

#
# Generate multirotor mixer scale tables compatible with the ArduCopter layout
#

# for python2.7 compatibility
from __future__ import  print_function

import math

print("/*")
print("* This file is automatically generated by multi_tables - do not edit.")
print("*/")
print("")
print("#ifndef _MIXER_MULTI_TABLES")
print("#define _MIXER_MULTI_TABLES")
print("")

def rcos(angleInRadians):
  return math.cos(math.radians(angleInRadians))

CCW = 1.0
CW = -CCW

quad_x = [
    [  45, CCW],
    [-135, CCW],
    [-45,  CW],
    [135,  CW],
]

quad_h = [
    [  45, CW],
    [-135, CW],
    [-45,  CCW],
    [135,  CCW],
]

quad_plus = [
    [  90, CCW],
    [ -90, CCW],
    [   0, CW],
    [ 180, CW],
]

quad_deadcat = [
    [  60, CCW, 1.0],
    [-125, CCW, 0.92],
    [ -60, CW, 1.0],
    [ 125, CW, 0.92],
]

quad_v = [
    [  18.8, 0.4242],
    [ -18.8, 1.0],
    [ -18.8, -0.4242],
    [  18.8, -1.0],
]

quad_wide = [
    [   68, CCW],
    [ -129, CCW],
    [  -68, CW],
    [  129, CW],
]

quad_s250aq = [
    [   59, CCW, 1.0 ],
    [ -139, CCW, 0.67],
    [  -59,  CW, 1.0 ],
    [  139,  CW, 0.67],
]

hex_x = [
    [  90, CW],
    [ -90, CCW],
    [ -30, CW],
    [ 150, CCW],
    [  30, CCW],
    [-150, CW],
]

hex_plus = [
    [   0, CW],
    [ 180, CCW],
    [-120, CW],
    [  60, CCW],
    [ -60, CCW],
    [ 120, CW],
]

hex_cox = [
    [  60, CW],
    [  60, CCW],
    [ 180, CW],
    [ 180, CCW],
    [ -60, CW],
    [ -60, CCW],
]

hex_t = [
    [  43.21, CCW],
    [  43.21, CW],
    [ 180, CW],
    [ 180, CCW],
    [ -43.21, CW],
    [ -43.21, CCW],
]

octa_x = [
    [  22.5, CW],
    [-157.5, CW],
    [  67.5, CCW],
    [ 157.5, CCW],
    [ -22.5, CCW],
    [-112.5, CCW],
    [ -67.5, CW],
    [ 112.5, CW],
]

octa_plus = [
    [   0, CW],
    [ 180, CW],
    [  45, CCW],
    [ 135, CCW],
    [ -45, CCW],
    [-135, CCW],
    [ -90, CW],
    [  90, CW],
]

octa_cox = [
    [  45, CCW],
    [ -45, CW],
    [-135, CCW],
    [ 135, CW],
    [ -45, CCW],
    [  45, CW],
    [ 135, CCW],
    [-135, CW],
]

octa_cox_wide = [
    [  68, CCW],
    [ -68, CW],
    [-129, CCW],
    [ 129, CW],
    [ -68, CCW],
    [  68, CW],
    [ 129, CCW],
    [-129, CW],
]

twin_engine = [
    [ 90, 0.0],
    [-90, 0.0],
]

tri_y = [
    [  60, 0.0],
    [ -60, 0.0],
    [ 180, 0.0],
]

dodeca_top_cox = [
    [  90, CW],
    [ -90, CCW],
    [ -30, CW],
    [ 150, CCW],
    [  30, CCW],
    [-150, CW],
]

dodeca_bottom_cox = [
    [  90, CCW],
    [ -90, CW],
    [ -30, CCW],
    [ 150, CW],
    [  30, CW],
    [-150, CCW],
]

hexa_dismotor_1 = [
    [ 0, 0, 0, 0],
    [ 1.7253, 0, 0, 1],
    [ 0.4313, 0.7471, 0, 1],
    [-0.4313, -0.7471, 0, 1],
    [-0.4313, 0.7471, 0, 1],
    [ 0.4313, -0.7471, 0, 1],
]

hexa_dismotor_2 = [
    [-1.7253, 0, 0, 1],
    [ 0, 0, 0, 0],
    [ 0.4313, 0.7471, 0, 1],
    [-0.4313, -0.7471, 0, 1],
    [-0.4313, 0.7471, 0, 1],
    [ 0.4313, -0.7471, 0, 1],
]

hexa_dismotor_3 = [
    [-0.8627, 0, 0, 1],
    [ 0.8627, 0, 0, 1],
    [ 0, 0, 0, 0],
    [-0.8627, -1.4942, 0, 1],
    [-0.4313, 0.7471, 0, 1],
    [ 0.4313, -0.7471, 0, 1],
]

hexa_dismotor_4 = [
    [-0.8627, 0, 0, 1],
    [ 0.8627, 0, 0, 1],
    [ 0.8627, 1.4942, 0, 1],
    [ 0, 0, 0, 0],
    [-0.4313, 0.7471, 0, 1],
    [ 0.4313, -0.7471, 0, 1],
]

# hexa_dismotor_5 = [
#     [-0.8627, 0, 0, 1],
#     [ 0.8627, 0, 0, 1],
#     [ 0.4313, 0.7471, 0, 1],
#     [-0.4313, -0.7471, 0, 1],
#     [ 0, 0, 0, 0],
#     [ 0.8627, -1.4942, 0, 1],
# ]

hexa_dismotor_5 = [
    [  90, CW,  170, 160, 1],
    [ -90, CCW, -170, 160, 1],
    [ -30, CW,  170, 160, 1],
    [ 150, CCW, -170, 160, 1],
    [  30, CCW, -170, 160, 1],
    [-150, CW,  170, 160, 1],
]

hexa_dismotor_6 = [
    [-0.8627, 0, 0, 1],
    [ 0.8627, 0, 0, 1],
    [ 0.4313, 0.7471, 0, 1],
    [-0.4313, -0.7471, 0, 1],
    [-0.8627, 1.4942, 0, 1],
    [ 0, 0, 0, 0],
]

# hex_tilt = [
#     [  90, CW,  15, 20, 1],
#     [ -90, CCW, -15, 20, 1],
#     [ -30, CW,  15, 20, 1],
#     [ 150, CCW, -15, 20, 1],
#     [  30, CCW, -30, 60, 1],
#     [-150, CW,  15, 20, 1],
# ]

hex_tilt = [
    [  90, CW,  15, 20, 1],
    [ -90, CCW, -15, 20, 1],
    [ -30, CW,  15, 20, 1],
    [ 150, CCW, -15, 20, 1],
    [  30, CCW, -15, 20, 1],
    [-150, CW,  15, 20, 1],
]

hex_tilt_s = [
    [  90, CW,  15, 20, 1],
    [ -90, CCW, -15, 20, 1],
    [ -30, CW,  15, 20, 1],
    [ 150, CCW, -15, 20, 1],
    [  30, CCW, -15, 20, 1],
    [-150, CW,  15, 20, 1],
]

hexa_degrade = [
    [ -1.000000,  0.000000, 0,  1.000000 ],
    [  1.000000,  0.000000, 0,  1.000000 ],
    [  0.500000,  0.866025, 0,  1.000000 ],
    [ -0.500000, -0.866025, 0,  1.000000 ],
    [ -0.500000,  0.866025, 0,  1.000000 ],
    [  0.500000, -0.866025, 0,  1.000000 ],
]

tables = [quad_x, quad_h, quad_plus, quad_v, quad_wide, quad_s250aq, quad_deadcat, hex_x, hex_plus, hex_cox, hex_t, octa_x, octa_plus, octa_cox, octa_cox_wide, twin_engine, tri_y, dodeca_top_cox, dodeca_bottom_cox]
# hexa_tables = [hexa_dismotor_1, hexa_dismotor_2, hexa_dismotor_3, hexa_dismotor_4, hexa_dismotor_5, hexa_dismotor_6,  hexa_degrade]
hexa_tables = [hexa_dismotor_1, hexa_dismotor_2, hexa_dismotor_3, hexa_dismotor_4, hexa_dismotor_6,  hexa_degrade]
hexa_tilt_tables = [hexa_dismotor_5, hex_tilt]

keys   = ["4x", "4h", "4+", "4v", "4w", "4s", "4dc",
          "6x", "6+", "6c", "6t",
          "8x", "8+", "8c", "8cw",
          "2-", "3y",
          "6m", "6a"]
# keys_hexa = ["6xm1", "6xm2", "6xm3", "6xm4", "6xm5", "6xm6", "6xDCS"]
keys_hexa = ["6xm1", "6xm2", "6xm3", "6xm4",  "6xm6", "6xDCS"]
keys_hexa_tilt = ["6xm5", "6ht"]

def variableName(variable):
    for variableName, value in list(globals().items()):
        if value is variable:
            return variableName

def unpackScales(scalesList):
    if len(scalesList) == 2:
        scalesList += [1.0] #Add thrust scale
    return scalesList

def printEnum():
    print("enum class MultirotorGeometry : MultirotorGeometryUnderlyingType {")
    for table in tables:
        print("\t{},".format(variableName(table).upper()))
    for table in hexa_tables:
        print("\t{},".format(variableName(table).upper()))
    for table in hexa_tilt_tables:
        print("\t{},".format(variableName(table).upper()))

    print("\t{},".format(variableName(hex_tilt_s).upper()))
    print("\n\tMAX_GEOMETRY")
    print("}; // enum class MultirotorGeometry\n")

def printScaleTables():
    for table in tables:
        print("const MultirotorMixer::Rotor _config_{}[] = {{".format(variableName(table)))
        for row in table:
            angle, yawScale, thrustScale = unpackScales(row)
            rollScale = rcos(angle + 90)
            pitchScale = rcos(angle)
            print("\t{{ {:9f}, {:9f}, {:9f}, {:9f} }},".format(rollScale, pitchScale, yawScale, thrustScale))
        print("};\n")

    for table in hexa_tables:
        print("const MultirotorMixer::Rotor _config_{}[] = {{".format(variableName(table)))
        for row in table:
            rollScale, pitchScale, yawScale, thrustScale = unpackScales(row)
            print("\t{{ {:9f}, {:9f}, {:9f}, {:9f} }},".format(rollScale, pitchScale, yawScale, thrustScale))
        print("};\n")

    # print("const MultirotorMixer::Rotor _config_{}[] = {{".format(variableName(hex_tilt)))

    for table in hexa_tilt_tables:
        print("const MultirotorMixer::Rotor _config_{}[] = {{".format(variableName(table)))
        for row in table:
            angle, yawScale, alpha, beta, thrustScale = unpackScales(row)
            rollScale =  rcos(angle + 90)* rcos(beta) * rcos(alpha) +   (rcos(angle + 90) * rcos(beta + 90) + rcos(angle) * rcos(alpha + 90) * rcos(beta)) * yawScale
            pitchScale =  rcos(angle) * rcos(beta) * rcos(alpha) +  (- rcos(angle) * rcos(beta + 90) + rcos(angle + 90) * rcos(alpha + 90) * rcos(beta)) * yawScale
            # rollScale =  rcos(angle + 90)* rcos(beta) * rcos(alpha) +  (- rcos(angle) * rcos(beta + 90) + rcos(angle + 90) * rcos(alpha + 90) * rcos(beta)) * yawScale
            # pitchScale =  rcos(angle) * rcos(beta) * rcos(alpha) +  (rcos(angle + 90) * rcos(beta + 90) + rcos(angle) * rcos(alpha + 90) * rcos(beta)) * yawScale
            rollScale /=  math.sqrt(rollScale * rollScale + pitchScale * pitchScale)
            pitchScale /= math.sqrt(rollScale * rollScale + pitchScale * pitchScale)
            yawScale = rcos(alpha + 90) * rcos(beta) + rcos(alpha) * rcos(beta) * yawScale
            thrustScale = thrustScale * rcos(beta) * rcos(alpha)
            print("\t{{ {:9f}, {:9f}, {:9f}, {:9f} }},".format(rollScale, pitchScale, yawScale, thrustScale))
        print("};\n")

    print("const MultirotorMixer::Rotor _config_{}[] = {{".format(variableName(hex_tilt_s)))
    for row in hex_tilt_s:
        angle, yawScale, alpha, beta, thrustScale = unpackScales(row)
        rollScale =  rcos(angle + 90)* rcos(beta) * rcos(alpha) + (rcos(angle + 90) * rcos(beta + 90) + rcos(angle) * rcos(alpha + 90) * rcos(beta)) * yawScale * rcos(60) + (- rcos(angle) * rcos(beta + 90) + rcos(angle + 90) * rcos(alpha + 90) * rcos(beta)) * yawScale * rcos(150)
        pitchScale = rcos(angle) * rcos(beta) * rcos(alpha) + (rcos(angle + 90) * rcos(beta + 90) - rcos(angle) * rcos(alpha + 90) * rcos(beta)) * yawScale * rcos(60 + 90) + (- rcos(angle) * rcos(beta + 90) + rcos(angle + 90) * rcos(alpha + 90) * rcos(beta)) * yawScale * rcos(60)
        rollScale /=  math.sqrt(rollScale * rollScale + pitchScale * pitchScale)
        pitchScale /= math.sqrt(rollScale * rollScale + pitchScale * pitchScale)
        yawScale = rcos(alpha + 90) * rcos(beta) + rcos(alpha) * rcos(beta) * yawScale
        thrustScale = thrustScale * rcos(beta) * rcos(alpha)
        print("\t{{ {:9f}, {:9f}, {:9f}, {:9f} }},".format(rollScale, pitchScale, yawScale, thrustScale))
    print("};\n")

def printScaleTablesIndex():
    print("const MultirotorMixer::Rotor *_config_index[] = {")
    for table in tables:
        print("\t&_config_{}[0],".format(variableName(table)))

    for table in hexa_tables:
        print("\t&_config_{}[0],".format(variableName(table)))

    for table in hexa_tilt_tables:
        print("\t&_config_{}[0],".format(variableName(table)))

    print("\t&_config_{}[0],".format(variableName(hex_tilt_s)))
    print("};\n")


def printScaleTablesCounts():
    print("const unsigned _config_rotor_count[] = {")
    for table in tables:
        print("\t{}, /* {} */".format(len(table), variableName(table)))
    for table in hexa_tables:
        print("\t{}, /* {} */".format(len(table), variableName(table)))
    for table in hexa_tilt_tables:
        print("\t{}, /* {} */".format(len(table), variableName(table)))
    print("\t{}, /* {} */".format(len(table), variableName(hex_tilt_s)))
    print("};\n")

def printScaleTablesKeys():
    print("const char* _config_key[] = {")
    for key, table in zip(keys, tables):
        print("\t\"{}\",\t/* {} */".format(key, variableName(table)))
    for key, table in zip(keys_hexa, hexa_tables):
        print("\t\"{}\",\t/* {} */".format(key, variableName(table)))
    for key, table in zip(keys_hexa_tilt, hexa_tilt_tables):
        print("\t\"{}\",\t/* {} */".format(key, variableName(table)))

    print("\t\"{}\",\t/* {} */".format("6hts",variableName(hex_tilt_s)))
    print("};\n")

printEnum()

print("namespace {")
printScaleTables()
printScaleTablesIndex()
printScaleTablesCounts()
printScaleTablesKeys()

print("} // anonymous namespace\n")
print("#endif /* _MIXER_MULTI_TABLES */")
print("")
