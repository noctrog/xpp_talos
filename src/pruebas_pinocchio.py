#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import pinocchio
import os
import numpy as np

# Get URDF filename
script_dir = os.path.dirname(os.path.realpath(__file__))
pinocchio_model_dir = os.path.join(script_dir, "../urdf")
urdf_filename = pinocchio_model_dir + "/talos_full_legs_v2.urdf"

# Load URDF model
model = pinocchio.buildModelFromUrdf(urdf_filename, pinocchio.JointModelFreeFlyer())
data = model.createData()
print('model name: ' + model.name)

# Se ponen las posiciones, velocidades, aceleraciones y fuerzas (en el
# marco de referencia de cada articulación) 
q = pinocchio.utils.zero(model.nq)
v = pinocchio.utils.zero(model.nv)
a = pinocchio.utils.zero(model.nv)
fext = pinocchio.StdVec_Force()
for _ in range(model.njoints):
    fext.append(pinocchio.Force.Zero())

# Valores de prueba
v[12] = 1
a[12] = 10

# Calcula las derivadas de la dinámica
pinocchio.computeABADerivatives(model, data, q, v, a, fext)

# Muestra las aceleraciones
print('ddq: ' + str(data.ddq))
# Muestra las derivadas de la aceleración articular con respecto a la q
print('ddq_dq: ' + str(data.ddq_dq))
# Muestra las derivadas de la aceleración articular con respecto a la velocidad articular
print('ddq_dv: ' + str(data.ddq_dv))
# Muestra las derivadas del torque con respecto a q
print('dtau_dq: ' + str(data.dtau_dq))
# Muestra las derivadas del torque con respecto a la velocidad articular
print('dtau_dv: ' + str(data.dtau_dv))
