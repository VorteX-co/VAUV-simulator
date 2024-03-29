#!/usr/bin/env python3
# Copyright (c) 2020 The Plankton Authors.
# All rights reserved.
#
# This source code is derived from UUV Simulator
# (https://github.com/uuvsimulator/uuv_simulator)
# Copyright (c) 2016-2019 The UUV Simulator Authors
# licensed under the Apache license, Version 2.0
# cf. 3rd-party-licenses.txt file in the root directory of this source tree.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import rclpy
import numpy as np
import traceback

from utils.transform import get_world_ned_to_enu
from uuv_control_interfaces import DPControllerBase
from plankton_utils.time import time_in_float_sec
from plankton_utils.time import is_sim_time

class ROV_SFController(DPControllerBase):
    """
    Singularity-free tracking controller

    Reference
    ---------

    [1] O.-E. Fjellstad and T. I. Fossen, Singularity-free tracking of unmanned
        underwater vehicles in 6 DOF, Proceedings of 1994 33rd IEEE Conference
        on Decision and Control
    [2] G. Antonelli, "Underwater Robots," Springer Tracts in Advanced Robotics,
        2003
    """

    _LABEL = 'Singularity-free tracking controller'

    def __init__(self, node_name, **kwargs):
        DPControllerBase.__init__(self, node_name, True, **kwargs)
        self._tau = np.zeros(6)
        self._logger.info('Initializing: ' + self._LABEL)

        self._Kd = np.zeros(shape=(6, 6))

        if self.has_parameter('Kd'):
            coefs = self.get_parameter('Kd').value
            if len(coefs) == 6:
                self._Kd = np.diag(coefs)
            else:
                raise RuntimeError('Kd coefficients: 6 coefficients '
                                         'needed')

        self._logger.info('Kd=\n' + str(self._Kd))

        # Build delta matrix
        self._delta = np.zeros(shape=(6, 6))

        l = [0.0]
        if self.has_parameter('lambda'):
            l = self.get_parameter('lambda').value

        # Turn l into a list if it is not the case
        if type(l) is not list:
            l = [l]

        if len(l) == 1:
            self._delta[0:3, 0:3] = l[0] * np.eye(3)
        elif len(l) == 3:
            self._delta[0:3, 0:3] = np.diag(l)
        else:
            raise RuntimeError(
                'lambda: either a scalar or a 3 element vector must be provided')

        c = [0.0]
        if self.has_parameter('c'):
            c = self.get_parameter('c').value

        # Turn c into a list if it is not the case
        if type(c) is not list:
            c = [c]

        if len(c) == 1:
            self._delta[3:6, 3:6] = c[0] * np.eye(3)
        elif len(c) == 3:
            self._delta[3:6, 3:6] = np.diag(c)
        else:
            raise RuntimeError(
                'c: either a scalar or a 3 element vector must be provided')

        self._logger.info('delta=\n' + str(self._delta))

        self._prev_t = None
        self._prev_vel_r = np.zeros(6)

        self._is_init = True
        self._logger.info(self._LABEL + ' ready')

    # =========================================================================
    def update_controller(self):
        if not self._is_init:
            return False

        t = time_in_float_sec(self.get_clock().now())

        # Compute the generalized pose error vector using the quaternion
        # orientation error
        error = np.hstack((self._errors['pos'], self.error_orientation_quat))

        # Compute the virtual velocity signal
        vel_r = self._reference['vel'] + np.dot(self._delta, error)

        if self._prev_t is None:
            self._prev_t = t
            self._prev_vel_r = vel_r
            return False

        dt = t - self._prev_t

        if dt <= 0:
            return False

        # Compute virtual velocity error
        vel = self._vehicle_model.to_SNAME(self._vehicle_model.vel)
        # s = vel - self._vehicle_model.to_SNAME(vel_r)
        s = self._errors['vel'] + np.dot(self._delta, error)

        # Compute numerical derivative for virtual velocity signal
        d_vel_r = (vel_r - self._prev_vel_r) / dt

        sname_vel_r = self._vehicle_model.to_SNAME(vel_r)
        sname_d_vel_r = self._vehicle_model.to_SNAME(d_vel_r)
        self._vehicle_model._update_damping(vel)
        self._vehicle_model._update_coriolis(vel)
        self._vehicle_model._update_restoring(use_sname=True)

        self._tau = np.dot(self._vehicle_model.Mtotal, sname_d_vel_r) + \
            np.dot(self._vehicle_model.Ctotal, sname_vel_r) + \
            np.dot(self._vehicle_model.Dtotal, sname_vel_r) + \
            self._vehicle_model.restoring_forces

        # Update PID control action
        self.publish_control_wrench(
            self._vehicle_model.from_SNAME(self._tau) + np.dot(self._Kd, s))

        self._prev_t = t
        self._prev_vel_r = vel_r
        return True


# =============================================================================
def main():
    rclpy.init()

    try:
        sim_time_param = is_sim_time()

        tf_world_ned_to_enu = get_world_ned_to_enu(sim_time_param)

        node = ROV_SFController(
            'rov_sf_controller',
            world_ned_to_enu=tf_world_ned_to_enu,
            parameter_overrides=[sim_time_param])

        rclpy.spin(node)
    except Exception as e:
        print('Caught exception: ' + repr(e))
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    print('Exiting')


# =============================================================================
if __name__ == '__main__':
    main()
