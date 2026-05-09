#!/usr/bin/env python3
"""
set_px4_params.py
=================
Sets all required PX4 parameters via MAVROS2 service calls.

Run ONCE after flashing PX4 firmware and before first flight.
Requires MAVROS2 to be running and connected to the Pixhawk.

Usage:
  # Terminal 1: start MAVROS2
  ros2 launch mavros px4.launch fcu_url:=serial:///dev/ttyUSB0:921600

  # Terminal 2: run this script
  python3 set_px4_params.py

Parameters are organised into groups. Review each group before running.
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import sys
import time

# =============================================================================
# Parameter definitions
# Format: (name, value, type)
#   type: 'int' | 'float'
# =============================================================================

PX4_PARAMS = [
    # ── VIO / External Vision ─────────────────────────────────────────────── #
    # EKF2_EV_CTRL bits: 0=hpos, 1=vpos, 2=vel, 3=yaw  → 15 = all enabled
    ('EKF2_EV_CTRL',     15,    'int'),
    ('EKF2_EV_DELAY',    50.0,  'float'),   # ms — tune with ekf2_innovations log
    ('EKF2_EV_NOISE_MD', 0,     'int'),     # 0 = use covariance from VIO message
    ('EKF2_HGT_REF',     3,     'int'),     # 3 = use EV (vision) for height ref

    # ── Disable GPS indoors ───────────────────────────────────────────────── #
    ('EKF2_GPS_CTRL',    0,     'int'),     # 0 = GPS disabled

    # ── Optical Flow (backup localisation at low altitude) ────────────────── #
    ('EKF2_OF_CTRL',     1,     'int'),     # enable optical flow fusion
    ('EKF2_RNG_CTRL',    1,     'int'),     # enable rangefinder
    ('EKF2_RNG_AID',     1,     'int'),     # range aid for low-altitude hover
    ('SENS_FLOW_MINHGT', 0.3,   'float'),
    ('SENS_FLOW_MAXHGT', 3.0,   'float'),

    # ── Collision Prevention ──────────────────────────────────────────────── #
    ('CP_DIST',          0.8,   'float'),   # m  — hard-stop clearance
    ('CP_DELAY',         0.4,   'float'),   # s  — sensor+actuator latency
    ('CP_GO_NO_DATA',    0,     'int'),     # 0 = STOP if data is lost

    # ── Flight speed limits (conservative for indoor testing) ────────────── #
    ('MPC_XY_VEL_MAX',   0.80,  'float'),   # m/s  max horizontal
    ('MPC_Z_VEL_MAX_UP', 0.50,  'float'),   # m/s  max climb
    ('MPC_LAND_SPEED',   0.30,  'float'),   # m/s  landing descent
    ('MPC_TKO_SPEED',    0.50,  'float'),   # m/s  takeoff climb

    # ── Position control mode (required for CP) ───────────────────────────── #
    ('MPC_POS_MODE',     0,     'int'),     # 0 = required for collision prevention

    # ── Failsafe ─────────────────────────────────────────────────────────── #
    # COM_RCL_EXCEPT: bit 2 = don't failsafe on RC loss in offboard mode
    ('COM_RCL_EXCEPT',   4,     'int'),
    # CBRK_IO_SAFETY: 22027 = bypass IO safety switch (if using Pixhawk 6X)
    # Only set if you have confirmed your safety setup. Comment out if unsure.
    # ('CBRK_IO_SAFETY', 22027, 'int'),
]


class PX4ParamSetter(Node):
    def __init__(self):
        super().__init__('px4_param_setter')
        self.client = self.create_client(
            SetParameters, '/mavros/param/set_parameters')

    def set_params(self):
        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                'MAVROS param service not available. '
                'Is MAVROS2 running and connected?')
            return False

        success_count = 0
        fail_count    = 0

        for name, value, ptype in PX4_PARAMS:
            param = Parameter()
            param.name = name

            pv = ParameterValue()
            if ptype == 'int':
                pv.type          = ParameterType.PARAMETER_INTEGER
                pv.integer_value = int(value)
            else:
                pv.type          = ParameterType.PARAMETER_DOUBLE
                pv.double_value  = float(value)

            param.value = pv

            req = SetParameters.Request()
            req.parameters = [param]

            future = self.client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            if future.result() and future.result().results[0].successful:
                self.get_logger().info(f'  ✓  {name} = {value}')
                success_count += 1
            else:
                reason = (future.result().results[0].reason
                          if future.result() else 'timeout')
                self.get_logger().error(f'  ✗  {name} = {value}  [{reason}]')
                fail_count += 1

            time.sleep(0.05)  # brief pause between param sets

        self.get_logger().info(
            f'\nDone: {success_count} set, {fail_count} failed. '
            f'Reboot Pixhawk to apply all changes.'
        )
        return fail_count == 0


def main():
    rclpy.init()
    node = PX4ParamSetter()
    print('\n=== Setting PX4 Parameters for Indoor VIO Flight ===\n')
    ok = node.set_params()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
