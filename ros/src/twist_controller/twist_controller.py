import rospy

from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID
import numpy as np

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, *args, **kwargs):
        self.yaw_control = YawController(kwargs['wheel_base'], kwargs['steer_ratio'],
                                         kwargs['min_speed'], kwargs['max_lat_accel'],
                                         kwargs['max_steer_angle'])

        self.vehicle_mass    = kwargs["vehicle_mass"]
        self.brake_deadband  = kwargs["brake_deadband"]
        self.decel_limit     = kwargs["decel_limit"]
        self.accel_limit     = kwargs["accel_limit"]
        self.wheel_radius    = kwargs["wheel_radius"]
        self.max_steer_angle = kwargs["max_steer_angle"]
        self.updata_rate     = kwargs["updata_rate"]
        self.fuel_capacity   = kwargs["fuel_capacity"]

        # self.low_pass_filter = LowPassFilter(1.0, 1.0/self.updata_rate)
        
        #self.low_pass_brake  = LowPassFilter(0.25, 1.0/self.updata_rate)

        self.longitudinal_control = PID(12.0, 0.01, 0.5, self.decel_limit, self.accel_limit)

        self.last_run_ts = None

    def control(self, vx_cmd, vx_act, wz_cmd, dbw_enabled):
        if not dbw_enabled or self.last_run_ts is None:
            self.last_run_ts = rospy.get_time()
            self.longitudinal_control.reset()
            return 0.0, 0.0, 0.0

        # get time, and compute cycle time
        current_time = rospy.get_time()
        dt =  current_time - self.last_run_ts
        self.last_run_ts = current_time

        # compute longitudinal velocity error
        vx_err = vx_cmd - vx_act

        # update max change allowed in longitudinal velocity
        max_delta_vx = self.accel_limit*dt
        min_delta_vx = self.decel_limit*dt
        # and clamp the error with the limits
        vx_err = max(min(vx_err, max_delta_vx), min_delta_vx)

        # longitudinal PID control produces a control signal in [-1, 1]
        longitudinal_control = self.longitudinal_control.step(vx_err, dt)

        throttle = 0.0
        brake    = 0.0

        if longitudinal_control < 0:
            # brake_control = abs(longitudinal_control)*abs(self.decel_limit) - abs(self.brake_deadband)
            brake_control = abs(longitudinal_control) - abs(self.brake_deadband)
            if brake_control > 0.0:
                # direct brake torque = brake_force * wheel radius
                # brake_force = mass*deceleration
                # deceleration = v_err/dt
                brake_coeff = (self.vehicle_mass + self.fuel_capacity*GAS_DENSITY)*self.wheel_radius
                brake = brake_coeff*abs(brake_control)

        else:
            throttle = longitudinal_control

        #brake = self.low_pass_brake.filt(brake)

        steer = self.yaw_control.get_steering(vx_cmd, wz_cmd, vx_act)
        # steer = self.low_pass_filter.filt(steer)

        # clamp brake signal
        steer = max(min(steer, self.max_steer_angle), -self.max_steer_angle)

        return throttle, brake, steer
