import rospy

from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID
import numpy as np

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MAX_THROTTLE = 1.0
MIN_THROTTLE = 0.001

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

        self.low_pass_filter = LowPassFilter(1.0, 1.0/self.updata_rate)
        
        #self.low_pass_brake  = LowPassFilter(0.25, 1.0/self.updata_rate)

        self.throttle_control = PID(12.5, 0.01, 1.0, -MAX_THROTTLE, MAX_THROTTLE)

        self.last_run_ts = None

        # self.debug_throttle_max = -1e9
        # self.debug_throttle_min =  1e9

        # self.debug_brake_max = -1e9
        # self.debug_brake_min =  1e9

        # self.debug_steering_max = -1e9
        # self.debug_steering_min =  1e9

        # self.debug_vxerr_max = -1e9
        # self.debug_vxerr_min =  1e9


    def control(self, vx_cmd, vx_act, wz_cmd, dbw_enabled):
        if not dbw_enabled or self.last_run_ts is None:
            self.last_run_ts = rospy.get_time()
            self.throttle_control.reset()
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
        longitudinal_control = self.throttle_control.step(vx_err, dt)

        throttle = 0.0
        brake    = 0.0

        if longitudinal_control < -abs(self.brake_deadband):
            brake_control = abs(longitudinal_control) - abs(self.brake_deadband)
            # direct brake torque = brake_force * wheel radius
            # brake_force = mass*deceleration
            # deceleration = v_err/dt
            brake_coeff = (self.vehicle_mass + self.fuel_capacity*GAS_DENSITY)*self.wheel_radius*abs(self.decel_limit)
            brake = brake_coeff*abs(brake_control)
            # if abs(longitudinal_control) < self.brake_deadband:
                # brake = 0.0
                
        else:
            throttle = longitudinal_control

        #brake = self.low_pass_brake.filt(brake)

        steer = self.yaw_control.get_steering(vx_cmd, wz_cmd, vx_act)
        #steer = self.low_pass_filter.filt(steer)

        # clamp brake signal
        steer = max(min(steer, self.max_steer_angle), -self.max_steer_angle)

        # self.debug_vxerr_min = min(self.debug_vxerr_min, vx_err)
        # self.debug_vxerr_max = max(self.debug_vxerr_max, vx_err)

        # if throttle > 1e-5:
        #     self.debug_throttle_min = min(self.debug_throttle_min, throttle)
        # self.debug_throttle_max = max(self.debug_throttle_max, throttle)

        # if brake > 1e-5:
        #     self.debug_brake_min = min(self.debug_brake_min, brake)
        # self.debug_brake_max = max(self.debug_brake_max, brake)

        # self.debug_steering_min = min(self.debug_steering_min, steer)
        # self.debug_steering_max = max(self.debug_steering_max, steer)

        # rospy.loginfo("vx_err: %s\tmax: %s\tmin: %s", vx_err, self.debug_vxerr_max, self.debug_vxerr_min)

        # rospy.loginfo("throttle: %s\tmax: %s\tmin: %s", throttle, self.debug_throttle_max, self.debug_throttle_min)
        # rospy.loginfo("brake: %s\tmax: %s\tmin: %s", brake, self.debug_brake_max, self.debug_brake_min)
        # rospy.loginfo("steer: %s\tmax: %s\tmin: %s", steer, self.debug_steering_max, self.debug_steering_min)
        return throttle, brake, steer
