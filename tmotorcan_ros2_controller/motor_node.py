import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from tmotorcan_ros2_controller.mit_can import TMotorManager_mit_can
import time
import numpy as np
import os


class TMotorNode(Node):
    def __init__(self):
        super().__init__('tmotorcan_node')

        # Detect if parameters are avialble
        params_loaded = len(self._parameters) > 0

        #Declare and fetch number of motors in the params file
        self.declare_parameter('num_motors', 1)
        num_motors = self.get_parameter('num_motors').get_parameter_value().integer_value

        self.motors = []
        missing_params = []

        # Load parameters per-motor 
        for i in range(1, num_motors + 1):
            ns = f"motor_{i}"

            # Try to fetch required parameters
            motor_type = self.get_parameter_or(ns + '.type')
            motor_id   = self.get_parameter_or(ns + '.id')
            kp         = self.get_parameter_or(ns + '.kp')
            kd         = self.get_parameter_or(ns + '.kd')

            # Collect missing info
            if None in [motor_type, motor_id, kp, kd]:
                missing_params.append(ns)
                continue

            motor_cfg = {
                "serial": i
                "type": motor_type,
                "id": motor_id,
                "kp": kp,
                "kd": kd
            }
            self.motors.append(motor_cfg)
            self.get_logger().info(f"Loaded motor {i}: {motor_cfg}")

        if missing_params:
            if params_loaded or num_motors > 1:
                err_msg = (
                    f"Missing parameters for: {missing_params}."
                    "Please check your YAML file and ensure all fields are defined."
                )
                self.get_logger().error(err_msg)
                raise RuntimeError(err_msg)
            else:
                self.get_logger().warn(
                    "No parameter file detected."
                    "Using default motor configuration (AK60-6, ID 1, KP 50.0, KD 1.0)."
                )
                self.motors = [{
                    "type": "AK60-6",
                    "id": 1,
                    "kp": 50.0,
                    "kd": 1.0
                }]

        self.get_logger().info(f"Successfully loaded {len(self.motors)} motor configuration(s).")

    def get_parameter_or(self, name):
        """
        Try to get a parameter. if parameter is missing or fails, return None.
        """
        try:
            if not self.has_parameter(name):
                return default
            p = self.get_parameter(name)
            v = p.get_parameter_value()

            if isinstance(v.string_value, str) and v.string_value:
                return v.string_value
            elif v.integer_value != 0:
                return v.integer_value
            elif v.double_value != 0.0:
                return v.double_value
            else :
                return None
        except Exception:
            return None
