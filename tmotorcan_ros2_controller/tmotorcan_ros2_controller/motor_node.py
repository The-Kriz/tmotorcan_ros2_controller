# motor_node.py
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from tmotorcan_ros2_controller.mit_can import TMotorManager_mit_can
from tmotorcan_ros2_msgs.msg import MitCommand, MitState

import time
import numpy as np

class MotorController(Node):
    def __init__(self):
        super().__init__("tmotorcan_controller")

        # Parameters 
        self.declare_parameter('num_motors', 1, ParameterDescriptor(
            description='Number of Motors used'))
        num_motors = int(self.get_parameter('num_motors').get_parameter_value().integer_value)
        self.get_logger().info(f"Number of motors: {num_motors}")

        self.declare_parameter('motor_type', 'AK60-6', ParameterDescriptor(
            description='Default motor model (if not set per-motor)'))
        self.declare_parameter('motor_kp', 50.0)
        self.declare_parameter('motor_kd', 1.0)

        # Setup motors 
        self.motors = []
        for i in range(num_motors):
            ns = f"motor_{i+1}"            # namespace prefix for motors
            self.declare_parameter(f"{ns}.type", self.get_parameter('motor_type').get_parameter_value().string_value)
            self.declare_parameter(f"{ns}.id", i + 1)
            self.declare_parameter(f"{ns}.kp", self.get_parameter('motor_kp').get_parameter_value().double_value)
            self.declare_parameter(f"{ns}.kd", self.get_parameter('motor_kd').get_parameter_value().double_value)
            self.declare_parameter(f"{ns}.topic_base", f"motor_{i+1}")
            self.declare_parameter(f"{ns}.name", f"motor_{i+1}")

            motor_type = self.get_parameter(f"{ns}.type").get_parameter_value().string_value
            motor_id   = int(self.get_parameter(f"{ns}.id").get_parameter_value().integer_value)
            kp         = float(self.get_parameter(f"{ns}.kp").get_parameter_value().double_value)
            kd         = float(self.get_parameter(f"{ns}.kd").get_parameter_value().double_value)
            topic_base = self.get_parameter(f"{ns}.topic_base").get_parameter_value().string_value
            name       = self.get_parameter(f"{ns}.name").get_parameter_value().string_value

            # create motor driver using TMotorManager
            try:
                driver = TMotorManager_mit_can(motor_type, motor_id)
                driver.__enter__()
            except Exception as e:
                self.get_logger().error(f"Failed to create driver for motor {name} (id={motor_id}): {e}")
                # skip motor if any error and continue with others (usefull for testing without all motor present)
                continue

            # publishers / subscriptions
            pub = self.create_publisher(MitState, f"/{topic_base}/state", 10)
            sub = self.create_subscription(MitCommand, f"/{topic_base}/command",
                lambda msg, m_name=topic_base, m_driver=driver: 
                self.command_callback(msg, m_name, m_driver), 10 )

            motor_cfg = {
                "name": name,
                "id": motor_id,
                "type": motor_type,
                "kp": kp,
                "kd": kd,
                "topic_base": topic_base,
                "pub": pub,
                "driver": driver
            }

            self.motors.append(motor_cfg)

            self.get_logger().info(f"Initialized motor '{name}' id={motor_id} type={motor_type}"
                f"Kp={kp} Kd={kd} topic=/{topic_base}")

        # Timer for publishing feedback 
        self.timer = self.create_timer(0.02, self.publish_states)
        self.get_logger().info(f"Controller ready for {len(self.motors)} motor(s).")

    def command_callback(self, msg, motor_id, driver):
        """Receive MIT-mode command and send to Motor."""
        try:
            driver.set_impedance_gains_real_unit_full_state_feedback(K=msg.kp, B=msg.kd)
            driver.set_output_angle_radians(msg.pos)
            driver.set_output_velocity_radians_per_second(msg.vel)
            driver.set_output_torque_newton_meters(msg.torque)
            driver.update()
        except Exception as e:
            self.get_logger().error(f"Error sending command to motor {motor_id}: {e}")

    def publish_states(self):
        """Read state from all motors and publish feedback."""
        for m in self.motors:
            msg = MitState()
            drv = m["driver"]
            try:
                msg.position = drv.get_output_angle_radians()
                msg.velocity = drv.get_output_velocity_radians_per_second()
                msg.current = drv.get_current_qaxis_amps()
                msg.temperature = drv.get_temperature_celsius()
                msg.error = drv.get_motor_error_code()
                m["pub"].publish(msg)
            except Exception as e:
                self.get_logger().warn(f"Read failed for motor {m['id']}: {e}")

    def destroy_node(self):
        """Safely shut down motors."""
        for m in self.motors:
            try:
                m["driver"].__exit__(None, None, None)
            except Exception as e:
                self.get_logger().warn(f"Error closing motor {m['id']}: {e}")
        super().destroy_node()


def main():
    rclpy.init()
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
