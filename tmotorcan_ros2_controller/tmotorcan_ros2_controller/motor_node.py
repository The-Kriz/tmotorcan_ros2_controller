#motor_node.py
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time_source import TimeSource
from tmotorcan_ros2_controller.mit_can import TMotorManager_mit_can
from tmotorcan_ros2_msgs.msg import MitCommand
from tmotorcan_ros2_msgs.msg import MitState


import time
import numpy as np

class TMotorNode(Node):
    def __init__(self):
        super().__init__('tmotorcan_node', automatically_declare_parameters_from_overrides=False)

        #explicitly declare and set use_sim_time safely
        self.declare_parameter('use_sim_time', False)
        use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self._time_source = TimeSource()
        self._time_source.attach_node(self)

        self.get_logger().info(f"Starting TMotor ROS2 Controller Node (use_sim_time={use_sim_time})...")
        # Detect if parameters are avialble
        params_loaded = len(self.list_parameters([], depth=1).names) > 1

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
            topic_base = self.get_parameter_or(ns + '.topic_base') or f'motor_{i}'

            # Collect missing info
            if None in [motor_type, motor_id, kp, kd]:
                missing_params.append(ns)
                continue

            # Create TMotor CAN driver instance
            driver = TMotorManager_mit_can(motor_type, motor_id)
            driver.__enter__()

            pub = self.create_publisher(MitState, f'/{topic_base}/state', 10)
            sub = self.create_subscription(MitCommand, f'/{topic_base}/command',
                lambda msg, m_name=topic_base, m_driver=driver: self.cmd_callback(msg, m_name, m_driver),
                10
            )

            motor_cfg = {
                "serial": i,
                "type": motor_type,
                "id": motor_id,
                "kp": kp,
                "kd": kd,
                "topic_base": topic_base,
                "pub": pub,
                "driver": driver
            }
            self.motors.append(motor_cfg)
            self.get_logger().info(f"Loaded motor {i}: {motor_cfg}")

        if missing_params:
            if params_loaded or num_motors > 1:
                err_msg = (
                    f"Missing parameters for: {missing_params}. "
                    "Please check your YAML file and ensure all fields are defined."
                )
                self.get_logger().error(err_msg)
                raise RuntimeError(err_msg)
            else:
                self.get_logger().warn(
                    "No parameter file detected. Using default motor configuration."
                )
                default_driver = TMotorManager_mit_can("AK60-6", 1)
                default_driver.__enter__()
                pub = self.create_publisher(MitState, '/motor/state', 10)
                sub = self.create_subscription(
                    MitCommand,
                    '/motor/command',
                    lambda msg, m_name="motor", m_driver=default_driver: self.cmd_callback(msg, m_name, m_driver),
                    10
                )
                self.motors = [{
                    "serial": 1,
                    "type": "AK60-6",
                    "id": 1,
                    "kp": 50.0,
                    "kd": 1.0,
                    "topic_base": "motor",
                    "pub": pub,
                    "driver": default_driver
                }]

        # Timer for publishing feedback at 50 Hz
        self.timer = self.create_timer(0.02, self.publish_states)
        self.get_logger().info(f"Successfully loaded {len(self.motors)} motor configuration(s).")

    def get_parameter_or(self, name):
        """
        Try to get a parameter. if parameter is missing or fails, return None.
        """
        try:
            if not self.has_parameter(name):
                return None
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

    def cmd_callback(self, msg, motor_name, driver):
        """
        Handle incoming MIT command message and send it to the correct motor.
        """
        try:
            self.get_logger().debug(
                f"[{motor_name}] pos={msg.pos:.3f}, vel={msg.vel:.3f}, kp={msg.kp:.2f}, kd={msg.kd:.2f}, torque={msg.torque:.3f}"
            )

            # Send MIT-mode command to motor
            driver.set_impedance_gains_real_unit_full_state_feedback(K=msg.kp, B=msg.kd)
            driver.set_output_angle_radians(msg.pos)
            driver.set_output_velocity_radians_per_second(msg.vel)
            driver.set_output_torque_newton_meters(msg.torque)
            driver.update()

        except Exception as e:
            self.get_logger().error(f"Error sending command to {motor_name}: {e}")

    def publish_states(self):
        """
        Periodically publish state feedback for all motors.
        """
        for m in self.motors:
            driver = m["driver"]
            pub = m["pub"]
            msg = MitState()

            try:
                msg.position = driver.get_output_angle_radians()
                msg.velocity = driver.get_output_velocity_radians_per_second()
                msg.current = driver.get_current_qaxis_amps()
                msg.temperature = driver.get_temperature_celsius()
                msg.error = driver.get_motor_error_code()
                pub.publish(msg)
            except Exception as e:
                self.get_logger().warn(f"Failed to read state for {m['topic_base']}: {e}")

    def destroy_node(self):
        """
        Safely shut down the motors and stop CAN communication.
        """
        self.get_logger().info("Shutting down TMotor node...")
        for m in self.motors:
            try:
                m["driver"].__exit__(None, None, None)
            except Exception as e:
                self.get_logger().warn(f"Error during motor shutdown: {e}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TMotorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Node initialization failed: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()




if __name__ == '__main__':
    main()