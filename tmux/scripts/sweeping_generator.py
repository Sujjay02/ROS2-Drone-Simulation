#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import numpy

from mrs_msgs.msg import ControlManagerDiagnostics, Reference
from mrs_msgs.srv import PathSrv
from mrs_msgs.srv import Vec1

class SweepingGeneratorNode(Node):

    def __init__(self):
        # 1. Initialize the node
        super().__init__('sweeping_generator')

        # 2. Declare and load parameters (ROS 2 requires declaration)
        self.declare_parameter('frame_id', 'uav1/gps_origin')
        self.declare_parameter('center.x', 0.0)
        self.declare_parameter('center.y', 0.0)
        self.declare_parameter('center.z', 3.0)
        self.declare_parameter('dimensions.x', 10.0)
        self.declare_parameter('dimensions.y', 10.0)
        self.declare_parameter('timer_main.rate', 1.0)

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.center_x = self.get_parameter('center.x').get_parameter_value().double_value
        self.center_y = self.get_parameter('center.y').get_parameter_value().double_value
        self.center_z = self.get_parameter('center.z').get_parameter_value().double_value
        self.dimensions_x = self.get_parameter('dimensions.x').get_parameter_value().double_value
        self.dimensions_y = self.get_parameter('dimensions.y').get_parameter_value().double_value
        timer_main_rate = self.get_parameter('timer_main.rate').get_parameter_value().double_value
        
        self.sub_control_manager_diag_msg = None
        self.is_initialized = False

        # 3. Create subscribers
        self.sub_control_manager_diag = self.create_subscription(
            ControlManagerDiagnostics, '~/control_manager_diag_in', self.callback_control_manager_diagnostics, 10)

        # 4. Create service servers
        self.ss_start = self.create_service(Vec1, '~/start_in', self.callback_start)

        # 5. Create service clients
        self.sc_path = self.create_client(PathSrv, '~/path_out')

        # 6. Create timers
        self.timer_main = self.create_timer(1.0 / timer_main_rate, self.timer_main)

        self.get_logger().info('Initialized')
        self.is_initialized = True

    ## | ------------------------- methods ------------------------ |

    def plan_path(self, step_size):
        self.get_logger().info('Planning path')

        path_request = PathSrv.Request()
        path_request.path.header.frame_id = self.frame_id
        # In ROS 2, get the current time from the node's clock
        path_request.path.header.stamp = self.get_clock().now().to_msg()
        path_request.path.fly_now = True
        path_request.path.use_heading = True

        sign = 1.0
        # fill in the path with a sweeping pattern
        for i in numpy.arange(-self.dimensions_x / 2.0, self.dimensions_x / 2.0, step_size):
            for j in numpy.arange(-self.dimensions_y / 2.0, self.dimensions_y / 2.0, step_size):
                point = Reference()
                point.position.x = self.center_x + i
                point.position.y = self.center_y + j * sign
                point.position.z = self.center_z
                point.heading = 0.0
                path_request.path.points.append(point)

            sign *= -1.0 # More concise way to flip the sign

        return path_request

    ## | ------------------------ callbacks ----------------------- |

    def callback_control_manager_diagnostics(self, msg):
        if not self.is_initialized:
            return
        # Use a logger with the 'once' flag
        self.get_logger().info('Getting ControlManager diagnostics', once=True)
        self.sub_control_manager_diag_msg = msg

    def callback_start(self, request, response):
        if not self.is_initialized:
            response.success = False
            response.message = "Not initialized"
            return response

        step_size = request.goal
        path_request = self.plan_path(step_size)

        # Wait until the service is available
        if not self.sc_path.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Path service not available')
            response.success = False
            response.message = "Path service not available"
            return response

        # Service calls in ROS 2 are asynchronous
        future = self.sc_path.call_async(path_request)
        # For this example, we will block until the call is complete
        # A more advanced implementation might handle the future in a separate thread
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            service_response = future.result()
            if service_response.success:
                self.get_logger().info('Path set successfully')
                response.message = "Starting"
                response.success = True
            else:
                self.get_logger().warn(f'Path setting failed, message: {service_response.message}')
                response.message = service_response.message
                response.success = False
        else:
            self.get_logger().error('Service call failed')
            response.success = False
            response.message = "Service call failed"
        
        return response

    ## | ------------------------- timers ------------------------- |

    def timer_main(self):
        if not self.is_initialized:
            return

        self.get_logger().info('Main timer spinning', once=True)

        if isinstance(self.sub_control_manager_diag_msg, ControlManagerDiagnostics):
            if self.sub_control_manager_diag_msg.tracker_status.have_goal:
                self.get_logger().info('Tracker has goal')
            else:
                self.get_logger().info('Waiting for command')

def main(args=None):
    rclpy.init(args=args)
    try:
        sweeping_generator_node = SweepingGeneratorNode()
        rclpy.spin(sweeping_generator_node)
    except KeyboardInterrupt:
        pass
    finally:
        sweeping_generator_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
