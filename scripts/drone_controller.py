#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration
from crazyflie_interfaces.srv import GoTo, Takeoff, Land
from ros2_aruco_interfaces.msg import ArucoMarkers
from icuas25_msgs.msg import TargetInfo
import time


class DroneControllerNode(Node):
    def __init__(self):
        super().__init__('drone_controller_node')
        
        # Service clients
        self.goto_client = self.create_client(GoTo, 'cf_1/go_to')
        self.takeoff_client = self.create_client(Takeoff, 'cf_1/takeoff')
        self.land_client = self.create_client(Land, 'cf_1/land')
        
        self.get_logger().info('Waiting for services...')
        if not self.goto_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('GoTo service not available, will retry when sending commands')
        if not self.takeoff_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('Takeoff service not available, will retry when sending commands')
        if not self.land_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('Land service not available, will retry when sending commands')
        
        # Subscriber for aruco markers
        self.aruco_subscriber = self.create_subscription(
            ArucoMarkers,
            'cf_1/aruco_markers',
            self.aruco_callback,
            10
        )
        
        # Publisher for target found
        self.target_publisher = self.create_publisher(
            TargetInfo,
            'cf_1/target_found',
            10
        )
        
        # Setpoints list: [x, y, z, yaw, time]
        # You can modify this list or load from a file
        self.setpoints = [
            [-4.0, -5.0, 2.0, 0.0, 20.0],
            [-3.0, -2.4, 2.0, 0.0, 20.0],
            [-4.0, -5.0, 2.0, 0.0, 10.0],
            [-5.0, -5.0, 1.0, 0.0, 5.0],
        ]
        
        # State management
        # States: 'IDLE', 'TAKEOFF', 'FLYING', 'LANDING', 'DONE'
        self.state = 'IDLE'
        self.current_setpoint_index = 0
        self.last_command_time = None
        self.takeoff_height = 1.0  # meters
        self.takeoff_duration = 5.0  # seconds
        self.land_height = 0.05  # meters (slightly above ground)
        self.land_duration = 5.0  # seconds
        
        # Create timer to manage state machine
        # Timer frequency: 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('Drone controller node started')
        
        # Start with takeoff
        self.send_takeoff_command()
    
    def timer_callback(self):
        """Timer callback to manage state machine and send GoTo commands"""
        if self.state == 'IDLE':
            # Waiting for takeoff to complete
            return
        elif self.state == 'TAKEOFF':
            # Takeoff in progress, will transition to FLYING in callback
            return
        elif self.state == 'FLYING':
            # Execute setpoints
            if self.current_setpoint_index >= len(self.setpoints):
                # All setpoints completed, start landing
                self.state = 'LANDING'
                self.send_land_command()
                return
            
            current_time = time.time()
            
            # Check if we need to send a new command
            if self.last_command_time is None:
                # Send first command
                self.send_goto_command(self.setpoints[self.current_setpoint_index])
                self.last_command_time = current_time
            else:
                # Check if current command duration has elapsed
                setpoint_time = self.setpoints[self.current_setpoint_index][4]
                elapsed_time = current_time - self.last_command_time
                
                if elapsed_time >= setpoint_time:
                    # Move to next setpoint
                    self.current_setpoint_index += 1
                    if self.current_setpoint_index < len(self.setpoints):
                        self.send_goto_command(self.setpoints[self.current_setpoint_index])
                        self.last_command_time = current_time
                    else:
                        # All setpoints completed, start landing
                        self.state = 'LANDING'
                        self.send_land_command()
        elif self.state == 'LANDING':
            # Landing in progress, will transition to DONE in callback
            return
        elif self.state == 'DONE':
            # Mission complete
            return
    
    def send_goto_command(self, setpoint):
        """Send GoTo command to cf_1"""
        x, y, z, yaw, duration_time = setpoint
        
        if not self.goto_client.service_is_ready():
            self.get_logger().warn('GoTo service not ready, skipping command')
            return
        
        request = GoTo.Request()
        request.group_mask = 0  # Typically 0 for single drone
        request.relative = False  # Absolute position
        request.goal = Point(x=float(x), y=float(y), z=float(z))
        request.yaw = float(yaw)  # yaw in degrees
        
        # Convert duration_time (seconds) to Duration message
        duration = Duration()
        duration.sec = int(duration_time)
        duration.nanosec = int((duration_time - int(duration_time)) * 1e9)
        request.duration = duration
        
        self.get_logger().info(
            f'Sending GoTo command: x={x:.2f}, y={y:.2f}, z={z:.2f}, '
            f'yaw={yaw:.2f}, duration={duration_time:.2f}s'
        )
        
        # Send async request
        future = self.goto_client.call_async(request)
        future.add_done_callback(self.goto_response_callback)
    
    def send_takeoff_command(self):
        """Send takeoff command to cf_1"""
        if not self.takeoff_client.service_is_ready():
            self.get_logger().warn('Takeoff service not ready, will retry')
            return
        
        request = Takeoff.Request()
        request.group_mask = 0
        request.height = float(self.takeoff_height)
        
        # Convert duration to Duration message
        duration = Duration()
        duration.sec = int(self.takeoff_duration)
        duration.nanosec = int((self.takeoff_duration - int(self.takeoff_duration)) * 1e9)
        request.duration = duration
        
        self.get_logger().info(
            f'Sending takeoff command: height={self.takeoff_height:.2f}m, '
            f'duration={self.takeoff_duration:.2f}s'
        )
        
        self.state = 'TAKEOFF'
        future = self.takeoff_client.call_async(request)
        future.add_done_callback(self.takeoff_response_callback)
    
    def takeoff_response_callback(self, future):
        """Callback for takeoff service response"""
        try:
            response = future.result()
            self.get_logger().info('Takeoff command sent successfully')
            # Wait a bit for takeoff to complete, then start flying
            self.state = 'FLYING'
            self.get_logger().info('Starting setpoint sequence')
        except Exception as e:
            self.get_logger().error(f'Failed to send takeoff command: {e}')
            self.state = 'IDLE'
    
    def send_land_command(self):
        """Send land command to cf_1"""
        if not self.land_client.service_is_ready():
            self.get_logger().warn('Land service not ready, will retry')
            return
        
        request = Land.Request()
        request.group_mask = 0
        request.height = float(self.land_height)
        
        # Convert duration to Duration message
        duration = Duration()
        duration.sec = int(self.land_duration)
        duration.nanosec = int((self.land_duration - int(self.land_duration)) * 1e9)
        request.duration = duration
        
        self.get_logger().info(
            f'Sending land command: height={self.land_height:.2f}m, '
            f'duration={self.land_duration:.2f}s'
        )
        
        future = self.land_client.call_async(request)
        future.add_done_callback(self.land_response_callback)
    
    def land_response_callback(self, future):
        """Callback for land service response"""
        try:
            response = future.result()
            self.get_logger().info('Land command sent successfully')
            self.state = 'DONE'
            self.get_logger().info('Mission completed - drone landing')
        except Exception as e:
            self.get_logger().error(f'Failed to send land command: {e}')
            self.state = 'DONE'
    
    def goto_response_callback(self, future):
        """Callback for GoTo service response"""
        try:
            response = future.result()
            self.get_logger().info('GoTo command sent successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to send GoTo command: {e}')
    
    def aruco_callback(self, msg):
        """Callback for aruco markers subscription"""
        # Extract marker IDs and poses
        marker_ids = msg.marker_ids
        poses = msg.poses
        
        # Publish each marker as a TargetInfo message
        for i, marker_id in enumerate(marker_ids):
            if i < len(poses):
                pose = poses[i]
                # Extract position from pose
                position = pose.position
                
                # Create TargetInfo message
                target_info = TargetInfo()
                target_info.id = marker_id
                target_info.location = Point(
                    x=position.x,
                    y=position.y,
                    z=position.z
                )
                
                # Publish to /target_found topic
                self.target_publisher.publish(target_info)
                self.get_logger().info(
                    f'Published target: id={marker_id}, '
                    f'location=({position.x:.2f}, {position.y:.2f}, {position.z:.2f})'
                )


def main(args=None):
    rclpy.init(args=args)
    node = DroneControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

