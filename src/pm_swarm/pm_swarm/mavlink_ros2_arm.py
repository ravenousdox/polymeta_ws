#!/usr/bin/env python3 
 
import time 
import rclpy 
from rclpy.node import Node 
 
from std_msgs.msg import String 
from sensor_msgs.msg import BatteryState, Range 
from std_srvs.srv import Trigger 
from geometry_msgs.msg import PoseStamped 
 
from pymavlink import mavutil 
 
class MAVLinkROS2Arm(Node): 
    def __init__(self): 
        super().__init__('mavlink_ros2_arm') 
 
        self.declare_parameter('mavlink_endpoint', rclpy.Parameter.Type.STRING) 
        self.declare_parameter('heartbeat_topic', rclpy.Parameter.Type.STRING) 
        self.declare_parameter('battery_topic', rclpy.Parameter.Type.STRING) 
        self.declare_parameter('source_system', rclpy.Parameter.Type.INTEGER) 
        self.declare_parameter('source_component', rclpy.Parameter.Type.INTEGER) 
        self.declare_parameter('target_system', rclpy.Parameter.Type.INTEGER) 
        self.declare_parameter('target_component', rclpy.Parameter.Type.INTEGER) 
        self.declare_parameter('arm_service_name', rclpy.Parameter.Type.STRING) 
 
        endpoint = self.get_parameter('mavlink_endpoint').get_parameter_value().string_value 
        heartbeat_topic = self.get_parameter('heartbeat_topic').get_parameter_value().string_value 
        battery_topic = self.get_parameter('battery_topic').get_parameter_value().string_value 
        self.src_sys = self.get_parameter('source_system').get_parameter_value().integer_value 
        self.src_comp = self.get_parameter('source_component').get_parameter_value().integer_value 
        self.tgt_sys = self.get_parameter('target_system').get_parameter_value().integer_value 
        self.tgt_comp = self.get_parameter('target_component').get_parameter_value().integer_value 
        arm_service_name = self.get_parameter('arm_service_name').get_parameter_value().string_value 

        self.heartbeat_pub = self.create_publisher(String, heartbeat_topic, 10) 
        self.battery_pub = self.create_publisher(BatteryState, battery_topic, 10) 
        self.distance_pub = self.create_publisher(Range, f'/drone{self.tgt_sys}/distance_sensor', 10) 
        self.optical_flow_pub = self.create_publisher(String, f'/drone{self.tgt_sys}/optical_flow', 10) 
        self.sys_status_pub = self.create_publisher(String, f'/drone{self.tgt_sys}/sys_status', 10) 
        self.power_status_pub = self.create_publisher(String, f'/drone{self.tgt_sys}/power_status', 10) 
        self.local_pos_pub = self.create_publisher(PoseStamped, f'/drone{self.tgt_sys}/local_position_ned', 10) 
 
        self.arm_service = self.create_service(Trigger, arm_service_name, self.handle_arm_drone) 
 
        self.get_logger().info(f'[drone{self.tgt_sys}] Connecting to MAVLink at {endpoint}') 

        self.conn = mavutil.mavlink_connection( 
            endpoint, 
            dialect='ardupilotmega', 
            source_system=self.src_sys, 
            source_component=self.src_comp, 
            autoreconnect=True, 
            force_connected=True, 
            robust_parsing=True 
        ) 

        self.conn.target_system = self.tgt_sys 
        self.conn.target_component = self.tgt_comp 
 
        self.conn.mav.heartbeat_send( 
            mavutil.mavlink.MAV_TYPE_GCS, 
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 
            0, 0, 0 
        ) 
 
        self.get_logger().info(f'[drone{self.tgt_sys}] Waiting for HEARTBEAT...') 
        hb = self.conn.recv_match(type='HEARTBEAT', blocking=True, timeout=5) 

        if hb: 
            mode = mavutil.mode_string_v10(hb) 
            self.get_logger().info(f"[drone{self.tgt_sys}] Connected: sysid={hb.get_srcSystem()}, mode={mode}") 
            self.heartbeat_pub.publish(String(data=mode)) 
        else: 
            self.get_logger().error(f"[drone{self.tgt_sys}] No HEARTBEAT received. Check routing.") 
            return 
 
        self.get_logger().info(f"[drone{self.tgt_sys}] Waiting 3s to ensure autopilot is ready...") 
        time.sleep(3) 
        self.get_logger().info(f"[drone{self.tgt_sys}] Requesting message streams via SET_MESSAGE_INTERVAL...") 
        self.request_streaming_messages() 
        self.create_timer(0.1, self.poll_mavlink) 
 
    def request_streaming_messages(self): 
        def req(msg_id, hz): 
            interval_us = int(1e6 / hz) 
            try: 
                self.conn.mav.message_interval_send(msg_id, interval_us) 
                ack = self.conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=1) 
                if ack and ack.command == mavutil.mavlink.MAVLINK_MSG_ID_MESSAGE_INTERVAL: 
                    self.get_logger().info(f"[drone{self.tgt_sys}] Stream {msg_id} confirmed.") 
                else: 
                    self.get_logger().warn(f"[drone{self.tgt_sys}] No ACK for stream {msg_id}.") 
            except Exception as e: 
                self.get_logger().warn(f"[drone{self.tgt_sys}] Failed to request message {msg_id} at {hz} Hz: {e}") 
 
        req(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 1) 
        req(mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS, 1) 
        req(mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, 5) 
        req(mavutil.mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW, 5) 
        req(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1) 
        req(mavutil.mavlink.MAVLINK_MSG_ID_POWER_STATUS, 1) 
        req(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 5) 
 
    def poll_mavlink(self): 
        self.conn.recv_msg() 
        while True: 
            msg = self.conn.recv_match(type=None, blocking=False) 
            if not msg: 
                break 
 
            msg_type = msg.get_type() 
 
            if msg_type == 'HEARTBEAT': 
                mode = mavutil.mode_string_v10(msg) 
                self.get_logger().info(f"[drone{self.tgt_sys}] Mode: {mode}") 
                self.heartbeat_pub.publish(String(data=mode)) 
 
            elif msg_type == 'BATTERY_STATUS': 
                voltage = msg.voltages[0] / 1000.0 if msg.voltages and msg.voltages[0] != 65535 else 0.0 
                percent = msg.battery_remaining / 100.0 if msg.battery_remaining != -1 else 0.0 
                self.get_logger().info( 
                    f"[drone{self.tgt_sys}] Battery: {voltage:.2f}V, {percent:.0%} remaining" 
                ) 
                self.handle_battery_status(msg) 
 
            elif msg_type == 'DISTANCE_SENSOR': 
                self.get_logger().info( 
                    f"[drone{self.tgt_sys}] Distance: {msg.current_distance/100.0:.2f} m" 
                ) 
                self.handle_distance_sensor(msg) 
 
            elif msg_type == 'OPTICAL_FLOW': 
                self.get_logger().info( 
                    f"[drone{self.tgt_sys}] OpticalFlow: flow=({msg.flow_x},{msg.flow_y}), " 
                    f"dist={msg.ground_distance:.2f} m, quality={msg.quality}" 
                ) 
                self.handle_optical_flow(msg) 
 
            elif msg_type == 'SYS_STATUS': 
                self.get_logger().info( 
                    f"[drone{self.tgt_sys}] SYS_STATUS: Vbat={msg.voltage_battery/1000.0:.2f}V, " 
                    f"remain={msg.battery_remaining}%, load={msg.load/10.0:.1f}%" 
                ) 
                self.handle_sys_status(msg) 
 
            elif msg_type == 'POWER_STATUS': 
                self.get_logger().info( 
                    f"[drone{self.tgt_sys}] POWER: Vcc={msg.Vcc}mV, Vservo={msg.Vservo}mV" 
                ) 
                self.handle_power_status(msg) 
 
            elif msg_type == 'LOCAL_POSITION_NED': 
                self.get_logger().info( 
                    f"[drone{self.tgt_sys}] LocalPos: x={msg.x:.1f}, y={msg.y:.1f}, z={msg.z:.1f}" 
                ) 
                self.handle_local_position_ned(msg) 
 
    # The rest of the handler methods remain unchanged 
    def handle_battery_status(self, msg): 
        voltage = msg.voltages[0] / 1000.0 if msg.voltages and msg.voltages[0] != 65535 else 0.0 
        remaining = msg.battery_remaining / 100.0 if msg.battery_remaining != -1 else 0.0 
        bmsg = BatteryState() 
        bmsg.voltage = voltage 
        bmsg.percentage = remaining 
        bmsg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING 
        bmsg.present = True 
        self.battery_pub.publish(bmsg) 
 
    def handle_distance_sensor(self, msg): 
        rng = Range() 
        rng.header.stamp = self.get_clock().now().to_msg() 
        rng.header.frame_id = 'distance_sensor_link' 
        rng.min_range = msg.min_distance / 100.0 
        rng.max_range = msg.max_distance / 100.0 
        rng.range = msg.current_distance / 100.0 
        self.distance_pub.publish(rng) 
 
    def handle_optical_flow(self, msg): 
        self.optical_flow_pub.publish(String(data=str(msg))) 
 
    def handle_sys_status(self, msg): 
        self.sys_status_pub.publish(String(data=str(msg))) 
 
    def handle_power_status(self, msg): 
        self.power_status_pub.publish(String(data=str(msg))) 
 
    def handle_local_position_ned(self, msg): 
        pose = PoseStamped() 
        pose.header.stamp = self.get_clock().now().to_msg() 
        pose.header.frame_id = 'local_origin_ned' 
        pose.pose.position.x = msg.x 
        pose.pose.position.y = msg.y 
        pose.pose.position.z = msg.z 
        self.local_pos_pub.publish(pose) 
 
    def handle_arm_drone(self, request, response): 
        self.get_logger().info(f"[drone{self.tgt_sys}] Sending ARM command...") 
        self.conn.mav.command_long_send( 
            self.conn.target_system, 
            self.conn.target_component, 
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
            1, 1, 0, 0, 0, 0, 0, 0 
        ) 
        ack = self.conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=3) 
        if ack: 
            result = ack.result 
            response.success = result == mavutil.mavlink.MAV_RESULT_ACCEPTED 
            response.message = "ARM accepted." if response.success else f"ARM rejected. MAV_RESULT={result}" 
        else: 
            response.success = False 
            response.message = "No COMMAND_ACK received." 
        return response 
 
def main(args=None): 
    rclpy.init(args=args) 
    node = MAVLinkROS2Arm() 
    try: 
        rclpy.spin(node) 
    except KeyboardInterrupt: 
        pass 
    node.destroy_node() 
    rclpy.shutdown() 
    
if __name__ == '__main__': 