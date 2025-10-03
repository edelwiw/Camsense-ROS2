import rclpy
from rclpy.node import Node
import serial 
import math
import struct 
import threading

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

class CamsenseNode(Node):

    def __init__(self):
        super().__init__("camsense_node")
        self.declare_parameter("port", "/dev/ttyAMA0")
        self.declare_parameter("topic", "/laser_scan")
        self.declare_parameter("frame_id", "laser_link")
        self.declare_parameter("range_min", 0.0)
        self.declare_parameter("range_max", 8.0)

        self.angle_min = 0.0
        self.angle_max = 2.0 * math.pi
        self.range_min = self.get_parameter("range_min").get_parameter_value().double_value
        self.range_max = self.get_parameter("range_max").get_parameter_value().double_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        
        port = self.get_parameter("port").get_parameter_value().string_value
        topic = self.get_parameter("topic").get_parameter_value().string_value
        
        self.publisher = self.create_publisher(LaserScan, topic, 10)
        self.ser = serial.Serial(port=port, baudrate=115200)

        self.num_points = 400
        self.points_buffer = [0] * self.num_points # in average 50 data packets per rotation, 8 point in each (in average 5 rpm)
        self.intensities_buffer = [0] * self.num_points 
        
        self.thread = threading.Thread(target=self.listen_port)
        self.thread.daemon = True
        self.thread.start()
        self.angle_offset = 15
        
        self.last_scan_time = self.get_clock().now()
        
        # send message with timer 
        self.timer = self.create_timer(1/60, self.publish_message) # 60 Hz

        
    def listen_port(self):
        prev_byte = 0 
        while True: 
            cur_byte = struct.unpack("B", self.ser.read(1))[0]
            if prev_byte == 0x55 and cur_byte == 0xAA: # check header matching 
                self.ser.read(2) # skip 2 bytes 
                speed = struct.unpack("H", self.ser.read(2))[0] / 64 # rpm
                start_bytes = struct.unpack("BB", self.ser.read(2))
                start_angle = (start_bytes[1] - 0xA0 + start_bytes[0] / 265) * 4 + self.angle_offset
                distances = []
                intensities = []
                for i in range(8):
                    distances.append(struct.unpack("H", self.ser.read(2))[0] / 1000.0) 
                    intensities.append(struct.unpack("B", self.ser.read(1))[0])

                end_bytes = struct.unpack("BB", self.ser.read(2))
                end_angle = (end_bytes[1] - 0xA0 + end_bytes[0] / 265) * 4 + self.angle_offset
                
                crc = struct.unpack("H", self.ser.read(2))[0]
                if (end_angle < start_angle): end_angle += 360
                angle_increment = (end_angle - start_angle) / 7
                angles = [2 * math.pi - (start_angle + angle_increment * i) for i in range(8)]
                
                self.find_best_positions(angles, distances, intensities)
                # if end_angle >= 360: 
                    # self.publish_message()
                
            prev_byte = cur_byte
            
    # Estimation of best ROS2 point to write current distance 
    # Actual angle could be not in preinited angles, it's impossible to use it in ROS2 
    def find_best_positions(self, angles, distances, intensities):
        for i in range(len(angles)): 
            actual_angle = angles[i] % 360.0
            nearest_angle_index = min(round(actual_angle * self.num_points / 360.0), 399)
            self.points_buffer[nearest_angle_index] = distances[i]
            self.intensities_buffer[nearest_angle_index] = intensities[i]


    def publish_message(self):      
        scan = LaserScan()
        scan.header = Header()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id
        
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = 2.0 * math.pi / self.num_points 
        scan.time_increment = 1.0 / (400 * 5) # in average 5 rpm, 400 points per rotation
        scan.scan_time = (self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9
        self.last_scan_time = self.get_clock().now()
        
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        
        scan.ranges = self.points_buffer
        scan.intensities = self.intensities_buffer

        self.publisher.publish(scan)
        
    def cleanup(self):
        self.timer.reset()
        self.thread.join()
    

def main(args=None):
    rclpy.init(args=args)

    camsense_x1_node = CamsenseNode()

    try:
        rclpy.spin(camsense_x1_node)
    finally: 
        camsense_x1_node.cleanup()

    camsense_x1_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()