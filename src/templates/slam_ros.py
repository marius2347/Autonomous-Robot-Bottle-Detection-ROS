#!/usr/bin/env python3
import rospy
import serial
import math
from collections import deque
from sensor_msgs.msg import LaserScan


class Datapoint:
    def __init__(self, distance, signal, timestamp):
        self.angle = None
        self.distance = distance
        self.signal_strength = signal
        self.timestamp = timestamp

    def to_dict(self):
        return {
            "timestamp": self.timestamp,
            "angle": self.angle,
            "distance": self.distance,
            "signal": self.signal_strength,
        }


class Packet:
    def __init__(self, raw_data: bytearray):
        self.datapoints = []
        self.radar_speed = None
        self.start_angle = None
        self.end_angle = None
        self.timestamp = None
        self.complete = False
        self.raw_data = raw_data
        if raw_data[0] == 0x54 and raw_data[1] == 0x2C:
            self._decode()

    def _decode(self):
        self.radar_speed = int.from_bytes(self.raw_data[2:4], "little")
        self.start_angle = int.from_bytes(self.raw_data[4:6], "little") * 0.01
        self.end_angle = int.from_bytes(self.raw_data[42:44], "little") * 0.01
        self.timestamp = int.from_bytes(self.raw_data[44:46], "little")
        if self.timestamp > 30000:
            return
        for i in range(6, 42, 3):
            three_bytes = self.raw_data[i : i + 3]
            distance_mm = int.from_bytes(three_bytes[0:2], "little")
            signal = int.from_bytes(three_bytes[2:3], "little")
            self.datapoints.append(Datapoint(distance_mm, signal, self.timestamp))
        if len(self.datapoints) > 1:
            step = (self.end_angle - self.start_angle) / (len(self.datapoints) - 1)
        else:
            step = 0.0
        for i, d in enumerate(self.datapoints):
            d.angle = round(self.start_angle + step * i, 2)
        self.complete = True


class Circle:
    def __init__(self):
        self.packet_list = []

    def add(self, new_packet: Packet):
        if len(self.packet_list) == 0:
            self.packet_list.append(new_packet)
            return True
        last_angle = self.packet_list[-1].end_angle
        if new_packet.end_angle is not None and new_packet.end_angle > last_angle:
            self.packet_list.append(new_packet)
            return True
        return False

    def points(self):
        for packet in self.packet_list:
            for datapoint in packet.datapoints:
                yield datapoint


class Lidar:
    START_BYTE = b"\x54"
    VER_LEN = b"\x2c"


def read_one_circle(ser):
    circles = []
    current_circle = Circle()
    circles.append(current_circle)
    current_bytes = bytearray()
    last_two = deque([b"\x00", b"\x00"], maxlen=2)
    while len(circles) <= 2:
        x = ser.read()
        last_two.append(x)
        if last_two[0] == Lidar.START_BYTE and last_two[1] == Lidar.VER_LEN:
            if len(current_bytes) > 0:
                new_packet = Packet(current_bytes)
                if new_packet.complete:
                    if not current_circle.add(new_packet):
                        current_circle = Circle()
                        circles.append(current_circle)
                        current_circle.add(new_packet)
                current_bytes = bytearray()
                current_bytes += last_two[0]
        current_bytes += x
    return circles[1]


def main():
    rospy.init_node("d200_lidar_node")
    port = rospy.get_param("~port", "/dev/ttyUSB1")
    baud_rate = rospy.get_param("~baudrate", 230400)
    frame_id = rospy.get_param("~frame_id", "laser_link")
    pub_topic = rospy.get_param("~scan_topic", "/scan")
    pub_scan = rospy.Publisher(pub_topic, LaserScan, queue_size=1)
    ser = serial.Serial(port=port, baudrate=baud_rate)
    rospy.loginfo(f"D200 Lidar: opened {port} at baud {baud_rate}")

    while not rospy.is_shutdown():
        circle = read_one_circle(ser)
        points = list(circle.points())
        points.sort(key=lambda p: p.angle)
        if len(points) < 2:
            continue
        scan_msg = LaserScan()
        scan_msg.header.stamp = rospy.Time.now()
        scan_msg.header.frame_id = frame_id
        angle_min_rad = math.radians(points[0].angle)
        angle_max_rad = math.radians(points[-1].angle)
        scan_msg.angle_min = angle_min_rad
        scan_msg.angle_max = angle_max_rad
        num_points = len(points)
        scan_msg.angle_increment = (
            angle_max_rad - angle_min_rad
        ) / (num_points - 1) if num_points > 1 else 0.0
        scan_msg.range_min = 0.05
        scan_msg.range_max = 8.0
        scan_msg.ranges = [
            dp.distance / 1000.0 if dp.distance / 1000.0 >= 0.01 else float("inf")
            for dp in points
        ]
        scan_msg.intensities = [dp.signal_strength for dp in points]
        pub_scan.publish(scan_msg)
    ser.close()
    rospy.loginfo("D200 Lidar node stopped.")


if __name__ == "__main__":
    main()
