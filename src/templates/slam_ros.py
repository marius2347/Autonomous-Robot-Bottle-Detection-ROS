#!/usr/bin/env python3
import rospy
import serial
import struct
import math
import numpy as np
import threading
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header

class LidarMapper:
    def __init__(self):
        # map parameters
        self.MAP_WIDTH = 400        # horizontal cells
        self.MAP_HEIGHT = 400       # vertical cells
        self.RESOLUTION = 0.05      # size cell in meters
        self.ORIGIN_X = - (self.MAP_WIDTH * self.RESOLUTION) / 2.0  # centering
        self.ORIGIN_Y = - (self.MAP_HEIGHT * self.RESOLUTION) / 2.0
        
        # grill initiazation -1 = unknown, 0 = free, 100 = busy
        self.grid = -1 * np.ones((self.MAP_HEIGHT, self.MAP_WIDTH), dtype=np.int8)
        
        # Publisher grid
        self.map_pub = rospy.Publisher("map", OccupancyGrid, queue_size=1)
        # Publisher laserScan
        self.scan_pub = rospy.Publisher("scan", LaserScan, queue_size=10)
        # subscribe to /map for creating the map
        rospy.Subscriber("scan", LaserScan, self.scan_callback)
        
        # open the port
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)
        except serial.SerialException as e:
            rospy.logerr("Eroare la deschiderea portului serial: %s", e)
            rospy.signal_shutdown("Serial error")
        
        # publish rate
        self.scan_rate = rospy.Rate(10)  # 10 Hz
        
        # timer for map
        rospy.Timer(rospy.Duration(1.0), self.publish_map)

    def read_scan(self):
        
        bytes_to_read = 360 * 4
        raw_data = self.ser.read(bytes_to_read)
        if len(raw_data) != bytes_to_read:
            return None
        try:
            distances = struct.unpack('<360f', raw_data)
            return distances
        except struct.error:
            return None

    def publish_scan(self):
        # config LaserScan
        scan_msg = LaserScan()
        scan_msg.header.frame_id = 'laser'  
        scan_msg.angle_min = 0.0
        scan_msg.angle_max = 2 * math.pi
        scan_msg.angle_increment = (2 * math.pi) / 360.0
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.0
        scan_msg.range_max = 10.0 

        rospy.loginfo("Începem publicarea datelor de la LiDAR...")
        while not rospy.is_shutdown():
            distances = self.read_scan()
            if distances is None:
                rospy.logwarn("Scanare incompletă sau eroare la parsare!")
                continue
            scan_msg.header.stamp = rospy.Time.now()
            scan_msg.ranges = list(distances)
            self.scan_pub.publish(scan_msg)
            self.scan_rate.sleep()

    def bresenham(self, x0, y0, x1, y1):
        """Algorithm Bresenham"""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        points.append((x1, y1))
        return points

    def world_to_map(self, x, y):
        map_x = int((x - self.ORIGIN_X) / self.RESOLUTION)
        map_y = int((y - self.ORIGIN_Y) / self.RESOLUTION)
        return map_x, map_y

    def scan_callback(self, scan):
        
        robot_map_x, robot_map_y = self.world_to_map(0.0, 0.0)
        
        angle = scan.angle_min
        for r in scan.ranges:
            # check for the right measurement
            if math.isnan(r) or r < scan.range_min or r > scan.range_max:
                angle += scan.angle_increment
                continue

            
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            map_x, map_y = self.world_to_map(x, y)
            
            line_points = self.bresenham(robot_map_x, robot_map_y, map_x, map_y)
            for pt in line_points[:-1]:
                grid_x, grid_y = pt
                if 0 <= grid_x < self.MAP_WIDTH and 0 <= grid_y < self.MAP_HEIGHT:
                    self.grid[grid_y, grid_x] = 0
            if 0 <= map_x < self.MAP_WIDTH and 0 <= map_y < self.MAP_HEIGHT:
                self.grid[map_y, map_x] = 100
            angle += scan.angle_increment

    def publish_map(self, event):
        map_msg = OccupancyGrid()
        map_msg.header.stamp = rospy.Time.now()
        map_msg.header.frame_id = "map"
        map_msg.info = MapMetaData()
        map_msg.info.resolution = self.RESOLUTION
        map_msg.info.width = self.MAP_WIDTH
        map_msg.info.height = self.MAP_HEIGHT
        map_msg.info.origin.position.x = self.ORIGIN_X
        map_msg.info.origin.position.y = self.ORIGIN_Y
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0
        map_msg.info.origin.orientation.x = 0.0
        map_msg.info.origin.orientation.y = 0.0
        map_msg.info.origin.orientation.z = 0.0
        map_msg.data = self.grid.flatten().tolist()
        self.map_pub.publish(map_msg)

if __name__ == '__main__':
    rospy.init_node('lidar_mapper', anonymous=True)
    lm = LidarMapper()
    scan_thread = threading.Thread(target=lm.publish_scan)
    scan_thread.daemon = True
    scan_thread.start()
    rospy.loginfo("Nodul LiDAR Mapper a pornit.")
    rospy.spin()
