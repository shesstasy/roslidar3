#!/usr/bin/env python3

import rospy
import random
import numpy as np
from sensor_msgs.msg import LaserScan

class LidarScanSimulator:
    def __init__(self):
        rospy.init_node('lidar_simulator')
        self.pub = rospy.Publisher('/scan', LaserScan, queue_size=10)

        # это лидар
        self.num_beams = 720
        self.angle_min = -np.radians(90)
        self.angle_max = np.radians(90)
        self.angle_step = np.pi / 720
        self.min_range = 0.05
        self.max_range = 4.0

        # это создание стенки 
        self.wall_distance = 1.0
        self.wall_speed = 0.025
        self.wall_direction = 1

        self.rate = rospy.Rate(10)

    def calculate_scan(self) -> LaserScan:
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = 'laser'
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_step
        scan.range_min = self.min_range
        scan.range_max = self.max_range

        self.adjust_wall_distance()
        scan.ranges = self.generate_ranges()
        return scan

    def generate_ranges(self):
        return [
            self.calculate_wall_range(angle)
            for angle in [
                self.angle_min + i * self.angle_step
                for i in range(self.num_beams)
            ]
        ]

    def adjust_wall_distance(self):
        self.wall_distance += self.wall_speed * self.wall_direction
        if self.wall_distance < 0.5:
            self.wall_direction = 1
            rospy.loginfo("Стена приближается")
        elif self.wall_distance > 3.0:
            self.wall_direction = -1
            rospy.loginfo("Стена отдаляется")
     #углы
    def calculate_wall_range(self, angle: float) -> float:
        if abs(angle) < np.radians(10):
            return self.wall_distance + random.uniform(-0.005, 0.005)
        return self.max_range

    def run(self):
        while not rospy.is_shutdown():
            self.pub.publish(self.calculate_scan())
            self.rate.sleep()

def main() -> None:
    simulator = LidarScanSimulator()
    simulator.run()

if __name__ == '__main__':
    main()
