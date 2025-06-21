#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float32

SECTOR_ANGLE = 30.0
DISTANCE_THRESHOLD = 0.02

class LidarMotionAnalyzer:

    def __init__(self):

        rospy.init_node('lidar_motion_tracker', anonymous=True)
        
        self.movement_pub = rospy.Publisher('/obstacle_movement', String, queue_size=10)
        self.speed_pub = rospy.Publisher('/obstacle_speed', Float32, queue_size=10)
        
        rospy.Subscriber('/scan', LaserScan, self.analyze_scan)
        
        self.last_distance = None
        
        rospy.loginfo("узел запущен")

    def analyze_scan(self, scan_data):
        start_time = rospy.get_time()
        sector_ranges = self.extract_sector(
            scan_data.ranges,
            scan_data.angle_min,
            scan_data.angle_max,
            scan_data.angle_increment
        )
        
        if not sector_ranges:
            rospy.logwarn("нет данных в секторе")
            return
        
        min_distance = min(sector_ranges)
        
        movement, delta = self.track_movement(min_distance)
        
        movement_msg = String()
        movement_msg.data = movement
        self.movement_pub.publish(movement_msg)
        
        elapsed_time = rospy.get_time() - start_time
        speed = abs(delta / elapsed_time) if elapsed_time > 0 else 0.0
        speed_msg = Float32()
        speed_msg.data = speed
        self.speed_pub.publish(speed_msg)
        
        rospy.loginfo(f"Движение: {movement}, cкорость: {speed:.2f} м/с")

    def extract_sector(self, ranges, angle_min, angle_max, angle_increment):
        sector_ranges = []
        sector_rad = np.radians(SECTOR_ANGLE)
        min_index = int((angle_max - sector_rad) / angle_increment)
        max_index = int((angle_max + sector_rad) / angle_increment)
        
        for i in range(min_index, max_index + 1):
            distance = ranges[i]
            if np.isfinite(distance):
                sector_ranges.append(distance)
        
        return sector_ranges

    def track_movement(self, current_distance):
        movement = "stable"
        delta = 0.0
        
        if self.last_distance is not None:
            delta = self.last_distance - current_distance
            if abs(delta) > DISTANCE_THRESHOLD:
                movement = "approaching" if delta > 0 else "receding"
        
        self.last_distance = current_distance
        return movement, delta

def main():
    analyzer = LidarMotionAnalyzer()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("остановка")
