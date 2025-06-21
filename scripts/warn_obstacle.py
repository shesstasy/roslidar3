#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

THRESHOLD_DISTANCE = 0.5 
class WarnObstacale:

    def __init__(self):
        rospy.init_node('proximity_warning', anonymous=True)
        rospy.Subscriber('/scan', LaserScan, self.check_scan)
        rospy.loginfo("Запуcк узла обнаружения препятствий")

    def check_scan(self, scan_data):
        for distance in scan_data.ranges:
            # Пропускаем некорректные значения (inf, nan)
            if np.isfinite(distance) and distance < THRESHOLD_DISTANCE:
                rospy.logwarn(f"Предупреждение! Достигнуто пороговое значение на расстоянии {distance:.2f} м")

def main():
    alert = WarnObstacale()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Узел остановлен")
