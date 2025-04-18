#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, Image, Imu
from cv_bridge import CvBridge
import cv2

class SensorDataProcessor:
    def __init__(self):
        # Инициализация ROS-узла
        rospy.init_node('data_processor', anonymous=True)
        
        # Инициализация CV Bridge для работы с изображениями
        self.bridge = CvBridge()
        
        # Флаги активации датчиков
        self.lidar_active = rospy.get_param('~lidar_active', True)
        self.camera_active = rospy.get_param('~camera_active', True)
        self.imu_active = rospy.get_param('~imu_active', True)
        
        # Подписки на топики
        if self.lidar_active:
            rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback)
            self.pointcloud_data = None
            
        if self.camera_active:
            rospy.Subscriber('/camera/rgb/image_raw', Image, self.camera_callback)
            self.image_data = None
            # Окно для отображения изображения (если GUI включен)
            if rospy.get_param('~gui', False):
                cv2.namedWindow('Camera View', cv2.WINDOW_NORMAL)
        
        if self.imu_active:
            rospy.Subscriber('/imu/data_raw', Imu, self.imu_callback)
            self.imu_data = None
        
        # Публикация обработанных данных
        self.processed_data_pub = rospy.Publisher('/processed_data', Image, queue_size=10)
        
        # Таймер для основной обработки
        rospy.Timer(rospy.Duration(0.1), self.process_data)
        
        rospy.loginfo("Data Processor node initialized")

    def lidar_callback(self, data):
        """Обработка данных LIDAR"""
        self.pointcloud_data = data
        rospy.loginfo_once("Получены данные LIDAR")

    def camera_callback(self, data):
        """Обработка изображения с камеры"""
        try:
            self.image_data = self.bridge.imgmsg_to_cv2(data, "bgr8")
            if rospy.get_param('~gui', False):
                cv2.imshow('Camera View', self.image_data)
                cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Ошибка обработки изображения: {e}")

    def imu_callback(self, data):
        """Обработка данных IMU"""
        self.imu_data = data
        rospy.loginfo_once("Получены данные IMU")

    def process_data(self, event):
        """Основной цикл обработки данных"""
        if self.camera_active and self.image_data is not None:
            # Пример обработки изображения (детекция краев)
            processed_img = cv2.Canny(self.image_data, 100, 200)
            
            # Публикация результата
            try:
                img_msg = self.bridge.cv2_to_imgmsg(processed_img, "mono8")
                self.processed_data_pub.publish(img_msg)
            except Exception as e:
                rospy.logerr(f"Ошибка публикации изображения: {e}")

        if self.lidar_active and self.pointcloud_data:
            # Здесь могла бы быть обработка облака точек
            pass
            
        if self.imu_active and self.imu_data:
            # Здесь могла бы быть обработка IMU данных
            pass

    def cleanup(self):
        """Очистка ресурсов"""
        if rospy.get_param('~gui', False):
            cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        processor = SensorDataProcessor()
        rospy.on_shutdown(processor.cleanup)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
