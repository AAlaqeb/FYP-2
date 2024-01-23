#!/usr/bin/env python3
import rospy
import socket
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Float32
import math
from geometry_msgs.msg import Point

def calculate_distance(point):
    return math.sqrt(point.x**2 + point.y**2 + point.z**2)

def send_udp_message(message, host, port):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.sendto(message.encode(), (host, port))

def callback(data, distance_publisher):
    for marker in data.markers:
        if not marker.points:
            continue
        
        centroid = Point(
            sum(point.x for point in marker.points) / len(marker.points),
            sum(point.y for point in marker.points) / len(marker.points),
            sum(point.z for point in marker.points) / len(marker.points)
        )

        distance = calculate_distance(centroid)
        if distance >= 2.0:
            formatted_distance = "{:.3f}".format(distance)
            formatted_centroid_x = "{:.3f}".format(centroid.x)
            formatted_centroid_y = "{:.3f}".format(centroid.y)
            formatted_centroid_z = "{:.3f}".format(centroid.z)

            rospy.loginfo(f"Clustering 1: ID = {marker.id}, Distance = {formatted_distance}, Position = ({formatted_centroid_x}, {formatted_centroid_y}, {formatted_centroid_z})")
            distance_publisher.publish(Float32(distance))
            
            udp_message = f"Clustering 1: ID = {marker.id}, Distance = {formatted_distance}, Position = ({formatted_centroid_x}, {formatted_centroid_y}, {formatted_centroid_z})"
            send_udp_message(udp_message, '127.0.0.1', 20002)


def listener():
    rospy.init_node('distance_calculator', anonymous=True)
    distance_publisher = rospy.Publisher('/vdis1', Float32, queue_size=10)
    rospy.Subscriber("/adaptive_clustering/markers", MarkerArray, lambda data: callback(data, distance_publisher))
    rospy.spin()

if __name__ == '__main__':
    listener()
