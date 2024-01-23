#!/usr/bin/env python3
import rospy
import socket
from jsk_recognition_msgs.msg import BoundingBoxArray
from std_msgs.msg import Float32
import math

def send_udp_message(message, host, port):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.sendto(message.encode(), (host, port))

def callback(data, distance_publisher):
    for box in data.boxes:
        distance = math.sqrt(box.pose.position.x**2 + box.pose.position.y**2 + box.pose.position.z**2)
        
        # Extract position in XYZ coordinates
        position_x = box.pose.position.x
        position_y = box.pose.position.y
        position_z = box.pose.position.z
        
        rospy.loginfo("Clustering 2: ID = {}, Distance = {:.3f}, Position = ({:.3f}, {:.3f}, {:.3f})".format(box.label, distance, position_x, position_y, position_z))
        distance_publisher.publish(Float32(distance))

        # Send message via UDP
        udp_message = "Clustering 2: ID = {}, Distance = {:.3f}, Position = ({:.3f}, {:.3f}, {:.3f})".format(box.label, distance, position_x, position_y, position_z)
        send_udp_message(udp_message, '127.0.0.1', 20003)

def listener():
    rospy.init_node('my_vdis2_node', anonymous=True)
    distance_publisher = rospy.Publisher('/vdis2', Float32, queue_size=10)
    rospy.Subscriber("/obstacle_detector/jsk_bboxes", BoundingBoxArray, lambda data: callback(data, distance_publisher))
    rospy.spin()

if __name__ == '__main__':
    listener()
