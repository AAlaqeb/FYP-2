#!/usr/bin/env python3
import socket
import threading
import time
import numpy as np
import csv
from datetime import datetime, timedelta
from filterpy.kalman import KalmanFilter

# Initialize lists to store data
zed_data = []
clustering1_data = []
clustering2_data = []
data_lock = threading.Lock()

buffer_duration = timedelta(seconds=1)  # Buffer data for 1 second
last_processing_time = datetime.now()

def initialize_kalman_filter():
    kf = KalmanFilter(dim_x=1, dim_z=3)
    kf.x = np.array([0])  # initial state (initial distance)
    kf.F = np.array([[1]])  # state transition matrix
    kf.H = np.array([[1], [1], [1]])  # measurement function (3x1 matrix)
    kf.P = np.array([[1000]])  # covariance matrix
    kf.R = np.diag([1, 1, 1])  # measurement noise for each of the 3 measurements
    kf.Q = np.array([[0.1]])  # process noise
    return kf


def process_data_periodically(csv_writer, kf):
    global last_processing_time
    while True:
        time.sleep(0.1)  # Check every 100ms
        with data_lock:
            if datetime.now() - last_processing_time >= buffer_duration:
                if zed_data and clustering1_data and clustering2_data:
                    compare_and_merge(zed_data, clustering1_data, clustering2_data, csv_writer, kf)
                    print(f"Processing buffered data... Number of ZED objects detected: {len(zed_data)}")
                    zed_data.clear()
                    clustering1_data.clear()
                    clustering2_data.clear()
                    last_processing_time = datetime.now()

                    
def parse_zed_data(data_string):
    try:
        parts = data_string.split(', ')
        id_part = parts[0].split(' = ')[1]
        distance_part = parts[1].split(' = ')[1]
        position_part = parts[2].split(' = ')[1].strip('()').split(', ')

        return {
            'id': int(id_part),
            'distance': float(distance_part),
            'position': [float(x) for x in position_part]
        }
    except Exception as e:
        print(f"Error parsing ZED data '{data_string}': {e}")
        return None


def parse_clustering_data(data_string):
    try:
        # Splitting the string based on ', ' and then further splitting based on ' = '
        parts = data_string.split(', ')
        id_part = parts[0].split(' = ')[1]  # Split and take the second part for ID
        distance_part = parts[1].split(' = ')[1]  # Split and take the second part for Distance
        position_part = parts[2].split(' = ')[1].strip('()').split(', ')  # Split and process Position

        return {
            'id': int(id_part),
            'distance': float(distance_part),
            'position': [float(x) for x in position_part]
        }
    except Exception as e:
        print(f"Error parsing clustering data '{data_string}': {e}")
        return None


def udp_listener(port, data_list, csv_writer):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', port))

    while True:
        data, addr = sock.recvfrom(1024)
        decoded_data = data.decode()

        if port == 20001:  # Assuming this is the ZED camera port
            parsed_data = parse_zed_data(decoded_data)
        else:  # Assuming other ports are for clustering data
            parsed_data = parse_clustering_data(decoded_data)

        if parsed_data:
            with data_lock:  # Lock for thread-safe access to the data lists
                data_list.append(parsed_data)

def euclidean_distance(point1, point2):
    return np.linalg.norm(np.array(point1) - np.array(point2))

def find_nearest(zed_item, clustering_objects):
    nearest_object = None
    min_distance = float('inf')

    for obj in clustering_objects:
        distance = euclidean_distance(zed_item['position'], obj['position'])
        if distance < min_distance:
            min_distance = distance
            nearest_object = obj

    return nearest_object, min_distance



def compare_and_merge(zed_data, clustering1_data, clustering2_data, csv_writer, kf):
    for zed_item in zed_data:
        nearest_clustering1, distance_clustering1 = find_nearest(zed_item, clustering1_data)
        nearest_clustering2, distance_clustering2 = find_nearest(zed_item, clustering2_data)

        # Apply Kalman filter
        kf.predict()
        kf.update(np.array([zed_item['distance'], distance_clustering1, distance_clustering2]))
        fused_distance = kf.x[0]

        # Write data to CSV
        csv_writer.writerow([
            zed_item['id'], 
            zed_item['distance'], 
            nearest_clustering1['id'] if nearest_clustering1 else 'None', 
            distance_clustering1 if nearest_clustering1 else 'None', 
            nearest_clustering2['id'] if nearest_clustering2 else 'None', 
            distance_clustering2 if nearest_clustering2 else 'None',
            fused_distance
        ])



if __name__ == "__main__":
    output_file_path = '/home/mic-711/Desktop/output.csv'
    with open(output_file_path, mode='w', newline='') as file:
        csv_writer = csv.writer(file)
        csv_writer.writerow(['ZED ID', 'ZED Distance', 'Clustering1 ID', 'Clustering1 Distance', 'Clustering2 ID', 'Clustering2 Distance', 'Fused Distance'])

        kf = initialize_kalman_filter()

        # Start the data processing thread
        processing_thread = threading.Thread(target=process_data_periodically, args=(csv_writer, kf))
        processing_thread.start()

        threads = []
        for port, data_list in zip([20001, 20002, 20003], [zed_data, clustering1_data, clustering2_data]):
            thread = threading.Thread(target=udp_listener, args=(port, data_list, csv_writer))
            thread.start()
            threads.append(thread)

        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("Shutting down...")
            for thread in threads:
                thread.join()
            processing_thread.join()


