# -----------------------------------------------------------------------------
# Copyright (c) 2024, Lucid Vision Labs, Inc.
#
# Modified by Yanyu Zhang
# Copyright (c) 2024, Yanyu Zhang (yzhan831@ucr.edu)
# -----------------------------------------------------------------------------
from argparse import ArgumentParser
from pathlib import Path

from arena_api.system import system
from arena_api.buffer import *

import ctypes
import numpy as np
import cv2
from cv2 import aruco
import time
from sensor_msgs_py import point_cloud2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge

def create_devices_with_tries():
	tries = 0
	tries_max = 6
	sleep_time_secs = 10
	while tries < tries_max:  # Wait for device for 60 seconds
		devices = system.create_device()
		if not devices:
			print(
				f'  Try {tries+1} of {tries_max}: waiting for {sleep_time_secs} '
				f'secs for a device to be connected!')
			for sec_count in range(sleep_time_secs):
				time.sleep(1)
				print(f'  {sec_count + 1 } seconds passed ',
					'.' * sec_count, end='\r')
			tries += 1
		else:
			print(f'  Created {len(devices)} device(s)')
			return devices
	else:
		raise Exception(f'  No device found! Please connect a device and run '
						f'the example again.')
     
def open_device_by_identifier(identifier: str):
    """
    identifier can be an IP like '192.168.1.12', or MAC '1C:0F:AF:18:16:C8',
    or serial number. We return a single opened device handle.
    """
    # Re-discover device infos until we see at least one
    tries, tries_max, sleep_time_secs = 0, 6, 10
    dev = None
    while tries < tries_max and dev is None:
        infos = system.device_infos  # does not open devices
        target = None
        print("Infos:", infos)
        for info in infos:
            ip_i   = info.get('ip')
            if identifier == ip_i:
                target = info
                break

        if target is None:
            print(f"Try {tries+1}/{tries_max}: waiting for camera '{identifier}'...")
            time.sleep(sleep_time_secs)
            tries += 1
        else:
            # OPEN ONLY THIS ONE
            dev = system.create_device([target])[0]

    if dev is None:
        raise RuntimeError(f"Camera '{identifier}' not found after {tries_max*sleep_time_secs}s.")
    return dev

def setup(device):

    num_channels = 3
    nodemap = device.nodemap
	
    nodes = nodemap.get_node(['Width', 'Height', 'PixelFormat'])
    # 640x360, 1280x720, 1920x1200
    nodes['Width'].value = nodes['Width'].max
    nodes['Height'].value = nodes['Height'].max
    # print(nodes['Width'].max) # 1920
    # print(nodes['Height'].max) # 1200
    nodes['PixelFormat'].value = 'RGB8' # Mono8 for 1 channel, RGB8 for 3 chaneels
	
    nodes_hz = nodemap.get_node(['AcquisitionMode', 'AcquisitionFrameRateEnable', 'AcquisitionFrameRate', 'ExposureTime', 'ExposureAuto'])
    nodes_hz['AcquisitionMode'].value= 'Continuous'
    nodes_hz['AcquisitionFrameRateEnable'].value= True
    nodes_hz['AcquisitionFrameRate'].value = 10.0 # Max: 15 HZ
    print("Exposure time", nodes_hz['ExposureTime'].value, "µs")
    
    # Change exposure time
    # Disable auto-exposure if enabled
    if 'ExposureAuto' in nodes_hz:
        nodes_hz['ExposureAuto'].value = 'Off'
        print("ExposureAuto:", nodes_hz['ExposureAuto'].value)
    # nodes_hz['ExposureTime'].value = 2000.0
    nodes_hz['ExposureTime'].value = 30_000.0
    print("Exposure time", nodes_hz['ExposureTime'].value)          

    # Stream nodemap
    tl_stream_nodemap = device.tl_stream_nodemap
    tl_stream_nodemap["StreamBufferHandlingMode"].value = "NewestOnly"
    tl_stream_nodemap['StreamAutoNegotiatePacketSize'].value = True
    tl_stream_nodemap['StreamPacketResendEnable'].value = True

    return num_channels

class ImagePublisher(Node):
    def __init__(self, ip):
        super().__init__('lucid_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        ) 
        host_num_mapping = {"10": 1, "12": 2}
        if ip is None:
            img_topic_name = "lucid/image_raw"
        else:
            ip_addr_host_num = ip.split(".")[-1]
            img_topic_name = f"lucid/image_raw_{host_num_mapping[ip_addr_host_num]}"
        self.publisher_ = self.create_publisher(Image, img_topic_name, qos_profile)
        self.timer = self.create_timer(0.02, self.image_callback)
        self.bridge = CvBridge()

        if ip is None:
            self.devices = create_devices_with_tries()
            self.device = system.select_device(self.devices)
        else:
             self.device = open_device_by_identifier(ip)
       
        self.num_channels = setup(self.device)

        self.device.start_stream()

    def image_callback(self):
        buffer = self.device.get_buffer()
        sys_sec = buffer.timestamp_ns // 1000000000
        sys_nanosec = buffer.timestamp_ns % 1000000000
        sys_frameid = str(buffer.frame_id)
        item = BufferFactory.copy(buffer)
        self.device.requeue_buffer(buffer)
        exposure_time = self.device.nodemap.get_node('ExposureTime').value

        array = (ctypes.c_ubyte * self.num_channels * item.width * item.height).from_address(ctypes.addressof(item.pbytes))
        buffer_bytes_per_pixel = int(len(item.data)/(item.width * item.height))
        frame = np.ndarray(buffer=array, dtype=np.uint8, shape=(item.height, item.width, buffer_bytes_per_pixel))

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='rgb8')
        
        msg.header.stamp.sec = sys_sec
        msg.header.stamp.nanosec = sys_nanosec
        msg.header.frame_id = sys_frameid
        
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing image at :' + str(msg.header.stamp.sec) + "." + str(msg.header.stamp.nanosec).zfill(9))
                
        BufferFactory.destroy(item)

    def __del__(self):
        self.device.stop_stream()
        self.system.destroy_device()

if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument("--ip", type=str)
    args = parser.parse_args()

    rclpy.init()
    image_publisher = ImagePublisher(args.ip)
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

