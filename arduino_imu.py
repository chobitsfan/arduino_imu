import serial
import struct
import rclpy
import math
import cv2 as cv
import numpy as np
import time
from gpiozero import OutputDevice
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Imu
from multiprocessing import shared_memory

ser = serial.Serial('/dev/ttyAMA1', 921600)

SOP = b'\xAA\x55'
payload_fmt = "<Iffffff"
payload_size = struct.calcsize(payload_fmt)
frame_size = 2 + payload_size + 2

# CRC16-CCITT (0xFFFF)
def crc16_ccitt(data: bytes):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

shm_ts = shared_memory.SharedMemory(name="my_imu_cam_sync", create=True, size=4)

trigger_pin = OutputDevice(16)
trigger_pin.off()

rclpy.init()
node = rclpy.create_node('arduino_imu')
imu_pub = node.create_publisher(Imu, "imu", QoSProfile(depth=20, durability=QoSDurabilityPolicy.VOLATILE))

cali_yml = cv.FileStorage('bmi270_mini_149.yml', cv.FileStorage_READ)
acc_mis_align = cali_yml.getNode("acc_misalign").mat()
acc_scale = cali_yml.getNode("acc_scale").mat()
acc_bias = cali_yml.getNode("acc_bias").mat()
acc_cor = acc_mis_align @ acc_scale
gyro_mis_align = cali_yml.getNode("gyro_misalign").mat()
gyro_scale = cali_yml.getNode("gyro_scale").mat()
gyro_bias = cali_yml.getNode("gyro_bias").mat()
gyro_cor = gyro_mis_align @ gyro_scale
cali_yml.release()

cnt = 0
now_ns = 0
ser.reset_input_buffer()

while True:
    try:
        # Search for SOP
        b = ser.read(1)
        if b != b'\xAA':
            continue
        b2 = ser.read(1)
        if b2 != b'\x55':
            continue

        # Read payload + CRC
        frame = ser.read(payload_size + 2)
        if len(frame) != payload_size + 2:
            continue
    except KeyboardInterrupt:
        break

    payload = frame[:payload_size]
    crc_recv = (frame[payload_size] << 8) | frame[payload_size+1]

    # Verify CRC
    crc_calc = crc16_ccitt(payload)
    if crc_recv != crc_calc:
        print("CRC ERROR")
        continue

    # Unpack payload
    ts, ax, ay, az, gx, gy, gz = struct.unpack(payload_fmt, payload)
    #print(f"ts={ts} sync_ts={sync_ts} ax={ax:.3f} ay={ay:.3f} az={az:.3f} gx={gx:.3f} gy={gy:.3f} gz={gz:.3f}")

    if ax == 0 and gx == 0:
        struct.pack_into('=I', shm_ts.buf, 0, ts)
        continue

    acc_raw = np.array([ax * 9.80665, ay * 9.80665, az * 9.80665], dtype=float).reshape((3, 1))
    acc_cali = acc_cor @ (acc_raw - acc_bias)
    gyro_raw = np.array([gx * math.pi / 180, gy * math.pi / 180, gz * math.pi / 180], dtype=float).reshape((3, 1))
    gyro_cali = gyro_cor @ (gyro_raw - gyro_bias)

    #print(acc_cali[0,0],acc_cali[1,0], acc_cali[2,0], gyro_cali[0,0], gyro_cali[1,0], gyro_cali[2,0])

    imu = Imu()
    imu.header.frame_id = "body"
    imu.header.stamp.sec = ts // 1_000_000
    imu.header.stamp.nanosec = (ts % 1_000_000) * 1000
    imu.linear_acceleration.x = acc_cali[0, 0]
    imu.linear_acceleration.y = acc_cali[1, 0]
    imu.linear_acceleration.z = acc_cali[2, 0]
    imu.angular_velocity.x = gyro_cali[0, 0]
    imu.angular_velocity.y = gyro_cali[1, 0]
    imu.angular_velocity.z = gyro_cali[2, 0]
    #imu.linear_acceleration.x = ax * 9.80665
    #imu.linear_acceleration.y = ay * 9.80665
    #imu.linear_acceleration.z = az * 9.80665
    #imu.angular_velocity.x = gx * math.pi / 180
    #imu.angular_velocity.y = gy * math.pi / 180
    #imu.angular_velocity.z = gz * math.pi / 180
    #imu.linear_acceleration_covariance[0] = xtr;
    imu_pub.publish(imu)

ser.close()
shm_ts.close()
shm_ts.unlink()
node.destroy_node()
rclpy.try_shutdown()
print("bye")
