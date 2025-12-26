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
from sensor_msgs.msg import Imu, Image
from std_msgs.msg import Int64

ser = serial.Serial('/dev/ttyAMA1', 921600, rtscts=True)

SOP = b'\xAA\x55'
payload_fmt = "<IHHhhhhhh"
payload_size = struct.calcsize(payload_fmt)

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

trigger_pin = OutputDevice(16)
trigger_pin.off()

rclpy.init()
node = rclpy.create_node('arduino_imu')
imu_pub = node.create_publisher(Imu, "imu", QoSProfile(depth=20, durability=QoSDurabilityPolicy.VOLATILE))
t_offset_pub = node.create_publisher(Int64, "pico_pi_t_offset", QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE))
cam_sync_pub = node.create_publisher(Image, "cam_sync", QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE))

acc_raw_to_ms = (2 << 2) / 32768.0 * 9.80665
gyro_raw_to_rad_sec = (125 * (1 << 4)) / 32768.0 * math.pi / 180

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

time.monotonic_ns()
cnt = 787
sync_ts = 0
prv_imu_ts = 0
prv_seq = -1
pico_pi_t_offset_ns = 0

ser.reset_input_buffer()

print("start")

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
    ts, exp_us, seq, ax, ay, az, gx, gy, gz = struct.unpack(payload_fmt, payload)
    #print(f"ts={ts} ax={ax:.3f} ay={ay:.3f} az={az:.3f} gx={gx:.3f} gy={gy:.3f} gz={gz:.3f}")

    if prv_seq != -1 and seq - prv_seq > 1 and seq - prv_seq != -65535:
        print("miss msg", prv_seq, seq)
    prv_seq = seq

    if ax == 0 and ay == 0 and gx == 0: # this is a cam trigger timestamp msg
        pi_cam_ts = (ts + exp_us) * 1000 - pico_pi_t_offset_ns
        # I use Image to send camera image timestamp, this is ugly, but I am too lazy to use custom msg
        sync_msg = Image()
        sync_msg.header.frame_id = "body"
        sync_msg.header.stamp.sec = pi_cam_ts // 1_000_000_000
        sync_msg.header.stamp.nanosec = pi_cam_ts % 1_000_000_000
        sync_msg.height = ts + exp_us // 2
        cam_sync_pub.publish(sync_msg)
        continue

    if sync_ts > 0 and ax == 1 and ay == 1 and gx == 1:
        pico_pi_t_offset_ns = ts * 1000 - sync_ts
        t_off = Int64()
        t_off.data = pico_pi_t_offset_ns
        t_offset_pub.publish(t_off)
        sync_ts = 0
        continue

    if ax == 2 and ay == 2 and gx == 2:
        #print("pico", ts)
        continue

    if prv_imu_ts > 0 and ts - prv_imu_ts >= 5500:
        print("miss imu data", prv_imu_ts, ts, ts - prv_imu_ts)
    prv_imu_ts = ts

    acc_raw = np.array([ax * acc_raw_to_ms, ay * acc_raw_to_ms, az * acc_raw_to_ms], dtype=float).reshape((3, 1))
    acc_cali = acc_cor @ (acc_raw - acc_bias)
    gyro_raw = np.array([gx * gyro_raw_to_rad_sec, gy * gyro_raw_to_rad_sec, gz * gyro_raw_to_rad_sec], dtype=float).reshape((3, 1))
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
    imu_pub.publish(imu)

    cnt = cnt + 1
    if cnt > 787:
        cnt = 0
        trigger_pin.on()
        sync_ts = time.monotonic_ns()
        trigger_pin.off()

ser.close()
node.destroy_node()
rclpy.try_shutdown()
print("bye")
