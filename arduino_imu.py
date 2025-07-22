import serial
import struct

ser = serial.Serial('/dev/ttyAMA1', 115200)

SOP = b'\xAA\x55'
payload_fmt = "<IBffffff"
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

while True:
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

    payload = frame[:payload_size]
    crc_recv = (frame[payload_size] << 8) | frame[payload_size+1]

    # Verify CRC
    crc_calc = crc16_ccitt(payload)
    if crc_recv != crc_calc:
        print("CRC ERROR")
        continue

    # Unpack payload
    ts, xtr, ax, ay, az, gx, gy, gz = struct.unpack(payload_fmt, payload)

    print(f"ts={ts} xtr={xtr} ax={ax:.3f} ay={ay:.3f} az={az:.3f} gx={gx:.3f} gy={gy:.3f} gz={gz:.3f}")
