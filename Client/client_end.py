import serial
import struct
from operator import xor

PORT = 'COM3'
BAUD = 57600
VERBOSE = 0
TIMEOUT = 30


def read(ser, n):
    data = ser.read(size=n)
    if VERBOSE:
        print('> ' + ''.join('{:02x} '.format(b) for b in data))
    return data


def write(ser, data):
    ser.write(bytearray(data))
    if VERBOSE:
        print('< ' + ''.join('{:02x} '.format(b) for b in data))


def init_connection(port):
    #  Serial settings
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = BAUD
    ser.timeout = TIMEOUT
    ser.setRTS(True)
    ser.setDTR(True)

    #  Handshake
    write(ser, [0, 0, 0, 0, 0])
    opcode, response_len = struct.unpack('<BI', read(ser, 5))
    if opcode != 0x80:
        print("unexpected opcode -- error detected")
    return ser


def upload_motion_profile(ser):
    tx_opcode = 0x08
    ear_1_state = 0x01
    ear_1_duration = 15
    ear_2_state = 0x01
    ear_2_duration = 10
    nl_state = 0x01
    nl_duration = 0x01
    motion_profile_pack = [tx_opcode, ear_1_state, ear_1_duration, ear_2_state, ear_2_duration, nl_state, nl_duration]
    write(ser, motion_profile_pack)
    rx_opcode, success_check = struct.unpack('B B', read(ser, 2))
    if rx_opcode != 0x08:
        print("unknown opcode! error")
    if success_check != 0x01:
        print("motion profile invalid, please reupload")
        exit(1)


def initiate_sonar(ser):
    write(ser, [2, 0, 0, 0, 0])
    opcode, input_len = struct.unpack('<BI', read(ser, 5))
    if opcode != 0x82:
        print("opcode mismatch! error")
        exit(1)

    print("drone sonar system initiated, collecting data...")


def secure_data_receive(ser):
    total_size = 20000
    #  Get checksum, sample numbering and size of each packet being sent
    rx_checksum, msg_len = struct.unpack('<BI', read(ser, 5))
    sample_number, packet_size = struct.unpack('B B', read(ser, 2))

    sample_number = int(hex(sample_number))
    print("incoming sample number: " + str(sample_number))

    packet_size = int(hex(packet_size))
    data_sample = []
    while True:
        for i in range(0, int(total_size/packet_size)):
            if ser.in_waiting == packet_size:
                checksum = 0x00
                for j in range(0, packet_size-1):
                    data_sample[j + packet_size * i] = read(ser, 1)

                    checksum = xor(checksum, data_sample[j + packet_size * i]) & 0xFF
                if checksum == rx_checksum:
                    write(ser, 0x04)
                    break
                else:
                    write(ser, 0x05)


def main():
    print("Welcome to python front end for computer -- connecting to drone")
    print("initiating connection")
    drone = init_connection(PORT)
    print("uploading motion profile")
    upload_motion_profile(drone)
    print("initiating drone sonar system")
    initiate_sonar(drone)
    #  print("going into data receiver mode")
    #  secure_data_receive(drone)


if __name__ == '__main__':
    main()
