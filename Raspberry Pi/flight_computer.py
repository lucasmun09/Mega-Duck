import serial
import struct
import math

# Show all packets
VERBOSE = 1

# Native USB port of Due (use Device Manager to find)
PORT = 'COM4'

# Port for wireless data transfer device
CLIENT_PORT = 'COM3'

# File to store output data
OUTPUT_FILE = 'data.txt'

# x secs before the communication times out
TIMEOUT = 30

# Baudrate for data transfer rate
BAUD = 57600

# Baudrate for client data transfer
CLIENT_BAUD = 57600

ready = False


def generate_chirp(t1, phi, f0, f1, num_samples):
    #f0 inital frequency find limits
    #f1 last frequency find limits
    #t1 duration
    #phi phase shift
    #num_samples
    # "Chirpyness" or rate of frequency change
    k = (f1 - f0) / t1
    output = [0] * num_samples
    for i in range(num_samples):
        t = t1 * (i/num_samples)
        # Create a chirp from the frequency f(t) = f0 + kt
        chirp = math.cos(phi + 2*math.pi * (f0*t + k/2 * t*t))
        # Create a Hanning window to envelope the chirp
        window = 0.5 * (1 - math.cos(2*math.pi * i/(num_samples - 1)))
        # Move the signal across a different reference
        output[i] = round(4095/2 + 4095/2 * (chirp * window))
    print("Chirp data generated")
    return queue(output)


def generate_chirp():
    t1 = 2e-3; phi = 0; f0 = 5e3; f1 = 105e3; num_samples = 2048
    output = [0] * num_samples
    # "Chirpyness" or rate of frequency change
    k = (f1 - f0) / t1
    for i in range(num_samples):
        t = t1 * (i / num_samples)
        # Create a chirp from the frequency f(t) = f0 + kt
        chirp = math.cos(phi + 2 * math.pi * (f0 * t + k / 2 * t * t))
        # Create a Hanning window to envelope the chirp
        window = 0.5 * (1 - math.cos(2 * math.pi * i / (num_samples - 1)))
        # Move the signal across a different reference
        output[i] = round(4095 / 2 + 4095 / 2 * (chirp * window))
    print("Chirp data generated")
    return queue(output)


def read(ser, n):
    data = ser.read(size=n)
    if VERBOSE:
        print('> ' + ''.join('{:02x} '.format(b) for b in data))
    return data


def write(ser, data):
    ser.write(bytearray(data))
    if VERBOSE:
        print('< ' + ''.join('{:02x} '.format(b) for b in data))


def close(ser):
    print('Closing connection')
    ser.close()


def queue(chirp):
    queue_data = [0] * (1 + 4 + 4096 + 1)
    queue_data[0] = 1
    queue_data[1] = (4096 >> 0) & 0xff
    queue_data[2] = (4096 >> 8) & 0xff
    queue_data[3] = (4096 >> 16) & 0xff
    queue_data[4] = (4096 >> 24) & 0xff
    for i in range(2048):
        queue_data[5 + 2 * i + 0] = (chirp[i] >> 0) & 0xff
        queue_data[5 + 2 * i + 1] = (chirp[i] >> 8) & 0xff
    if VERBOSE:
        print("Data queued")
    return queue_data


def request_data(ser):
    # Initiate data collection
    write(ser, [2, 0, 0, 0, 0])
    #  Get initial response
    opcode, response_len = struct.unpack('<BI', read(ser, 5))
    if opcode != 0x82 or response_len != 0:
        print('unexpected! opcode=0x{:02x}, response_len={}'
              .format(opcode, response_len))
        return

    print('collect_data: started... ', end='')

    #  Get header
    opcode, response_len = struct.unpack('<BI', read(ser, 5))
    if opcode != 0x82:
        print('unexpected! opcode=0x{:02x}, response_len={}'
              .format(opcode, response_len))
        return

    # Record the data

    raw_data = read(ser, response_len)
    while True: # waits for the ending opcode
        if ser.in_waiting >= 5:
            stopcode, packet = struct.unpack('<BI', read(ser, 5))
            if stopcode != 0x86:
                print('unexpected opcode! Data transmission unsuccessful')
                exit(1)
            print('done! ({} data points)'.format(response_len // 2))
            break

    total_length = len(raw_data)
    with open("1-" + OUTPUT_FILE, 'w') as f:
        for i in range(0, int(total_length/2), 2):
            data = raw_data[i] | (raw_data[i + 1] << 8)
            f.write('{}\n'.format(data))

    with open("2-" + OUTPUT_FILE, 'w') as f:
        for i in range(int(total_length/2), total_length, 2):
            data = raw_data[i] | (raw_data[i + 1] << 8)
            f.write('{}\n'.format(data))
    print('Output data written to 1-{} and 2-{}'.format(OUTPUT_FILE, OUTPUT_FILE))
    return raw_data

def send_sample(ser1, sample, sample_number):
    continue_code = 0x0B
    packet_size = 100
    hex_sample_number = hex(sample_number)
    hex_packet_size = hex(packet_size)
    write(ser1, [hex_sample_number, hex_packet_size])
    data_packet = []
    for i in range(0, int(len(sample)/packet_size)):
        for j in range(0, packet_size):
            data_packet[j] = sample[j + packet_size*i]
        length = len(data_packet)
        data_packet[length+1] = continue_code
        write(ser1, data_packet)
        while True:
            if ser1.in_waiting == 1:
                opcode, response = struct.unpack('<BI', read(ser1, 5))
                if opcode == 0x04:
                    # ACK
                    break
                if opcode == 0x05:
                    continue_code = 0x0B
                    write(ser1, data_packet)
    write(ser1, 0x06)


def upload_chirp(ser, queue_data):
    write(ser, queue_data)
    if VERBOSE:
        print("Chirp data sent")
    opcode, response_len = struct.unpack('<BI', read(ser, 5))
    if opcode != 0x81 or response_len != 0:
        print('unexpected! opcode=0x{:02x}, response_len={:04}'
              .format(opcode, response_len))
        return
    if VERBOSE:
        print("Chirp profile uploaded.")


def init_daq(port):
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = BAUD
    ser.timeout = TIMEOUT
    ser.setRTS(True)
    ser.setDTR(True)
    ser.open()
    # Handshake
    write(ser, [0, 0, 0, 0, 0])
    opcode, response_len = struct.unpack('<BI', read(ser, 5))

    if opcode != 0x80 or response_len != 2:
        print('unexpected! opcode=0x{:02x}, response_len={}'
              .format(opcode, response_len))
        exit(1)

    # write version
    version = struct.unpack('<B B', read(ser, response_len))
    print('hello: version={}'.format(version))
    print('Communicating over port {}'.format(ser.name))
    return ser


def init_client_coms(client_port):
    ser1 = serial.Serial()
    ser1.port = client_port
    ser1.baudrate = CLIENT_BAUD
    ser1.timeout = TIMEOUT
    ser1.setRTS(True)
    ser1.setDTR(True)
    ser1.open()
    #  handshake with computer
    write(ser1, [0, 0, 0, 0, 0])

    return ser1


def reset(ser):
    write(ser, [3, 0, 0, 0, 0])
    opcode, response_len = struct.unpack('<BI', read(ser, 5))

    if opcode != 0x80 or response_len != 2:
        print('unexpected! opcode=0x{:02x}, response_len={}'
              .format(opcode, response_len))
        exit(1)


def main():
    my_daq = init_daq(PORT)  # initialize DAQ object
    my_client = init_client_coms(CLIENT_PORT)
    request_data(my_daq)  # request data, 20K samples each per channel: A0 and A1

    #  chirp = generate_chirp() Generates the chirp and the data to be queued
    #  upload_chirp(my_daq, chirp), request chirp, with given data
    collected = 0
    ready = True
    while True:
        if ready:
            print('collecting next data point')
            data = request_data(my_daq)  # request data, 20K samples each per channel: A0 and A1
            collected = collected + 1
        if collected % 10 == 0 & collected != 0:
            send_sample(my_client, data, collected)
        if collected == 20:
            break
    my_daq.close()
    print("Closing connection")

if __name__ == '__main__':
    main()

