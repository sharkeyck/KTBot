
"""
ROS Bindings can be found in the kt_node package.
"""
import thread, struct
import serial

START_BYTE = 0xFA
INDEX_OFFSET = 0xA0
NUM_PACKETS = 90
PACKET_BODY_SIZE = 20 #speed_L to checksum_H
DATA_POINTS = 4

class Lidar():
    MIN_RANGE = 0.020
    MAX_RANGE = 5.0

    def __init__(self, port="/dev/ttyUSB0", cb=None):
        self.ser = serial.Serial(port, 115200, timeout=0.5)
        self.cb = cb
        th = thread.start_new_thread(self.read_lidar, ())
        self.set_spin(False)

    def read_lidar(self):
        """ Read values of a scan -- call requestScan first! """
        ranges = [0]*360

        prev_angle = 360

        while True:
            try: 
                packet = self.read_packet()
                if not packet:
                    continue
                (speed_rpm, data) = packet
            except Exception, e:
                print(e)
            for d in data:
                (angle, dist_m, is_bad_data, is_low_quality) = d
                if not is_bad_data:
                    ranges[angle] = dist_m
                else:
                    ranges[angle] = self.MAX_RANGE
            if angle < prev_angle:
                self.cb(ranges)
            prev_angle = angle

    def set_spin(self, enable=True):
        if enable:
            self.ser.write("MotorOn\n")
        else:
            self.ser.write("MotorOff\n")

    def parse_point_data(self, angle, data):
        dist_mm = data[0] | (( data[1] & 0x3f) << 8) # distance is coded on 14 bits
        quality = data[2] | (data[3] << 8) # quality is on 16 bits
        is_bad_data = data[1] & 0x80
        is_low_quality = data[1] & 0x40
        parsed_data = (angle, dist_mm/1000.0, is_bad_data, is_low_quality)
        return parsed_data

    def check_sum(self, data):
        """Compute and return the check_sum as an int.
            data is a list of 22 bytes (as ints), in the order they arrived in.
            last 2 bytes should be ignored, as they  are the provided checksum.
        """
        #group the data by word, little-endian
        data_list = []
        for t in range(10):
            data_list.append( data[2*t] + (data[2*t+1]<<8) )

        # compute the check_sum on 32 bits
        chk32 = 0
        for d in data_list:
            chk32 = (chk32 << 1) + d

        check_sum = (chk32 & 0x7FFF) + ( chk32 >> 15 ) # wrap around to fit into 15 bits
        check_sum = check_sum & 0x7FFF # truncate to 15 bits
        return int( check_sum )

    def read_packet(self):
        #data string: <start> <index> <speed_L> <speed_H> [Data 0] [Data 1] [Data 2] [Data 3] <checksum_L> <checksum_H>

        check_sum_errors = 0
        index = 0

        #start byte
        full_data = self.ser.read(1)
        if full_data == '' or ord(full_data) != START_BYTE:
            return None
        #position index
        full_data += self.ser.read(1)
        index = ord(full_data[1]) - INDEX_OFFSET
        if index < 0 or index > NUM_PACKETS:
            return None
        full_data += self.ser.read(PACKET_BODY_SIZE)
        data = [[] for inc in range(DATA_POINTS)]

        #x=pad byte, H=unsigned short, 4s=4 chars
        (speed_rpm, data[0], data[1], data[2], data[3], incoming_check_sum) = struct.unpack('x x H 4s 4s 4s 4s H', full_data)

        #verify that the received checksum is equal to the one computed from the data
        if self.check_sum([ord(b) for b in full_data]) == incoming_check_sum:
            speed_rpm = float( speed_rpm ) / 64.0
            # TODO: also return speed_rpm as computed here
            b_data = [bytearray(d) for d in data]
        else:
            #the checksum does not match, something went wrong...
            check_sum_errors +=1
            # TODO: also return errors
            #give the samples an error state
            b_data = [[0, 0x80, 0, 0] for d in data]

        return (speed_rpm, [self.parse_point_data(index*4 + inc, d) for (inc, d) in enumerate(b_data)])
