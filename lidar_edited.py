#Display Data from Neato LIDAR
#based on code from Nicolas "Xevel" Saugnier
#requires vpython and pyserial

import thread, time, sys, traceback, math, struct

COM_PORT = "COM3" # example: 5 == "COM6" == "/dev/tty5"
BAUD_RATE = 115200
FPS = 60
OFFSET = 140

from visual import *
point = points(pos=[(0,0,0) for i in range(360)], size=5, color=(0 , 1, 0)) #green, good point
lidar_representation = ring(pos=(0,0,0), axis=(0,1,0), radius=OFFSET-1, thickness=1, color = color.yellow)
lidar_representation.visible = True

label_speed = label(pos = (0,-500,0), xoffset=1, box=False, opacity=0.1)
label_errors = label(pos = (0,-1000,0), xoffset=1, text="errors: 0", visible = False, box=False)


def reset_display( angle ):
    angle_rad = angle * math.pi / 180.0
    c = math.cos(angle_rad)
    s = -math.sin(angle_rad)
    
    #reset the point display    
    point.pos[angle] = vector( 0, 0, 0 )         
         
def parse_point_data( angle, data ):
    dist_mm = data[0] | (( data[1] & 0x3f) << 8) # distance is coded on 14 bits
    quality = data[2] | (data[3] << 8) # quality is on 16 bits
    is_bad_data = data[1] & 0x80
    is_low_quality = data[1] & 0x40
    parsed_data = (dist_mm, is_bad_data, is_low_quality)
    return parsed_data
    
def update_point( angle, data ):
    """Updates the view of a sample.
Takes the angle (an int, from 0 to 359) and the list of four bytes of data in the order they arrived.
"""
    
    (dist_mm, is_bad_data, is_low_quality) = parse_point_data(angle, data)
    
    angle_rad = angle * math.pi / 180.0
    c = math.cos(angle_rad)
    s = -math.sin(angle_rad)
    
    dist_x = dist_mm*c
    dist_y = dist_mm*s
           
    reset_display(angle)
    # display the sample
    if is_bad_data:
        return
    else:
        point.pos[angle] = vector(dist_x, 0, dist_y)
        if is_low_quality:
            point.color[angle] =(0.4, 0, 0) #red for low quality
        else:
            point.color[angle] =(0, 1, 0) #green for good quality

def check_sum(data):
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

def read_lidar():
    #data string: <start> <index> <speed_L> <speed_H> [Data 0] [Data 1] [Data 2] [Data 3] <checksum_L> <checksum_H>
    
    START_BYTE = 0xFA
    INDEX_OFFSET = 0xA0
    NUM_PACKETS = 90
    PACKET_BODY_SIZE = 20 #speed_L to checksum_H
    DATA_POINTS = 4
    
    check_sum_errors = 0
    index = 0
    
    while True:
        #start byte
        full_data = ser.read(1)
        if ord(full_data) != START_BYTE:
            continue 
        #position index
        full_data += ser.read(1)
        index = ord(full_data[1]) - INDEX_OFFSET 
        if index < 0 or index > NUM_PACKETS:
            continue
        full_data += ser.read(PACKET_BODY_SIZE)
        data = [[] for inc in range(DATA_POINTS)]
        
        #x=pad byte, H=unsigned short, 4s=4 chars
        (speed_rpm, data[0], data[1], data[2], data[3], incoming_check_sum) = struct.unpack('x x H 4s 4s 4s 4s H', full_data)

        #verify that the received checksum is equal to the one computed from the data
        if check_sum([ord(b) for b in full_data]) == incoming_check_sum:
            speed_rpm = float( speed_rpm ) / 64.0
            label_speed.text = "RPM: " + str(speed_rpm) 
            b_data = [bytearray(d) for d in data]

        else:
            #the checksum does not match, something went wrong...
            check_sum_errors +=1
            label_errors.text = "errors: "+str(check_sum_errors)
            #give the samples an error state
            b_data = [[0, 0x80, 0, 0] for d in data]
            
        for (inc, d) in enumerate(b_data):
            update_point(index * 4 + inc, d)    

import serial
ser = serial.Serial(COM_PORT, BAUD_RATE)
th = thread.start_new_thread(read_lidar, ())

while True:
    rate(FPS) # synchronous repaint at 60fps