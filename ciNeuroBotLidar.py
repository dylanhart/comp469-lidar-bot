"""
    Get Data from Neato LIDAR using pyserial
    ========================================

    Data format for firmware V2.4 and v2.6 (recent production units)

    A full revolution will yield 90 packets, containing 4 consecutive readings each.
    The length of a packet is 22 bytes. This amounts to a total of 360 readings (1 per degree) on 1980
    bytes.

    Each packet is organized as follows:
    <start> <index> <speed_L> <speed_H> [Data 0] [Data 1] [Data 2] [Data 3] <checksum_L> <checksum_H>

    where:

    <start> is always 0xFA

    <index> is the index byte in the 90 packets, going from 0xA0 (packet 0, readings 0 to 3) to 0xF9
    (packet 89, readings 356 to 359).

    <speed>> is a two-byte information, little-endian. It represents the speed, in 64th of RPM (aka
    value in RPM represented in fixed point, with 6 bits used for the decimal part).

    [Data 0] to [Data 3] are the 4 readings. Each one is 4 bytes long, and organized as follows :

    `byte 0 : <distance 7:0>`
    `byte 1 : <"invalid data" flag> <"strength warning" flag> <distance 13:8>`
    `byte 2 : <signal strength 7:0>`
    `byte 3 : <signal strength 15:8>`

    The distance information is in mm, and coded on 14 bits. The minimum distance is around 15cm, and
    the maximum distance is around 6m.

    When bit 7 of byte 1 is set, it indicates that the distance could not be calculated. When this
    bit is set, it seems that byte 0 contains an error code. Examples of error code are 0x02, 0x03, 0x21,
    0x25, 0x35 or 0x50... When it's `21`, then the whole block is `21 80 XX XX`, but for all the other
    values it's the data block is `YY 80 00 00`...

    The bit 6 of byte 1 is a warning when the reported strength is greatly inferior to what is expected
    at this distance. This may happen when the material has a low reflectance (black material...), or
    when the dot does not have the expected size or shape (porous material, transparent fabric, grid,
    edge of an object...), or maybe when there are parasitic reflections (glass... ).

    Byte 2 and 3 are the LSB and MSB of the strength indication. This value can get very high when facing
    a retroreflector.

    <checksum> is a two-byte checksum of the packet. The algorithm is in checksum(), provided that
    `data` is the list of the 20 first bytes, in the same order they arrived in.

    """

import threading
import time
import sys
import traceback
import math
import codecs
import serial

class Lidar:
    
    def __init__(self):
        # for Linux it would be something like the following depending on which port USB is connected to:
        com_port = "/dev/ttyUSB0"
        # try the command "ls /dev/tty*" to see what's available
        # for Mac OS X, use:
        # com_port = "/dev/cu.usbserial"
        baudrate = 115200
        
        self.init_level = 0
        self.index = 0
        
        self.lidarData = [[] for i in range(360)]  # A list of 360 elements Angle, Distance , quality
        self.lidarBuffer = [[] for i in range(360)] #A buffer for LIDAR data to copy to and read from
        self.dataLock = threading.Lock()
        self.bufferLock = threading.Lock()
        self.ser = serial.Serial(com_port, baudrate)
        self.buffer_filled = threading.Condition(self.bufferLock)
        self.quit = False
        self.read_thread = threading.Thread(target=self.readLidar)
        self.read_thread.start()

    def checksum(self, data):
        """
        Compute and return the checksum as an int.
        
        data -- list of 20 bytes, in the order they arrived in.
        """
        # make it into a list of 20 integers
        data = list(data)
        # group the data by word, little-endian
        data_list = []
        for t in range(10):
            data_list.append(data[2*t] + (data[2*t+1] << 8))

        # compute the checksum on 32 bits
        chk32 = 0
        for d in data_list:
            chk32 = (chk32 << 1) + d

        # return a value wrapped around on 15bits, and truncated to still fit into 15 bits
        checksum = (chk32 & 0x7FFF) + (chk32 >> 15)  # wrap around to fit into 15 bits
        checksum = checksum & 0x7FFF  # truncate to 15 bits
        return int(checksum)


    def readLidar(self):

        nb_errors = 0
        while not self.quit:
            # try:
            time.sleep(0.00001)  # do not hog the processor power
            
            if self.init_level == 0:
                b_start = self.ser.read(1)
                #print("b_start = {0}".format(b_start))
                b = int.from_bytes(b_start, 'big')
                # start byte
                if b == 0xFA:
                    self.init_level = 1
                    #print("got start\n")
                else:
                    self.init_level = 0
            elif self.init_level == 1:
                # position index
                b_index = self.ser.read(1)
                #print("b_index = {0}".format(b_index))
                b = int.from_bytes(b_index, 'big')
                if b >= 0xA0 and b <= 0xF9:
                    self.index = b - 0xA0
                    self.init_level = 2
                    #print("got packet #{0:02x}\n".format(self.index))
                    
                    
                elif b != 0xFA:
                    self.init_level = 0
            elif self.init_level == 2:
                # speed
                # b_speed = [ b for b in ser.read(2)]
                b_speed = self.ser.read(2)
                
                # data
                # b_data0 = [ b for b in ser.read(4)]
                # b_data1 = [ b for b in ser.read(4)]
                # b_data2 = [ b for b in ser.read(4)]
                # b_data3 = [ b for b in ser.read(4)]
                b_data0 = self.ser.read(4)
                b_data1 = self.ser.read(4)
                b_data2 = self.ser.read(4)
                b_data3 = self.ser.read(4)
                
                #print("data\n")
                #print("data0 = {0}".format(b_data0))
                #print("data1 = {0}".format(b_data1))
                #print("data2 = {0}".format(b_data2))
                #print("data3 = {0}".format(b_data3))
                
                # we need all the data of the packet to verify the checksum
                all_data = b_start + b_index + b_speed + b_data0 + b_data1 + b_data2 + b_data3
                #print("all_data = {0}".format(all_data))
                
                # checksum
                b_checksum = self.ser.read(2)
                #print("checksum bytes: {0}".format(b_checksum))
                # incoming_checksum = int(b_checksum[0]) + (int(b_checksum[1]) << 8)
                incoming_checksum = int.from_bytes(b_checksum, 'little')
                #print("incoming_checksum = {0}".format(incoming_checksum))
                
                # verify that the received checksum is equal to the one computed from the data
                calculated_checksum = self.checksum(all_data)
                #print("calculated_checksum = {0}".format(calculated_checksum))
                if self.checksum(all_data) == incoming_checksum:
                    speed_rpm = float(int.from_bytes(b_speed, 'big')) / 64.0
                    #print("speed = {0}".format(speed_rpm))
                    # if visualization:
                    #     gui_update_speed(speed_rpm)
                    
                    self.dataLock.acquire()
                    self.update_view(self.index * 4 + 0, b_data0)
                    self.update_view(self.index * 4 + 1, b_data1)
                    self.update_view(self.index * 4 + 2, b_data2)
                    self.update_view(self.index * 4 + 3, b_data3)
                    self.dataLock.release()
                    
                    self.lidarBuffer[self.index * 4 + 0] = self.lidarData[self.index * 4 + 0]
                    self.lidarBuffer[self.index * 4 + 1] = self.lidarData[self.index * 4 + 1]
                    self.lidarBuffer[self.index * 4 + 2] = self.lidarData[self.index * 4 + 2]
                    self.lidarBuffer[self.index * 4 + 3] = self.lidarData[self.index * 4 + 3]
                    
                    if (self.index * 4 + 3) == 359:
                        with self.buffer_filled:
                            self.buffer_filled.notify()
                    
                else:
                    # the checksum does not match, something went wrong...
                    nb_errors += 1
                    #print("wrong checksum nb_errors={0}\n".format(nb_errors))
                # if visualization:
                #     label_errors.text = "errors: "+str(nb_errors)
                
                # display the samples in an error state
                # update_view(index * 4 + 0, [0, 0x80, 0, 0])
                # update_view(index * 4 + 1, [0, 0x80, 0, 0])
                # update_view(index * 4 + 2, [0, 0x80, 0, 0])
                # update_view(index * 4 + 3, [0, 0x80, 0, 0])
                
                self.init_level = 0  # reset and wait for the next packet
                #print(self.index)
                #print(self.lidarData)
            # return # to test

            else:  # default, should never happen...
                self.init_level = 0
    # except :
    #     traceback.print_exc(file=sys.stdout)
    #     return

    def update_view(self, angle, data):
        """
        Updates the view of a sample.
        
        Takes the angle (an int, from 0 to 359) and the list of four bytes of data in the order they arrived.
        """
        # global offset, use_outer_line, use_line
        # unpack data using the denomination used during the discussions
        # x0 = data[0]
        # x1 = data[1]
        # x2 = data[2]
        # x3 = data[3]
        
        angle_rad = angle * math.pi / 180.0
        c = math.cos(angle_rad)
        s = -math.sin(angle_rad)
        
        dist_calc_error = (data[1] & 0x80) > 0  # check bit 7 flag
        #if dist_calc_error:
            #print("distance calculation error: {0}\n".format(data[0]))  # error code in data[0]
        
        inferior_signal = (data[1] & 0x40) > 0  # check bit 6 flag
        #if inferior_signal:
            #print("inferior signal\n")
        
        dist_mm = data[0] | ((data[1] & 0x3f) << 8)  # remove the flags
        quality = data[2] | (data[3] << 8)  # quality is on 16 bits
        
        #quality = int.from_bytes(data[2:4], 'big')
        
        self.lidarData[angle] = [dist_mm, quality]
        dist_x = dist_mm*c
        dist_y = dist_mm*s
    #     if visualization:
    #         #reset the point display
    #         point.pos[angle] = vector( 0, 0, 0 )
    #         pointb.pos[angle] = vector( 0, 0, 0 )
    #         point2.pos[angle] = vector( 0, 0, 0 )
    #         point2b.pos[angle] = vector( 0, 0, 0 )
    #         if not use_lines : lines[angle].pos[1]=(offset*c,0,offset*s)
    #         if not use_outer_line :
    #             outer_line.pos[angle]=(offset*c,0,offset*s)
    #             outer_line.color[angle] = (0.1, 0.1, 0.2)


    #         # display the sample
    #         if x1 & 0x80: # is the flag for "bad data" set?
    #             # yes it's bad data
    #             lines[angle].pos[1]=(offset*c,0,offset*s)
    #             outer_line.pos[angle]=(offset*c,0,offset*s)
    #             outer_line.color[angle] = (0.1, 0.1, 0.2)
    #         else:
    #             # no, it's cool
    #             if not x1 & 0x40:
    #                 # X+1:6 not set : quality is OK
    #                 if use_points : point.pos[angle] = vector( dist_x,0, dist_y)
    #                 if use_intensity : point2.pos[angle] = vector( (quality + offset)*c,0,
    #                       (quality + offset)*s)
    #                 if use_lines : lines[angle].color[1] = (1,0,0)
    #                 if use_outer_line : outer_line.color[angle] = (1,0,0)
    #             else:
    #                 # X+1:6 set : Warning, the quality is not as good as expected
    #                 if use_points : pointb.pos[angle] = vector( dist_x,0, dist_y)
    #                 if use_intensity : point2b.pos[angle] = vector( (quality + offset)*c,0,
    #                       (quality + offset)*s)
    #                 if use_lines : lines[angle].color[1] = (0.4,0,0)
    #                 if use_outer_line : outer_line.color[angle] = (0.4,0,0)
    #             if use_lines : lines[angle].pos[1]=( dist_x, 0, dist_y)
    #             if use_outer_line : outer_line.pos[angle]=( dist_x, 0, dist_y)

    def get_image(self):
        """
        Returns the current lidar buffer
        """
        lidarOut = []
        
        with self.buffer_filled:
            while (self.index * 4 + 3) < 359:
                self.buffer_filled.wait()
            
            lidarOut = self.lidarBuffer
        
        #out = [a[0] if len(a) > 0 else 0 for a in lidarOut]
        # return lidarOut[180:] + lidarOut[:180]
        return lidarOut

