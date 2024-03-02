import serial
import os


class PacketReader:

    def __init__(self, port='/dev/ttyS0', baud_rate=100000, timeout=None):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.ser = serial.Serial(self.port, 
                                 self.baud_rate, 
                                 timeout=self.timeout, 
                                 bytesize=serial.EIGHTBITS, 
                                 parity=serial.PARITY_EVEN, 
                                 stopbits=serial.STOPBITS_TWO)

    
    def read_raw(self, read_bytes=25):
        sbus_packet = self.ser.read(read_bytes)
        hex_data = ' '.join(format(byte, '02X') for byte in sbus_packet)
        return hex_data

    def read_sbus(self, array_ints = False):
        """
        array_ints: (boolean) If True, it returns list of ints, else it returns the frame as a string
        """
        while 1:
            start_byte = self.ser.read(1)

            if start_byte != b'\x0F':
                continue

            sbus_packet = start_byte + self.ser.read(24) # read next 24 bytes
            
            if len(sbus_packet) != 25:
                print("Something went wrong!")

            if array_ints:
                sbus_packet = [int(byte) for byte in sbus_packet]
            else:
                sbus_packet = ' '.join(format(byte, '02X') for byte in sbus_packet)
            
            return sbus_packet

    def decode_sbus_channels(self, list_=None):
        sbus_raw = self.read_sbus(array_ints=True)
        # sbus_raw = list_
        HEADER = sbus_raw[0]
        CHANNELS = [0 for _ in range(16)]

        CHANNELS[0]  = (sbus_raw[1] |
                                            ((sbus_raw[2] << 8) ))& 0x07FF;
        CHANNELS[1]  = ((sbus_raw[2] >> 3) |
                                            ((sbus_raw[3] << 5) & 0x07FF));
        CHANNELS[2]  = ((sbus_raw[3] >> 6) |
                                            (sbus_raw[4] << 2) |
                                            ((sbus_raw[5] << 10) & 0x07FF));
        CHANNELS[3]  = ((sbus_raw[5] >> 1) |
                                            ((sbus_raw[6] << 7) & 0x07FF));
        CHANNELS[4]  = ((sbus_raw[6] >> 4) |
                                            ((sbus_raw[7] << 4) & 0x07FF));
        CHANNELS[5]  = ((sbus_raw[7] >> 7) |
                                            (sbus_raw[8] << 1) |
                                            ((sbus_raw[9] << 9) & 0x07FF));
        CHANNELS[6]  = ((sbus_raw[9] >> 2) |
                                            ((sbus_raw[10] << 6) & 0x07FF));
        CHANNELS[7]  = ((sbus_raw[10] >> 5) |
                                            ((sbus_raw[11] << 3) & 0x07FF));
        CHANNELS[8]  = (sbus_raw[12] |
                                            ((sbus_raw[13] << 8) & 0x07FF));
        CHANNELS[9]  = ((sbus_raw[13] >> 3) |
                                            ((sbus_raw[14] << 5) & 0x07FF));
        CHANNELS[10] = ((sbus_raw[14] >> 6) |
                                            (sbus_raw[15] << 2) |
                                            ((sbus_raw[16] << 10) & 0x07FF));
        CHANNELS[11] = ((sbus_raw[16] >> 1) |
                                            ((sbus_raw[17] << 7) & 0x07FF));
        CHANNELS[12] = ((sbus_raw[17] >> 4) |
                                            ((sbus_raw[18] << 4) & 0x07FF));
        CHANNELS[13] = ((sbus_raw[18] >> 7) |
                                            (sbus_raw[19] << 1) |
                                            ((sbus_raw[20] << 9) & 0x07FF));
        CHANNELS[14] = ((sbus_raw[20] >> 2) |
                                            ((sbus_raw[21] << 6) & 0x07FF));
        CHANNELS[15] = ((sbus_raw[21] >> 5) |
                                            ((sbus_raw[22] << 3) & 0x07FF));
        

        CHANNEL_17 = sbus_raw[23] & 0x01
        CHANNEL_18 = sbus_raw[23] & 0x02
        LOST_FRAME = sbus_raw[23] & 0x04
        FAILSAFE = sbus_raw[23] & 0x08
        FOOTER = sbus_raw[24]
        return [HEADER, *CHANNELS, CHANNEL_17, CHANNEL_18, LOST_FRAME, FAILSAFE, FOOTER]
    

    def decode_sbus_display(self, in_hex=False):
        sbus_packets_decoded = self.decode_sbus_channels()
        
        print("Header:", hex(sbus_packets_decoded[0]) if in_hex else sbus_packets_decoded[0])
        
        for _ in range(1, 20):
            print(f"Channel {_}:", hex(sbus_packets_decoded[_]) if in_hex else sbus_packets_decoded[_])
        
        print("Lost Frame:", hex(sbus_packets_decoded[19]) if in_hex else sbus_packets_decoded[19])
        
        print("Failsafe:", hex(sbus_packets_decoded[20]) if in_hex else sbus_packets_decoded[20])
        
        print("Footer:",  hex(sbus_packets_decoded[21]) if in_hex else sbus_packets_decoded[21])


    def clear_display(self):
        os.system('cls' if os.name == 'nt' else 'clear')