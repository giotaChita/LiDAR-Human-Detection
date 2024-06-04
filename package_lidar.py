import socket
import struct
import pandas as pd 
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

LIDAR_IP =  '192.168.1.102'
PORT = 2368 # MSOP
# PORT = 2369 # DIFOP



def payload_data_DIFOP(data):
        # Parse the payload fields
        header = data[0:8]
        motor_speed = struct.unpack('<H', data[8:10])[0]
        ethernet_config_1 = data[10:32]
        ethernet_config_2 = data[32:40]
        lidar_rotation = struct.unpack('<H', data[40:42])[0]
        reserved_1 = data[42:44]
        clock_source_selection = struct.unpack('<H', data[44:46])[0]
        pps_alignment_horizontal_angle = struct.unpack('<H', data[46:48])[0]
        monitor_pps_alignment_angle_error = struct.unpack('<H', data[48:50])[0]
        reserved_2 = data[50:52]
        utc_time = data[52:58]
        latitude_longitude = data[58:80]
        apd_board_temperature = struct.unpack('<H', data[80:82])[0]
        ld_board_temperature = struct.unpack('<H', data[82:84])[0]
        apd_high_voltage = struct.unpack('<H', data[84:86])[0]
        ld_emitting_high_voltage = struct.unpack('<H', data[86:88])[0]
        plate_temperature = struct.unpack('<H', data[88:90])[0]
        plate_humidity = struct.unpack('<H', data[90:92])[0]
        gps_status = data[92]
        pps_status = data[93]
        high_temperature_suspension = struct.unpack('<H', data[94:96])[0]
        cover_dirty_count = data[96]
        cover_dirty_alarm_message = data[97]
        cover_dirty_energy_value = data[98:327]
        threshold_adjustment_value = data[100]
        input_voltage_value = struct.unpack('<H', data[101:103])[0]
        input_current_value = struct.unpack('<H', data[103:105])[0]
        length_of_work = struct.unpack('<I', data[105:109])[0]
        reserved_3 = data[109:1204]
        tail = data[1204:1206]


# Ethernet info the 42 first bytes
def return_ethernet_info(data):
            # Parse Ethernet header fields
        destination_mac = data[0:6]
        source_mac = data[6:12]
        packet_type = data[12:14]

        # Parse Internet Protocol (IP) header fields
        source_ip = data[26:30]
        destination_ip = data[30:34]

        # Parse UDP header fields
        lidar_port = struct.unpack('<H', data[34:36])[0]  # Convert from network byte order (little-endian) to host byte order
        computer_port = struct.unpack('<H', data[36:38])[0]  # Convert from network byte order (little-endian) to host byte order
        udp_length = struct.unpack('<H', data[38:40])[0]  # Convert from network byte order (little-endian) to host byte order
        udp_checksum = struct.unpack('<H', data[40:42])[0]  # Convert from network byte order (little-endian) to host byte order

        # Print parsed header fields
        print("Destination MAC:", ':'.join('%02x' % b for b in destination_mac))
        print("Source MAC:", ':'.join('%02x' % b for b in source_mac))
        print("Packet Type:", ':'.join('%02x' % b for b in packet_type))
        print("Source IP:", '.'.join(map(str, source_ip)))
        print("Destination IP:", '.'.join(map(str, destination_ip)))
        print("LiDAR Port:", lidar_port)
        print("Computer Port:", computer_port)
        print("UDP Length:", udp_length)
        print("UDP Checksum:", udp_checksum)


def calculate_vertical_angle(channel):
    vertical_angles = {
        0: -16, 1: -8, 2: 0, 3: 8, 4: -15, 5: -7, 6: 1, 7: 9,
        8: -14, 9: -6, 10: 2, 11: 10, 12: -13, 13: -5, 14: 3, 15: 11,
        16: -12, 17: -4, 18: 4, 19: 12, 20: -11, 21: -3, 22: 5, 23: 13,
        24: -10, 25: -2, 26: 6, 27: 14, 28: -9, 29: -1, 30: 7, 31: 15
    }
    return vertical_angles.get(channel, None)


def decode_channel_data(data):
    # distance, intensity
    channels = []
    step_channel = 3
    for i in range(0, len(data), step_channel):
        distance = (data[i + 1] << 8) + data[i]
        intensity = data[i + 2]
        # Convert distance to meters
        distance_meters = round(((distance * 0.4) / 100),3)
        channels.append((distance_meters, intensity))
    return channels


def calculate_azimuth(az_hex):
    a = hexToDec(az_hex[0:2])
    b = hexToDec(az_hex[2:4])
    azimuth_angle = round(((b*256 + a) * 0.01),3) # in degrees
    return azimuth_angle


def calculate_horizontal_angle(azimuth_curr, df_azimuth, channel_index):
    # For channel 0, the horizontal angle is equal to the azimuth value
    if channel_index == 0:
        return azimuth_curr
    else:
#         print("Entering channel: ", channel_index)
        angle_increment = df_azimuth / 32
        horizontal_angle = round((azimuth_curr + angle_increment * channel_index), 3)
        return horizontal_angle
    

def calculate_xyz_points(r,a,theta,direction):
    """
     @param a: vertical angle
     @param r: distance
     @param theta: horizontal rotation angle
     @param direction : 1 for direction y=0, 0 for direction x = 0 
 
     @return (x ,y ,z) coordinates

    """

    alpha_rad = np.radians(a)
    theta_rad = np.radians(theta)

    if direction:
        # Calculate Cartesian coordinates y=0
        x = round((r * np.cos(alpha_rad) * np.sin(theta_rad)),3)
        y = round((r * np.cos(alpha_rad) * np.cos(theta_rad)),3)
        z = round((r * np.sin(alpha_rad)),3)
    else:
        # Calculate Cartesian coordinates x=0
        x = round((r * np.cos(alpha_rad) * np.cos(theta_rad)),3)
        y = round((-r * np.cos(alpha_rad) * np.sin(theta_rad)),3)
        z = round((r * np.sin(alpha_rad)),3)

    return x,y,z

def getDecDigit(digit):
     digits = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f']
     for x in range(len(digits)):
          if digit == digits[x]:
               return x
          
def hexToDec(hexNum):
     dec = 0
     power = 0
     for digit in range(len(hexNum),0,-1):
        dec = dec + 16 ** power * getDecDigit(hexNum[digit-1])
        power += 1
     return dec 

def calculate_timestamp_ns(timestamp_hex):

    dec = []
    for i in range(0,len(timestamp_hex),2):
        if i == 7:
                break
        hex = timestamp_hex[i:i+2]
        dec_i = hexToDec(hex)
        dec.append(dec_i)
    # calculate the timestamp
    power = 0
    sum = 0
    for num in dec:
        # print(num)
        sum = sum +  num * (2 ** power)
        power += 8
        # print(power)
    return sum


def data_block_echo(data):
    # Single Echo Mode 
    df_list = []
    azimuth_channel_list= []
    package_size = 1200
    block_size = 100
    
    for i in range(0, package_size, block_size):
        flag = data[i:i + 2]
        azimuth_bytes = data[i + 2:i + 4]
        az_hex = azimuth_bytes.hex()
        azimuth_curr = calculate_azimuth(az_hex)
        azimuth_channel_list.append(azimuth_curr)

        if i % 100 == 0 and i != 1100: # all blocks except from 12 (last)
            az_curr = azimuth_curr
            azimuth_bytes_next_block = data[i + 2 + 100 : i + 4 + 100]
            az_hex_nextb = azimuth_bytes_next_block.hex()
            azimuth_nextb = calculate_azimuth(az_hex_nextb)
            df_azimuth = round(azimuth_nextb - az_curr,3)
            # print(df_azimuth)

            ## Fix values verified ##
            # df_az = 0.09 for freq = 5Hz, 
            # df_az = 0.18 for freq = 10 Hz,
            # df_az = 0.36 for freq = 20 Hz
        
        channels = decode_channel_data(data[i + 4:i + 100])
        
        for count, channel in enumerate(channels, start=0):
            # print(count)
            distance = channel[0]
            intensity = channel[1]
            vertical_angle = calculate_vertical_angle(count)  
            horizontal_angle = calculate_horizontal_angle(azimuth_curr, df_azimuth, count)
            pointsX, pointsY, pointsZ = calculate_xyz_points(distance,vertical_angle,horizontal_angle,1)
            df_list.append({
                "Flag": flag.hex(),
                "Azimuth": azimuth_curr,
                "Channel": count,
                "Distance": distance,
                "Intensity": intensity,
                "Vertical_Angle": vertical_angle,
                "Horizontal_Angle": horizontal_angle,
                "Points_X": pointsX,
                "Points_Y": pointsY,
                "Points_Z": pointsZ
            })
            
    df = pd.DataFrame(df_list)
    return df
    

def decode_additional_info(data):
    # Parse UTC
    utc_bytes =  data[1200:1206]
    year = int.from_bytes(utc_bytes[:1], byteorder='little') + 2000
    month = int.from_bytes(utc_bytes[1:2], byteorder='little')
    day = int.from_bytes(utc_bytes[2:3], byteorder='little')
    hour = int.from_bytes(utc_bytes[3:4], byteorder='little')
    minute = int.from_bytes(utc_bytes[4:5], byteorder='little')
    second = int.from_bytes(utc_bytes[5:6], byteorder='little')
    
    # Parse Timestamp
    timestamp_bytes  = data[1206:1210]
    timestamp_hex = timestamp_bytes.hex()
    timestamp = calculate_timestamp_ns(timestamp_hex)

    # Parse Factory
    factory_bytes = data[1210:1212]
    factory_info = factory_bytes.hex()
    echo_info = factory_info[:2]
    vendor_info = factory_info[2:4]

    # Print decoded information
    print("UTC: {}/{:02d}/{:02d} {:02d}:{:02d}:{:02d}".format(year, month, day, hour, minute, second))
    print("Timestamp: {} ns".format(timestamp))
    print("Factory - Echo Information (x37 strongest echo): {}".format(echo_info))
    print("Factory - Vendor Information (x20 C32, x10 C16): {}".format(vendor_info))

def plot_real_time(df,ax):
    x = df['Points_X']
    y = df['Points_Y']
    z = df['Points_Z']
    d = df['Distance']
    refl = df['Intensity']

    # Plot the new points
    # Chose the color: d for Distance and refl: Intensity 
    ax.scatter(x, y, z, c=d, cmap = 'viridis', marker='o')

    # Update the plot
    plt.draw()
    plt.pause(0.001)
    return ax


# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the socket to the local address and port
sock.bind((LIDAR_IP, PORT)) 
print("Connection done socket created! ")

figure = True # false if i do not want to plot
if figure:
    # Create a figure and axis for 3D plotting
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

try:
    count = 0


    # Receive data from the LiDAR sensor in real-time
    while True:
        start_time = datetime.now()
        count +=1 
        print(f"---------- {count} BEAM ---------")

        data, addr = sock.recvfrom(1260)

        # decode_additional_info(data)
        df = data_block_echo(data)
        # print(df)
        
        if figure:
            ax = plot_real_time(df,ax)

        ## Save Dataframe to CSV file
        # if count == 1: 
        #     df1 = data_block(data)
        #     # df1.to_csv("C:\\Users\\Giota.x\\Desktop\\LiDAR\\lidar_data.csv", index=False) 
        # elif count == 2:
        #     df = data_block(data)
        #     df = pd.concat([df1,df])
        #     # df.to_csv("C:\\Users\\Giota.x\\Desktop\\LiDAR\\lidar_data.csv", index=False) 
        # else:
        #     dfnew = data_block(data)
        #     df = pd.concat([df,dfnew])
        #     # df.to_csv("C:\\Users\\Giota.x\\Desktop\\LiDAR\\lidar_data.csv", index=False) 

        # Parse data
        end_time = datetime.now()
        time_difference = (end_time - start_time).total_seconds() * 10**3
        print("Time Interval (sec): ", time_difference) 

        if count == 10:
            # print("Done!")
            break


        
finally:
    # Close the socket
    sock.close()
    print("Socket is closed!")
    if figure:
        plt.show()