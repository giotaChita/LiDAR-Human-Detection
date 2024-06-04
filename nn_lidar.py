import numpy as np
import matplotlib.pyplot as plt
from matplotlib import figure
import socket
import pandas as pd 
import numpy as np
import matplotlib.pyplot as plt
from keras.models import load_model
import os 



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
    

# Function to apply restrictions to filter false detections
def filter(X, predictions):
    ...
    # return filtered_predictions

# Function to detect humans using the trained model
def detect_humans(X):
    # Predict probabilities of being a human
    probabilities = trained_model.predict(X)

    # Apply threshold to convert probabilities to binary predictions
    binary_predictions = (probabilities > 0.9).astype(int)
    return binary_predictions

    # Filter detections based on restrictions (e.g., height)
    # filtered_predictions = apply_restrictions(X, binary_predictions)
    # return filtered_predictions

# Function to plot detected humans in real-time
def plot_real_time(df, ax):
    X = df[['Points_X', 'Points_Y', 'Points_Z', 'Intensity']].values
    detected_humans = detect_humans(X)
    # Filter the dataframe to include only detected humans
    df_detected_humans = df[detected_humans == 1]
    x = df_detected_humans['Points_X']
    y = df_detected_humans['Points_Y']
    z = df_detected_humans['Points_Z']
    # Plot the detected humans with red color
    ax.scatter(x, y, z, c='red', marker='o')
    # Update the plot
    plt.draw()
    plt.pause(0.001)
    return ax


# Set environment variable to avoid OpenMP error
os.environ['KMP_DUPLICATE_LIB_OK'] = 'TRUE'

LIDAR_IP =  '192.168.1.102'
PORT = 2368 # MSOP

# Load the trained model
model_path ="C:\\Users\\Giota.x\\Desktop\\LiDAR\\trained_human_detection_model.h5"  # Path to your saved model
trained_model = load_model(model_path)

figure = True  # False if you do not want to plot
if figure:
    # Create a figure and axis for 3D plotting
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((LIDAR_IP, PORT))
print("Connection done socket created!")
try:
    count = 0
    while True:
        count += 1
        data, addr = sock.recvfrom(1260)
        df = data_block_echo(data)
        if figure:
            ax = plot_real_time(df, ax)
        if count == 100:
            break
finally:
    # Close the socket
    sock.close()
    print("Socket is closed!")
    if figure:
        plt.show()

