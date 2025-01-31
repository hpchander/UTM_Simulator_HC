import numpy as np
import math as Math
import matplotlib.pyplot as MLP

def byte_packer(high,low):
    """
    Convert two 4 bit integers into a single 8 bit integer.
    """
    return (high << 4) | low

def byte_unpacker(byte):
    """
    Convert an 8 bit integer into two 4 bit integers.
    """
    return (byte >> 4), (byte & 0x0F)

def colour_chart(int):
    """
    Convert a number into a colour.
    """
    if int % 10 == 0:
        return "red"
    elif int % 10 == 1:
        return "blue"
    elif int % 10 == 2:
        return "green"
    elif int % 10 == 3:
        return "yellow"
    elif int % 10 == 4:
        return "purple"
    elif int % 10 == 5:
        return "orange"
    elif int % 10 == 6:
        return "pink"
    elif int % 10 == 7:
        return "brown"
    elif int % 10 == 8:
        return "black"
    
def return_uav_type(int):
    """
    Convert a number into a UAV type.
    """
    if int == 0:
        return "Not Declared / None"
    elif int == 1:
        return "Aeroplane"
    elif int == 2:
        return "Helicopter"
    elif int == 3:
        return "Gyroplane"
    elif int == 4:
        return "Hybrid Lift"
    elif int == 5:
        return "Ornithopter"
    elif int == 6:
        return "Glider"
    elif int == 7:
        return "Kite"
    elif int == 8:
        return "Free Balloon"
    elif int == 9:
        return "Captive Balloon"
    elif int == 10:
        return "Airship"
    elif int == 11:
        return "Parachute"
    elif int == 12:
        return "Rocket"
    elif int == 13:
        return "Tethered Powered Aircraft"
    elif int == 14:
        return "Ground Obstacle"
    elif int == 15:
        return "Other"
    else:
        return "Unknown UAV Type"