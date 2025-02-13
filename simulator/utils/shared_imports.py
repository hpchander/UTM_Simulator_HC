import numpy as np
import math as Math
import matplotlib.pyplot as PLT
import matplotlib.widgets as WDG
from abc import ABC, abstractmethod

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

    
colour_chart = ["red", "blue", "green", "yellow", "purple", "orange", "pink", "brown", "black"]
    
uav_type = ["Not Declared / None", "Aeroplane", "Helicopter", "Gyroplane", "Hybrid Lift", "Ornithopter", "Glider", "Kite", "Free Balloon", "Captive Balloon", "Airship", "Parachute", "Rocket", "Tethered Powered Aircraft", "Ground Obstacle", "Other"] 