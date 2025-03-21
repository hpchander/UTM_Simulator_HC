import numpy as np
import math as Math
import matplotlib.pyplot as PLT
import matplotlib.widgets as WDG
from abc import ABC, abstractmethod
import csv
    
colour_chart = ["red", "blue", "green", "yellow", "purple", "orange", "pink", "brown", "black"]

uav_type = ["Not Declared / None", "Aeroplane", "Helicopter", "Gyroplane", "Hybrid Lift", "Ornithopter", "Glider", "Kite", "Free Balloon", "Captive Balloon", "Airship", "Parachute", "Rocket", "Tethered Powered Aircraft", "Ground Obstacle", "Other"] 