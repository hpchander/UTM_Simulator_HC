import numpy as np
import math as Math
import matplotlib.pyplot as PLT
import matplotlib.widgets as WDG
import csv
from dataclasses import dataclass
from prettytable import PrettyTable
import pytest

#For UAVs to pull colour name strings for display
colour_chart = ["red", "blue", "green","purple", "orange","teal","gold", "pink", "brown","lime","crimson","navy","olive","black"]
#Corrolates colours in above list to ANSI escape codes for text
ansi_colours = {
    "red":     "\033[31m",
    "blue":    "\033[34m",
    "green":   "\033[32m",
    "purple":  "\033[35m",
    "orange":  "\033[33m",
    "teal":    "\033[36m",
    "gold":    "\033[93m",
    "pink":    "\033[95m",
    "brown":   "\033[33m",
    "lime":    "\033[92m",
    "crimson": "\033[91m",
    "navy":    "\033[94m",
    "olive":   "\033[90m",
    "black":   "\033[0m",
    "reset":   "\033[0m"     # Reset to default
}
#UAV type list sourced from "https://cdn.standards.iteh.ai/samples/105468/200257eeb5fa47a78e98a17ac132ba03/ASTM-F3411-19.pdf"
uav_type = ["Not Declared / None", "Aeroplane", "Helicopter", "Gyroplane", "Hybrid Lift", "Ornithopter", "Glider", "Kite", "Free Balloon", "Captive Balloon", "Airship", "Parachute", "Rocket", "Tethered Powered Aircraft", "Ground Obstacle", "Other"] 

@dataclass(frozen=True)
class Pos:
    """
    3D Position.
    x, y, z: int
    Immutable
    """
    x: int
    y: int
    z: int

    def __str__(self):
        return f"({self.x}, {self.y}, {self.z})"
    
    def to_array(self):
        return np.array([self.x, self.y, self.z])
    
    def __iter__(self):
        return iter((self.x, self.y, self.z))


@dataclass(frozen=True)
class State:
    """
    4D State of a UAV.
    x, y, z, time: int
    Immutable
    """
    x: int
    y: int
    z: int
    time: int

    def __str__(self):
        return f"({self.x}, {self.y}, {self.z}, {self.time})"

    def to_array(self):
        return np.array([self.x, self.y, self.z, self.time])
    
    def __iter__(self):
        return iter((self.x, self.y, self.z, self.time))

@dataclass(frozen=True)
class TMState:
    """State of a UAV including the time and number of moves used this turn.
    x, y, z, time, moves_used: int
    Immutable
    """
    x: int
    y: int
    z: int
    time: int
    moves_used: int

    def __str__(self):
        return f"({self.x}, {self.y}, {self.z}, {self.time}, {self.moves_used})"
    
    def to_array(self):
        return np.array([self.x, self.y, self.z, self.time, self.moves_used])
    
    def __iter__(self):
        return iter((self.x, self.y, self.z, self.time, self.moves_used))
    
def get_uav_colour(uav_id: int) -> str:
    """
    Get the string of a colour for a UAV based on its ID.
    """
    return colour_chart[uav_id % len(colour_chart)]

def get_ansi_colour(uav_id: int) -> str:
    """
    Get the ANSI escape string of a colour for a UAV based on its ID.
    """
    return ansi_colours[colour_chart[uav_id % len(colour_chart)]]