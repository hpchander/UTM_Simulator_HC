from typing import Any, List, Optional
from simulator.utils.shared_imports import np, Math

class ASP:
    def __init__(self, name: str = "ASP",world_data: np.ndarray = [], origin: tuple = (0,0,0), radius: int = 1) -> None:
        """
        Initialise an Airspace Service Provider (ASP) object.

        :param name: Name of the ASP (string)
        :param origin: 3D coordinates of the origin point (tuple)
        :param shape: Shape of the ASP's area (e.g., "square") (string)
        :param radius: Radius defining the ASP's coverage (int or float)
        :param asp_list: List of current ASPs to which this ASP will be added
        """
        self.name: str = name
        self.position: tuple = origin
        self.radius: int = radius
        self.id: int = -1
        self.schedule = []
        self.world_data = world_data
        

    def __str__(self):
        return f"ASP {self.id} -\n\tName:{self.name}\n\tPosition:{self.position}\n\tRadius:{self.radius}"

    def connected_uav_positions(self,uav_list):
        for uav in uav_list:
            print(uav.id," ", uav.current_position)

    