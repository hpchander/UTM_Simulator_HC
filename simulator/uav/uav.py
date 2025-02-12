from simulator.utils.shared_imports import np,byte_packer,byte_unpacker,uav_type

class UAV:
    def __init__(self, uav_type: int,destinations: list[tuple] = [(0,0,0)], horizontal_accuracy: int = 0,vertical_accuracy: int = 0,speed_accuracy: int = 0):
        """
        Initialise a UAV object.

        :param uav_list: List of current UAVs to which this UAV will be added
        :param uav_type: Type of UAV (e.g., 0 for quadcopter, 1 for fixed-wing)
        :param destinations: List of destinations (first is the origin)
        :param requested_time: Time at which the UAV is requested to begin flight
        """
        if uav_type < 0 or 15 < uav_type:
            raise ValueError("Invalid UAV type. Must be between 0 - 15.")
        self.id_types = byte_packer(uav_type,3)
        
        self.uav_type = uav_type
        self.id = -1
        self.destinations = destinations
        self.units_moved = 0
        self.connected_asp = None
        self.current_position = self.destinations[0]
        self.planned_route = self.destinations[0:]

        self.operational_status = 0
        self.horizontal_accuracy = horizontal_accuracy
        self.vertical_accuracy = vertical_accuracy
        self.speed_accuracy = speed_accuracy

    def __str__(self):
        return f"UAV {self.id} -\n\tType:{uav_type[self.uav_type]}\n\tCurrent Position:{self.current_position} \n\tNext Position:{self.next_position}\n"
    
    def connect_to_asp(self, asp_list: list):
        """
        Connects the UAV to the closest ASP.
        """
        min_distance = np.inf
        closest_asp = None
        for asp in asp_list:
            distance = np.linalg.norm(np.array(asp.position) - np.array(self.current_position))
            if distance < min_distance:
                min_distance = distance
                closest_asp = asp
        self.connected_asp = closest_asp

    def reset_uav(self):
        """
        Resets the UAV to its initial state.
        """
        self.current_position = self.destinations[0]
        self.planned_route = self.destinations[1:]
        self.operational_status = 0

    def move(self):
        """
        Moves the UAV to the next position in its planned route.
        """
        self.units_moved = (self.units_moved + 1) % len(self.planned_route)
        self.current_position = self.planned_route[self.units_moved]
        