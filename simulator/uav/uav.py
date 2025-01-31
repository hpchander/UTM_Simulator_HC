from simulator.utils.shared_imports import np,byte_packer,byte_unpacker,return_uav_type

class UAV:
    def __init__(self, uav_list: list, uav_type: int,destinations: list[tuple] = [(0,0,0)],requested_time: float = 0.0, horizontal_accuracy: int = 0,vertical_accuracy: int = 0,speed_accuracy: int = 0):
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
        taken_ids = [uav.id for uav in uav_list]
        for i in range(10000):
            if i not in taken_ids:
                self.id = i
                break
        if self.id == None:
            raise ValueError("Could not assign a unique ID to the UAV.")
            self.id = -1
        
        self.destinations = destinations
        self.requested_time = requested_time
        self.connected_asp = None
        self.position = destinations[0]
        self.operational_status = 0
        self.horizontal_accuracy = horizontal_accuracy
        self.vertical_accuracy = vertical_accuracy
        self.speed_accuracy = speed_accuracy
        uav_list.append(self)

    def __str__(self):
        return f"UAV {self.id} -\n\tType:{return_uav_type(self.uav_type)}\n\tCurrent Position:{self.position}"
    
    def connect_to_asp(self, asp_list: list):
        """
        Connects the UAV to the closest ASP.

        :param asp_list: List of ASPs
        """
        min_distance = np.inf
        closest_asp = None
        for asp in asp_list:
            distance = np.linalg.norm(np.array(asp.position) - np.array(self.destinations[0]))
            if distance < min_distance:
                min_distance = distance
                closest_asp = asp
        self.connected_asp = closest_asp