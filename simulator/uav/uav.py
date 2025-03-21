from simulator.utils.shared_imports import np,uav_type

class UAV:
    def __init__(self, uav_type: int,destinations: list[tuple] = [(0,0,0),(0,0,0)],inaccuracy = [0,0], start_time: int = 0,max_speed: int = 1,name: str = None):
        """
        Initialise a UAV object.
        inaccuracies: inaccuracy array. [0] = directional [1] = lateral [2] = vertical override
        """
        if uav_type < 0 or 15 < uav_type:
            raise ValueError("Invalid UAV type. Must be between 0 - 15.")
        self.uav_type = uav_type
        self.id = -1
        self.name = name
        self.destinations = destinations
        self.units_moved = 0
        self.times_waited = 0
        self.connected_asp = None
        self.current_position = self.destinations[0]
        self.previous_positions = []
        self.planned_route = (0,0,0,0)
        self.finished = False
        self.operational_status = 0
        self.inaccuracy = inaccuracy
        self.max_speed = max_speed if max_speed >= 1 else 1
        self.start_time = start_time
        self.made_first_move = False

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
        self.units_moved = 0
        self.finished = False
        self.current_position = self.destinations[0]
        self.times_waited = 0
        self.operational_status = 0
        self.made_first_move = False
        self.previous_positions = []
        self.planned_route = (0,0,0,0)

    def get_next_position(self):
        """Return the next waypoint in the planned route without moving."""
        next_index = self.units_moved + 1
        return self.planned_route[next_index][:3]

    def move(self):
        """
        Moves the UAV to the next position in its planned route.
        """
        self.operational_status = 1
        self.units_moved = (self.units_moved + 1)
        if self.current_position == self.planned_route[self.units_moved][:3]:
            self.operational_status = 0
            self.times_waited += 1
            return
        self.made_first_move = True
        self.previous_positions.append(self.current_position)
        self.current_position = self.planned_route[self.units_moved][:3]
    
    def assign_path(self,new_path:list[tuple]):
        """
        Assigns a new path to the UAV.
        """
        self.planned_route = new_path
        self.units_moved = 0
        self.current_position = self.planned_route[0]
    
    def wait(self):
        self.times_waited = getattr(self, 'times_waited', 0) + 1

    def is_finished(self):
        """
        Checks if the UAV has reached its final destination.
        """
        if not self.finished:
            if self.units_moved == len(self.planned_route) - 1:
                self.finished = True
                
        return self.finished
        