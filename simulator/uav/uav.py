from simulator.utils.shared_imports import np, uav_type, Math, Pos, State
from simulator.utils.config import DEFAULT_MAX_SPEED, DEFAULT_UAV_INACCURACY
from typing import List

class UAV:
    def __init__(
        self,
        uav_type: int,
        destinations: List[Pos] = None,
        inaccuracy = DEFAULT_UAV_INACCURACY,
        start_time: int = 0,
        max_speed: int = DEFAULT_MAX_SPEED,
        name: str = "UNNAMED_UAV",
    ):
        """
        Initialise a UAV object.
        - destinations: list of Pos objects, where the first element is the spawn position.
        - inaccuracy: inaccuracy array. [0] = directional, [1] = lateral (others can be added as needed).
        - start_time: the simulation time at which the UAV spawns.
        - max_speed: maximum number of moves per timestep; minimum value is 1.
        """
        # Set default destinations if none provided.
        if destinations is None:
            destinations = [Pos(0, 0, 0), Pos(0, 0, 0)]
            
        if uav_type < 0 or uav_type > 15:
            raise ValueError("Invalid UAV type. Must be between 0 - 15.")
        self.uav_type: int = uav_type
        self.id: int = -1
        self.name: str = name
        self.destinations: list[Pos] = destinations
        self.inaccuracy: list[int] = inaccuracy
        self.max_speed: int = max_speed if max_speed >= 1 else 1
        self.start_time: int = start_time
        self.minimum_start_time: int = start_time
        ## UAV state variables
        self.units_moved: int = 0
        self.times_waited: int = 0
        # Set current_position as a Pos object (from the first destination)
        self.current_position: Pos = self.destinations[0]
        self.previous_positions: List[Pos] = []
        self.positions_last_timestep: List[Pos] = []
        # Initialize the planned route with a State based on the UAV's spawn position and start time.
        self.planned_route: List[State] = [
            State(self.destinations[0].x, self.destinations[0].y, self.destinations[0].z, self.start_time)
        ]
        self.finished: bool = False
        self.time_finished: int = -1
        self.traversed_positions: List[Pos] = []

    def __str__(self):
        return (f"UAV {self.id} -\n\tType: {uav_type[self.uav_type]}\n\t"
                f"Current Position: {self.current_position}\n")


    def reset_uav(self):
        """
        Resets the UAV to its initial state.
        """
        self.units_moved = 0
        self.finished = False
        self.time_finished = -1
        self.current_position = self.destinations[0]
        self.times_waited = 0
        self.previous_positions = []
        self.planned_route = [
            State(self.destinations[0].x, self.destinations[0].y, self.destinations[0].z, self.start_time)
        ]
        self.traversed_positions = []
        self.positions_last_timestep = []
    def get_next_position(self) -> Pos:
        """
        Return the next waypoint in the planned route as a Pos object without moving.
        """
        next_index = self.units_moved + 1
        if next_index < len(self.planned_route):
            next_state = self.planned_route[next_index]
            return Pos(next_state.x, next_state.y, next_state.z)
        else:
            return self.current_position

    def move(self, timestep: int):
        """
        Advances the UAV along its planned route for the current simulation timestep.
        The planned_route is a list of State objects.
        In one simulation timestep, the UAV can take up to max_speed moves.

        If the next node is a waiting move (i.e. the spatial coordinate is the same as the current position
        but the time stamp increases), the UAV will record the wait and end its movement for this timestep.
        """
        # Add the current position to the list
        self.traversed_positions = []  # Reset the list of traversed positions
        steps_taken = 0

        # Process up to max_speed moves, or until no further nodes exist.
        while steps_taken < self.max_speed and (self.units_moved + 1) < len(self.planned_route):
            next_state = self.planned_route[self.units_moved + 1]
            next_position = Pos(next_state.x, next_state.y, next_state.z)
            next_time = next_state.time

            if timestep < next_time:
                # The next state is not yet available (in the future); the UAV waits.
                self.times_waited += 1
                return

            # Record the current position before moving.
            self.traversed_positions.append(self.current_position)
            self.previous_positions.append(self.current_position)
            self.units_moved += 1
            self.current_position = next_position
            steps_taken += 1

            # Check if the UAV has reached the final destination.
            final_state = self.planned_route[-1]
            final_position = Pos(final_state.x, final_state.y, final_state.z)
            if self.current_position == final_position:
                self.finished = True
                self.time_finished = timestep
                break  # UAV reached the destination

    def assign_path(self, new_path: List[State]):
        """
        Assigns a new path to the UAV.
        """
        self.planned_route = new_path
        self.units_moved = 0
        # Set the current position to the starting state of the new planned route.
        if len(self.planned_route) > 0:
            start_state = self.planned_route[0]
            self.current_position = Pos(start_state.x, start_state.y, start_state.z)
    
    def wait(self):
        self.times_waited += 1

    def is_finished(self) -> bool:
        """
        Checks if the UAV has reached its final destination.
        """
        if not self.finished:
            if self.units_moved >= len(self.planned_route):
                self.finished = True
                self.time_finished = Math.ceil((self.units_moved + self.times_waited) / self.max_speed) + self.start_time
                return self.finished
            if self.current_position == self.destinations[-1]:
                other_goals = self.destinations[1:-1]
                for goal in other_goals:
                    if goal not in self.previous_positions:
                        self.finished = False
                        return self.finished
                self.finished = True
        return self.finished
