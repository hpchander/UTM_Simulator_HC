from typing import Any, List, Optional
from simulator.utils.shared_imports import np, Math
from simulator.environment.display import DisplayManager
from simulator.uav.uav import UAV
from simulator.utils.shared_imports import Pos, State, TMState
import simulator.utils.config as cfg

class Environment:
    def __init__(self, world_data: np.ndarray, output_mode: int = 0) -> None:
        self.world_data: np.ndarray = world_data
        self.render_world_data: np.ndarray = np.swapaxes(self.world_data, 1, 2)
        self.uav_list: List[UAV] = []        
        self.reservations = []
        self.uav_map: dict = {}
        self.states: List[np.ndarray] = []
        self.collisions_map = {}
        self.timestep: int = 0
        self.candidate_paths: dict = {}
        self.active_candidate_path: Optional[int] = None
        self.schedule_index: int = 0
        self.output_mode = output_mode
        self.display_manager = None

    def reset_environment(self) -> None:
        """
        Reset the environment's simulation state to its initial conditions.
        """
        self.timestep = 0
        for uav in self.uav_list:
            uav.reset_uav()
        self.states = []
        self.uav_map = {}
        self.map_uavs()
        self.detect_collisions()
        
    def set_output_mode(self, mode: int) -> None:
        if mode not in [0, 1, 2, 3,4,5,6]:
            raise ValueError(f"Invalid output mode: {mode}")
        self.output_mode = mode

    def add_candidate(self, candidate_id: int, schedule: dict) -> None:
        """
        Add a candidate schedule.
        
        schedule: dict where keys are UAV IDs and values are lists of waypoints.
        """
        self.candidate_paths[candidate_id] = schedule

    def set_active_candidate_path(self, candidate_id: int) -> None:
        """
        Set the active candidate path and update each UAV's planned_route accordingly.
        """
        if candidate_id not in self.candidate_paths:
            raise ValueError(f"Candidate path {candidate_id} not found.")
        self.active_candidate_path = candidate_id
        candidate = self.candidate_paths[candidate_id]
        for uav in self.uav_list:
            # If a candidate route exists for the UAV, use it.
            if uav.id in candidate:
                uav.planned_route = candidate[uav.id]
                uav.start_time = uav.planned_route[0].time
            else:
                print(f"Warning: No candidate path found for UAV {uav.id}. Check environment setup")
                return
        self.states = []
        self.timestep = 0
        self.uav_map = {}
        self.map_uavs()
        self.detect_collisions()


    def detect_completed(self) -> bool:
        """
        Check if all UAVs have reached their final destination.
        """
        if len(self.uav_list) == 0:
            return True
        return all(uav.is_finished() for uav in self.uav_list)

    def run(self) -> dict:
        """
        Run the simulation using next_timestep() until all UAVs have finished.
        Accumulates movements and collisions, then returns a summary dictionary.
        """
        consecutive_no_moves = 0
        total_movements = 0
        # Initialize collision counters
        self.map_uavs()
        # Update collisions map
        self.detect_collisions()
        total_world_collisions, total_uav_collisions = self.collision_count()
        
        while not all(uav.finished for uav in self.uav_list):
            moves_made, collision_count = self.next_timestep()  # collision_count is a tuple (world, uav)
            total_movements += moves_made
            total_world_collisions += collision_count[0]
            total_uav_collisions += collision_count[1]
            
            if moves_made == 0:
                consecutive_no_moves += 1
                if consecutive_no_moves > 10:
                    break
            else:
                consecutive_no_moves = 0
        spawn_differences = 0
        for uav in self.uav_list:
            spawn_differences += uav.start_time - uav.minimum_start_time

        #total_waited = sum(uav.times_waited for uav in self.uav_list)
        success = self.detect_completed()
        return {
            "success": success,
            "timesteps": self.timestep,
            "movements": total_movements,
            "waited": spawn_differences,
            "collisions": (total_world_collisions, total_uav_collisions)
        }


    def register_uav(self, new_uav: Any) -> None:
        """Register a UAV object."""
        taken_ids = [uav.id for uav in self.uav_list]
        for i in range(100):
            if i not in taken_ids:
                new_uav.id = i
                if new_uav.name is None:
                    new_uav.name = f"{i}"
                break
        if new_uav.id == -1:
            if new_uav.name is None:
                new_uav.name = "-1"
            raise ValueError("Could not assign a unique ID to the UAV.")
        self.uav_list.append(new_uav)

    def set_reservations(self, reservations) -> None:
        """
        Set the reservations for the environment.
        Reservations symbolize prebooked flight paths / other air traffic.
        Consists of [(x,y,z,t)]
        """
        self.reservations = reservations
    
    def display(self) -> None:
        self.display_manager = DisplayManager(self)
    
    def next_timestep(self) -> tuple:
        """
        Advances the simulation by one timestep.
        Returns a tuple: (moves_made, (world_collisions, uav_collisions)).
        """
        if self.detect_completed():
            return 0, (0, 0)
        self.timestep += 1
        moves_made = 0
        # Precompute current and intended positions
        current_positions = {
            uav.id: uav.current_position
            for uav in self.uav_list
            if uav.start_time <= self.timestep and (not uav.finished or self.timestep <= uav.time_finished + 1)
        }
        intended_positions = {
            uav.id: uav.get_next_position()
            for uav in self.uav_list
            if uav.start_time <= self.timestep and (not uav.finished or self.timestep <= uav.time_finished + 1)
        }

        if self.output_mode in [1, 2]:
            print(f"Time {self.timestep}")
        

        moves_made = self.follow_candidate_path(intended_positions)
        
        self.map_uavs()
        # Update collisions map and get aggregated counts
        self.detect_collisions()
        return moves_made, self.collision_count()
    
    def follow_candidate_path(self, intended_positions: dict) -> int:
        """
        Move each UAV to its next intended position based on the candidate paths.
        """
        moves_made = 0
        for uav in self.uav_list:
            if uav.finished or self.timestep <= uav.start_time or uav.id not in intended_positions:
                continue
            if uav.current_position == intended_positions[uav.id]:
                if self.output_mode in [1, 2]:
                    print(f"UAV {uav.name} waiting at {intended_positions[uav.id].x},{intended_positions[uav.id].y},{intended_positions[uav.id].z}")
            else:
                if self.output_mode in [1, 2]:
                    print(f"UAV {uav.name} moving to ({intended_positions[uav.id].x},{intended_positions[uav.id].y},{intended_positions[uav.id].z})")
                moves_made += 1
            uav.move(self.timestep)
        return moves_made

    def compute_footprint(self, uav, pos: tuple) -> List[tuple]:
        """
        Compute the footprint of a UAV at a given position.
        The footprint is the set of voxels that the UAV may occupy.
        """
        footprint = []
        radius = uav.inaccuracy[0]
        shape = uav.inaccuracy[1]
        if shape == 0:  # no corners
            x_min = max(0, int(pos.x - radius))
            x_max = min(self.world_data.shape[0], int(pos.x + radius) + 1)
            y_min = max(0, int(pos.y - radius))
            y_max = min(self.world_data.shape[1], int(pos.y + radius) + 1)
            z_min = max(0, int(pos.z - radius))
            z_max = min(self.world_data.shape[2], int(pos.z + radius) + 1)
            for x in range(x_min, x_max):
                for y in range(y_min, y_max):
                    for z in range(z_min, z_max):
                        distance = Math.sqrt(
                            (x - pos.x) ** 2 +
                            (y - pos.y) ** 2 +
                            (z - pos.z) ** 2
                        )
                        if distance <= radius:
                            footprint.append((x, y, z))
        elif shape == 1:  # square
            x_min = max(0, int(pos.x - radius))
            x_max = min(self.world_data.shape[0], int(pos.x + radius) + 1)
            y_min = max(0, int(pos.y - radius))
            y_max = min(self.world_data.shape[1], int(pos.y + radius) + 1)
            z_min = max(0, int(pos.z - radius))
            z_max = min(self.world_data.shape[2], int(pos.z + radius) + 1)
            for x in range(x_min, x_max):
                for y in range(y_min, y_max):
                    for z in range(z_min, z_max):
                        footprint.append((x, y, z))
        else:
            raise ValueError("Invalid shape type")
        return footprint
    
    def map_uavs(self) -> None:
        """
        Build a UAV map (using a dictionary for scalability) where for each UAV,
        we compute the union of its footprint over all positions traversed in the current turn.
        """
        self.uav_map = {}
        for uav in self.uav_list:
            if self.timestep < uav.start_time or (uav.finished and uav.time_finished < self.timestep):
                continue
            positions = getattr(uav, 'traversed_positions', []) + [uav.current_position]
            union_footprint = set()
            for pos in positions:
                union_footprint.update(self.compute_footprint(uav, pos))
            for voxel in union_footprint:
                self.uav_map.setdefault(voxel, []).append(uav.id)

    def detect_collisions(self) -> dict:
        """
        Iterate over UAV footprints and check for collisions.
        For each voxel, determine if a world collision and/or UAV collision occurs.
        Returns a dictionary mapping voxel locations to a tuple:
        (world_collision, uav_collision)
        """
        collisions = {}
        for voxel, uav_ids in self.uav_map.items():
            uav_collision = len(uav_ids) > 1
            world_collision = (self.world_data[voxel] == 1 and len(uav_ids) > 0)
            key = State(voxel[0], voxel[1], voxel[2], self.timestep)

            if key in self.reservations:
                uav_collision += 1
            if world_collision or uav_collision:
                if self.output_mode in [1, 2]:
                    print(f"Time {self.timestep}")
                    print(f"Collision detected at {voxel}: world collision = {world_collision}, UAV collision = {uav_collision} with UAVs {uav_ids}")
                collisions[voxel] = (world_collision, uav_collision)
        self.collisions_map = collisions
        return collisions

    def collision_count(self) -> tuple:
        """
        Count the number of collisions in the current state.
        Returns a tuple: (number of world collisions, number of UAV collisions)
        """
        collisions = self.collisions_map.values()
        world_collisions = sum(1 for collision in collisions if collision[0])
        uav_collisions = sum(1 for collision in collisions if collision[1])
        return world_collisions, uav_collisions
