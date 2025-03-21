from typing import Any, List, Optional
from simulator.utils.shared_imports import np, Math, PLT, colour_chart,WDG
import matplotlib.colors as mcolors
from simulator.environment.display import DisplayManager

class Environment:
    def __init__(self, world_data: np.ndarray,output_mode: int = 0,uav_mode: str = "wait-on-pass") -> None:
        self.world_data: np.ndarray = world_data
        self.render_world_data: np.ndarray = np.swapaxes(self.world_data, 1, 2)
        self.asp_list: List[Any] = []
        self.uav_list: List[Any] = []
        self.asp_coverage: np.ndarray = np.empty(self.world_data.shape, dtype=object)
        for idx in np.ndindex(self.asp_coverage.shape):
            self.asp_coverage[idx] = []
        self.map_asp_coverage()
        
        self.uav_map: np.ndarray = np.empty(self.world_data.shape, dtype=object)
        self.states: List[np.ndarray] = []
        self.collisions_map = None
        self.timestep: int = 0
        
        self.candidate_paths: dict = {}
        self.active_candidate_path: Optional[int] = None
        self.schedule_index: int = 0
        self.uav_mode = uav_mode
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
        self.uav_map = np.empty(self.world_data.shape, dtype=object)
        self.map_uavs()
        self.detect_collisions()
        
    def set_output_mode(self, mode: int) -> None:
        if mode not in [0,1,2,3]:
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
            else:
                print(f"Warning: No candidate path found for UAV {uav.id}. Check environment setup")
                return
        self.states = []
        self.timestep = 0
        self.uav_map = np.empty(self.world_data.shape, dtype=object)

        self.map_uavs()
        self.detect_collisions()

    def set_uav_mode(self, mode: str) -> None:
        if mode not in ["wait-on-pass", "follow-candidate-path"]:
            raise ValueError(f"Invalid UAV mode: {mode}")
        self.uav_mode = mode

    def detect_completed(self) -> bool:
        """
        Check if all UAVs have reached their final destination.
        """
        if all(uav.is_finished() for uav in self.uav_list):
            return True
        return False

    def run(self) -> None:
        """
        Run the simulation using next_timestep() until all UAVs have finished.
        """
        consecutive_no_moves = 0
        total_movements = 0
        self.map_uavs()
        collisions = self.detect_collisions()
        while not all(uav.is_finished() for uav in self.uav_list):
            moves_made,collision_count = self.next_timestep()
            total_movements += moves_made
            collisions += collision_count
            
            if moves_made == 0:
                consecutive_no_moves += 1
            else:
                consecutive_no_moves = 0
            
            if consecutive_no_moves > 2:
                #print("Deadlock detected. Exiting.")
                break
        
        total_waited = sum(uav.times_waited for uav in self.uav_list)
        if self.detect_completed():
            #print(f"Simulation finished in {self.timestep} timesteps. Total movements: {total_movements} Total time waited: {total_waited}")
            return {"success": True,"timesteps": self.timestep, "movements": total_movements, "waited": total_waited, "collisions": collisions}
        else:
            if (2 <= consecutive_no_moves): 
                #print(f"Simulation finished in {self.timestep} timesteps. Total movements: {total_movements} Total time waited: {total_waited}")
                return {"success": False,"timesteps": self.timestep, "movements": total_movements, "waited": total_waited, "collisions": collisions}

    def register_asp(self, new_asp: Any) -> None:
        """Register an ASP object."""
        taken_ids = [asp.id for asp in self.asp_list]
        for i in range(100):
            if i not in taken_ids:
                new_asp.id = i
                break
        if new_asp.id == -1:
            raise ValueError("Could not assign a unique ID to the ASP.")
        self.asp_list.append(new_asp)

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
                new_uav.name = f"-1"
            raise ValueError("Could not assign a unique ID to the UAV.")
        
        self.uav_list.append(new_uav)

    def display(self) -> None:
        self.map_uavs()
        self.detect_collisions()
        self.display_manager = DisplayManager(self)
        fig = PLT.figure(figsize=(15, 20))
        axis = self.display_manager.setup_display(fig, self.render_world_data)
        # Set up your next and reset buttons as before...
        next_button_axes = fig.add_axes([0.1, 0.25, 0.2, 0.1])
        next_button_axes.set_xticks([])
        next_button_axes.set_yticks([])
        next_button = WDG.Button(
            next_button_axes,
            'Next Timestep',
            color='lightblue',
            hovercolor='lightgreen'
        )
        next_button.on_clicked(lambda event: self.on_next_button_click(axis, fig))
        reset_button_axes = fig.add_axes([0.1, 0.5, 0.2, 0.1])
        reset_button_axes.set_xticks([])
        reset_button_axes.set_yticks([])
        reset_button = WDG.Button(
            reset_button_axes,
            'Reset Environment',
            color='lightblue',
            hovercolor='lightgreen'
        )
        reset_button.on_clicked(lambda event: self.on_reset_button_click(axis, fig))
        # Delegate display to the display manager:
        self.display_manager.start_display(axis, fig)
        
    def on_next_button_click(self, axis, fig) -> None:
        self.next_timestep()
        self.map_uavs()
        self.detect_collisions()
        self.display_manager.update_display(axis, fig)

    def on_reset_button_click(self, axis, fig) -> None:
        self.reset_environment()
        self.detect_collisions()
        self.display_manager.update_display(axis, fig)

    
    def next_timestep(self) -> int:
        if self.detect_completed():
            return 0,0
        self.states.append(np.copy(self.uav_map))
        self.timestep += 1
        moves_made = 0
        # Precompute current positions and intended positions
        current_positions = {uav.id: uav.current_position for uav in self.uav_list if not uav.is_finished() or self.timestep < uav.start_time}
        intended_positions = {uav.id: uav.get_next_position() for uav in self.uav_list if not uav.is_finished() or self.timestep < uav.start_time}
        if self.output_mode in [1,2]:
            print(f"Time {self.timestep}")
        # Build a candidate occupancy map: voxel -> list of UAV IDs planning to occupy that voxel.
        
        
        if self.uav_mode == "wait-on-pass":
            moves_made = self.WOP(intended_positions, current_positions)
        elif self.uav_mode == "follow-candidate-path":
            moves_made = self.follow_candidate_path(intended_positions)
        self.map_uavs()
        collision_count = self.detect_collisions()
        return moves_made,collision_count
    
    def WOP(self, intended_positions: dict, current_positions: dict) -> None:
        candidate_map = np.empty(self.uav_map.shape, dtype=object)
        for idx in np.ndindex(self.uav_map.shape):
            candidate_map[idx] = []
        for uav in self.uav_list:
            if self.timestep < uav.start_time:
                continue
            if uav.is_finished():
                continue
            footprint = self.compute_footprint(uav, intended_positions[uav.id])
            for voxel in footprint:
                candidate_map[voxel].append(uav.id)
        moves_made = self.wait_on_pass(intended_positions, candidate_map,current_positions)
        return moves_made
    

    def follow_candidate_path(self, intended_positions: dict) -> int:
        """
        Move each UAV to its next intended position based on provided paths
        """
        moves_made = 0
        for uav in self.uav_list:
            if uav.is_finished():
                continue
            if self.timestep <= uav.start_time:
                continue
            if uav.id not in intended_positions:
                continue
            if uav.current_position == intended_positions[uav.id]:
                if self.output_mode in [1,2]:
                    print(f"UAV {uav.name} waiting at {intended_positions[uav.id]}")
            else:
                if self.output_mode in [1,2]:
                    print(f"UAV {uav.name} moving to {intended_positions[uav.id]}")
                moves_made += 1
            uav.move()
        return moves_made

        
    def wait_on_pass(self,intended_positions,candidate_map,current_positions) -> None:
        # Resolve conflicts: For each UAV, check if any voxel in its footprint has multiple candidates.
        # If so, choose one UAV per voxel (e.g., with lowest ID) to be allowed.
        moves_made = 0
        allowed_moves = {}  # UAV id -> True if allowed to move
        for uav in self.uav_list:
            if uav.is_finished():
                allowed_moves[uav.id] = False
                continue
            if self.timestep <= uav.start_time:
                allowed_moves[uav.id] = False
                continue
            allowed_moves[uav.id] = True  # assume allowed until conflict found

            footprint = self.compute_footprint(uav, intended_positions[uav.id])
            for voxel in footprint:
                if len(candidate_map[voxel]) > 1:
                    allowed_uav = min(candidate_map[voxel])
                    if uav.id != allowed_uav:
                        allowed_moves[uav.id] = False
                        break
        for uav in self.uav_list:
            if uav.is_finished():
                continue
            for other in self.uav_list:
                if other.is_finished() or uav.id == other.id:
                    continue
                if (intended_positions[uav.id] == current_positions[other.id] and
                    intended_positions[other.id] == current_positions[uav.id]):
                    # both are trying to swap positions.
                    allowed_moves[uav.id] = False
                    allowed_moves[other.id] = False
        for uav in self.uav_list:
            if uav.is_finished():
                continue
            if allowed_moves[uav.id]:
                if allowed_moves[uav.id] and uav.current_position == intended_positions[uav.id]:
                    if self.output_mode in [1,2]:
                        print(f"UAV {uav.name} waiting at {intended_positions[uav.id]}")
                    uav.move()
                elif allowed_moves[uav.id]:
                    if self.output_mode in [1,2]:
                        print(f"UAV {uav.name} moving to {intended_positions[uav.id]}")
                    moves_made += 1
                    uav.move()
            else:
                if self.output_mode in [1,2]:
                    print(f"UAV {uav.name} waiting at {current_positions[uav.id]}")
                uav.wait()
        return moves_made
    
    def map_asp_coverage(self) -> None:
        """
        For each ASP, mark the voxels within its coverage radius.
        Each voxel within range will have the ASP's id appended to its list.
        """
        for asp in self.asp_list:
            position = asp.position  # Expected to be a tuple like (x, y, z)
            radius = asp.radius

            # Calculate the bounding box limits for the ASP's coverage
            x_min = max(0, int(position[0] - radius))
            x_max = min(self.world_data.shape[0], int(position[0] + radius) + 1)
            y_min = max(0, int(position[1] - radius))
            y_max = min(self.world_data.shape[1], int(position[1] + radius) + 1)
            z_min = max(0, int(position[2] - radius))
            z_max = min(self.world_data.shape[2], int(position[2] + radius) + 1)

            for x in range(x_min, x_max):
                for y in range(y_min, y_max):
                    for z in range(z_min, z_max):
                        # Compute Euclidean distance from the voxel to the ASP's position
                        distance = Math.sqrt(
                            (x - position[0]) ** 2 +
                            (y - position[1]) ** 2 +
                            (z - position[2]) ** 2
                        )
                        if distance <= radius:
                            self.asp_coverage[x, y, z].append(asp.id)    

    def compute_footprint(self, uav, pos: tuple) -> List[tuple]:
        """
        Compute the footprint of a UAV at a given position.
        The footprint is the set of voxels that the UAV may occupy.
        """
        footprint = []
        radius = uav.inaccuracy[0]
        shape = uav.inaccuracy[1]
        if shape == 0: #no corners
            x_min = max(0, int(pos[0] - radius))
            x_max = min(self.world_data.shape[0], int(pos[0] + radius) + 1)
            y_min = max(0, int(pos[1] - radius))
            y_max = min(self.world_data.shape[1], int(pos[1] + radius) + 1)
            z_min = max(0, int(pos[2] - radius))
            z_max = min(self.world_data.shape[2], int(pos[2] + radius) + 1)
            for x in range(x_min, x_max):
                for y in range(y_min, y_max):
                    for z in range(z_min, z_max):
                        distance = Math.sqrt(
                            (x - pos[0]) ** 2 +
                            (y - pos[1]) ** 2 +
                            (z - pos[2]) ** 2
                        )
                        if distance <= radius:
                            footprint.append((x, y, z))
        elif shape == 1: #square
            x_min = max(0, int(pos[0] - radius))
            x_max = min(self.world_data.shape[0], int(pos[0] + radius) + 1)
            y_min = max(0, int(pos[1] - radius))
            y_max = min(self.world_data.shape[1], int(pos[1] + radius) + 1)
            z_min = max(0, int(pos[2] - radius))
            z_max = min(self.world_data.shape[2], int(pos[2] + radius) + 1)
            for x in range(x_min, x_max):
                for y in range(y_min, y_max):
                    for z in range(z_min, z_max):
                        footprint.append((x, y, z))
        else:
            raise ValueError("Invalid shape type")
        
        return footprint





    
    
    def map_uavs(self) -> None:
        """
        Clear and repopulate the UAV map based on the current positions and inaccuracy of each UAV.
        
        For each UAV in self.uav_list, determine its footprint (i.e. the set of voxels it may occupy)
        based on its current_position and its accuracy attributes. Then, for each voxel in that footprint,
        append the UAV's identifier to the list stored in that voxel.
        """
        for idx in np.ndindex(self.uav_map.shape):
            self.uav_map[idx] = []

        for uav in self.uav_list:
            if self.timestep < uav.start_time:
                continue
            if uav.is_finished():
                continue
            footprint = self.compute_footprint(uav, uav.current_position)
            for voxel in footprint:
                self.uav_map[voxel].append(uav.id)

        

    def detect_collisions(self) -> int:
        """
        Check for collisions between UAVs and the world.
        A collision is marked in self.collisions_map (a boolean array)
        if either:
        - More than one UAV occupies the same voxel, or
        - A UAV occupies a voxel that contains an obstacle in the world data.
        """
        self.collisions_map = np.zeros(self.world_data.shape, dtype=bool)

        for idx in np.ndindex(self.uav_map.shape):
            if len(self.uav_map[idx]) > 1:
                for uav_id in self.uav_map[idx]:
                    if self.uav_list[uav_id].is_finished() == True:
                        continue
                    if self.uav_list[uav_id].start_time < self.timestep:
                        continue
                if self.output_mode in [1,2]:
                    print(f"Time {self.timestep}")
                    print(f"Collision detected at {idx} between UAVs {self.uav_map[idx]}")
                self.collisions_map[idx] = True
            elif self.world_data[idx] == 1 and len(self.uav_map[idx]) > 0:
                self.collisions_map[idx] = True
            else:
                self.collisions_map[idx] = False
        return np.sum(self.collisions_map)

    def collision_count(self) -> int:
        """
        Count the number of collisions in the current state.
        """
        return np.sum(self.collisions_map)
