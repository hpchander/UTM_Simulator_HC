"""Path Planner for 4D A* pathfinding in a 3D environment."""
import heapq
from typing import List,Dict
import simulator.utils.config as cfg
from simulator.utils.shared_imports import np, Math, State, Pos
from simulator.uav.uav import UAV
from simulator.environment.environment import Environment



def remove_same_timestep_oscillations(path: List[State]) -> List[State]:
    """
    Collapse any A→B→A oscillation occurring all at the same time.
    """
    cleaned: List[State] = []
    for s in path:
        # if last two entries + this one share time and form A→B→A, drop the middle
        if len(cleaned) >= 2 \
           and cleaned[-1].time == s.time == cleaned[-2].time \
           and (cleaned[-2].x, cleaned[-2].y, cleaned[-2].z) == (s.x, s.y, s.z):
            cleaned.pop()
        else:
            cleaned.append(s)
    return cleaned

class ObliviousPlanner:
    """
    A simplistic path planner that moves each UAV along
    an axis‐aligned Manhattan route from its start to each goal,
    up to max_speed moves per timestep, ignoring obstacles.
    """
    def __init__(self):
        pass

    def plan_path(self, environment: Environment):
        """
        Plan a straight‐line (Manhattan) path for each UAV.
        Returns: (candidate_paths, delay_counts, searched_counts)
        """
        candidate_paths: Dict[int, List[State]] = {}
        # zero delay for every UAV
        delay_counts: Dict[int,int] = {uav.id: 0 for uav in environment.uav_list}
        searched_counts: Dict[int,int] = {uav.id: 0 for uav in environment.uav_list}

        for uav in environment.uav_list:
            raw: List[Pos] = []
            x, y, z = uav.destinations[0]
            raw.append(Pos(x, y, z))
            for wp in uav.destinations[1:]:
                while x != wp.x:
                    x += 1 if wp.x > x else -1
                    raw.append(Pos(x, y, z))
                while y != wp.y:
                    y += 1 if wp.y > y else -1
                    raw.append(Pos(x, y, z))
                while z != wp.z:
                    z += 1 if wp.z > z else -1
                    raw.append(Pos(x, y, z))

            path: List[State] = []
            for idx, pos in enumerate(raw):
                t = uav.start_time + (idx // uav.max_speed)
                path.append(State(pos.x, pos.y, pos.z, t))

            candidate_paths[uav.id] = path
            searched_counts[uav.id] = len(raw) - 1

        return candidate_paths, delay_counts, searched_counts

class AStarPlanner():
    """
    A* path planner.
    """
    cached_footprints = {}
    
    def __init__(
            self,
            heuristics: Dict[str,bool] = cfg.DEFAULT_HEURISTICS,
            beam_width: int = cfg.DEFAULT_BEAM_WIDTH,
            ordering: Dict[str,int] = cfg.DEFAULT_ORDERING,
            g_score_multiplier: float = cfg.DEFAULT_G_SCORE_MULTIPLIER,
            g_score_increment: float = cfg.DEFAULT_G_SCORE_INCREMENT,
            disable_collisions: bool = cfg.ENABLE_PARTIAL_COLLISION_DISABLER,
            enable_indirect_world_collisions: bool = cfg.ENABLE_INDIRECT_WORLD_COLLISIONS
            ):
        """
        Heuristics - Dict[heuristic_name: str, enabled: bool]
        beam_width - int, number of nodes to keep in the open set
        ordering - Dict[ordering_name: int], ordering of UAVs to process
        g_score_multiplier - float, multiplier for g_score
        g_score_increment - float, increment for g_score
        disable_collisions - bool, disable partial collision checking"""
        self.heuristics = heuristics
        self.beam_width = beam_width
        if (ordering == None):
            ordering = cfg.DEFAULT_ORDERING
        self.ordering = ordering
        self.g_score_multiplier = g_score_multiplier
        self.g_score_increment = g_score_increment
        self.disable_collisions = disable_collisions
        self.enable_indirect_world_collisions = enable_indirect_world_collisions

    # def get_uav_locations(self, candidate_paths: dict, t: int) -> dict:
    #     """Return a mapping from UAV ids to their candidate State at time t.
    #     Deprecated
    # """
    #     uav_locations = {}
    #     for uav_id, path in candidate_paths.items():
    #         if t < len(path):
    #             uav_locations[uav_id] = path[t]
    #     return uav_locations

    def compute_footprint(self, uav: UAV, pos: Pos) -> List[State]:
        """Compute the set of voxels that the UAV may occupy at a given position.
           Returns a list of Pos objects representing the footprint."""
        footprint = []
        radius = uav.inaccuracy[0]
        shape = uav.inaccuracy[1]
        if shape == 0:  # circular footprint
            x_min = max(0, int(pos.x - radius))
            x_max = int(pos.x + radius) + 1
            y_min = max(0, int(pos.y - radius))
            y_max = int(pos.y + radius) + 1
            z_min = max(0, int(pos.z - radius))
            z_max = int(pos.z + radius) + 1
            for x in range(x_min, x_max):
                for y in range(y_min, y_max):
                    for z in range(z_min, z_max):
                        distance = Math.sqrt((x - pos.x)**2 + (y - pos.y)**2 + (z - pos.z)**2)
                        if distance <= radius:
                            footprint.append(Pos(x, y, z))
        elif shape == 1:  # square footprint
            x_min = max(0, int(pos.x - radius))
            x_max = int(pos.x + radius) + 1
            y_min = max(0, int(pos.y - radius))
            y_max = int(pos.y + radius) + 1
            z_min = max(0, int(pos.z - radius))
            z_max = int(pos.z + radius) + 1
            for x in range(x_min, x_max):
                for y in range(y_min, y_max):
                    for z in range(z_min, z_max):
                        footprint.append(Pos(x, y, z))
        else:
            raise ValueError("Invalid shape type")
        return footprint

    def create_obstacle_dict(self, environment: Environment) -> dict:
        """Return a dict mapping (x,y,z) positions occupied by obstacles from world_data.
        If an obstacle is present, the value is True.
        """
        obstacles = {}
        obs_coords = np.argwhere(environment.world_data==1)
        obstacles = {tuple(coord):True for coord in obs_coords}
        return obstacles

    def add_footprints_to_reservations(
        self,
        reservations: dict,
        footprint: List[State],
        uav_id: int,
        time: int
        ) -> dict:
        """Add each voxel from footprint to reservations keyed by (x,y,z,time)."""
        for voxel in footprint:
            key = (voxel.x, voxel.y, voxel.z, time)
            if key in reservations:
                if uav_id not in reservations[key]:
                    reservations[key].append(uav_id)
            else:
                reservations[key] = [uav_id]
        return reservations

    def add_reservation(self, reservations: dict, uav: UAV, path: List[State]) -> dict:
        """For each state in a candidate path, reserve its footprint and
          the previous state's last footprint from the last time period."""
        for i, node in enumerate(path):
            pos = Pos(node.x, node.y, node.z)
            t = node.time
            footprint = self.compute_footprint(uav, pos)
            reservations = self.add_footprints_to_reservations(reservations, footprint, uav.id, t)
            if i > 0:
                prev_node = path[i-1]
                if node.time > prev_node.time:
                    prev_pos = Pos(prev_node.x, prev_node.y, prev_node.z)
                    prev_footprint = self.compute_footprint(uav, prev_pos)
                    reservations = self.add_footprints_to_reservations(reservations, prev_footprint, uav.id, t)
        # Add the last footprint for the last time step
        last_pos = Pos(path[-1].x, path[-1].y, path[-1].z)
        last_footprint = self.compute_footprint(uav, last_pos)
        last_time = path[-1].time + 1
        last_footprint = self.add_footprints_to_reservations(reservations, last_footprint, uav.id, last_time)
        return reservations
    

    def order_uavs(
            self,
            uav_list: List[UAV],
            delay_counts: Dict[int,int] = None
            ) -> List[UAV]:
        """
        Always primary‐sort by start_time (ascending), then by every other
        field in self.ordering (abs() gives position, sign gives direction).
        """
        key_funcs = {
            "id":         lambda u: u.id,
            "inaccuracy": lambda u: u.inaccuracy[0],
            "max_speed":  lambda u: u.max_speed,
            "delay":      lambda u: delay_counts.get(u.id, 0) if delay_counts else 0,
            "distance":   lambda u: np.linalg.norm(
                                  u.destinations[-1].to_array()
                                - u.current_position.to_array()
                               ),
        }
        other_fields = sorted(
            (f for f in self.ordering.keys() if f != "start_time"),
            key=lambda f: abs(self.ordering[f])
        )

        #build composite key
        def sort_key(u: UAV):
            #start time always primary key to simulate order of submission
            key = [u.start_time]
            # then each other field, reversed if int is negative but same position respected
            for f in other_fields:
                val = key_funcs[f](u)
                if self.ordering[f] < 0:
                    val = -val
                key.append(val)
            return tuple(key)
        uav_list.sort(key=sort_key)
        return uav_list

    

    def plan_path(self, environment: Environment) -> Dict[int, List[State]]:
        """Plan a path for each UAV in the environment.
        Returns a dict mapping UAV ids to their candidate paths."""

        candidate_paths: Dict[int, List[State]] = {}
        reservations: Dict[tuple, List[int]] = {}
        # env reservations act as a uav with an impossible id
        if len(environment.reservations) != 0:
            for state in environment.reservations:
                reservations[state] = [-1]
        uav_list = environment.uav_list
        #schedule times are the time at which each UAV is scheduled to start its route from the path planner
        schedule_times = {uav.id: uav.start_time for uav in uav_list}
        delay_counts: Dict[int, int] = {uav.id: 0 for uav in uav_list}


        uav_list = self.order_uavs(uav_list,delay_counts)
        #get all non start positions for each uav in one List[Pos]
        #starts and goals are both for heuristic use later
        goals: List[Pos] = [
            dest
            for uav in uav_list
            for dest in uav.destinations[1:]
        ]
        starts = [uav.destinations[0] for uav in uav_list]
        obstacles = self.create_obstacle_dict(environment)
        #Initialize searched node counts
        searched_counts: Dict[int, int] = {uav.id: 0 for uav in uav_list}
        latest_start_time = max(schedule_times[uav.id] for uav in uav_list)
        max_sim_time = latest_start_time + cfg.MAX_SIM_TIME
        for current_time in range(0, max_sim_time + 1):
            # collect UAVs whose scheduled time == current_time
            to_plan = [u for u in uav_list if schedule_times[u.id] == current_time]
            to_plan = self.order_uavs(to_plan,delay_counts)
            for uav in to_plan:
                # Check spawn-cell occupancy (ignore self)
                spawn_pos = Pos(uav.destinations[0].x, uav.destinations[0].y, uav.destinations[0].z)
                footprint = self.compute_footprint(uav, spawn_pos)
                occupied = False
                for voxel in footprint:
                    for i in range(0,1):
                        voxel_key = (voxel.x, voxel.y, voxel.z, current_time + i)
                        occupied_by = reservations[voxel_key] if voxel_key in reservations else []
                        if any(other_id != uav.id for other_id in occupied_by):
                            occupied = True
                            break
                if occupied:
                    schedule_times[uav.id] += 1
                    delay_counts[uav.id] += 1
                    continue
                #add uav starting positions to reservations if unoccupied
                reservations = self.add_footprints_to_reservations(reservations, footprint, uav.id, current_time)
                reservations = self.add_footprints_to_reservations(reservations, footprint, uav.id, current_time + 1)
            for uav in to_plan:
                # Check spawn-cell occupancy (ignore self)
                spawn_pos = Pos(uav.destinations[0].x, uav.destinations[0].y, uav.destinations[0].z)
                footprint = self.compute_footprint(uav, spawn_pos)
                occupied = False
                #second check to make sure there is value in spawning uav at this time
                #checks if an available move is possible from starting location
                free_neighbours = 0
                for voxel in footprint:
                    for dx in [-1, 0, 1]:
                        for dy in [-1, 0, 1]:
                            for dz in [-1, 0, 1]:
                                if dx == 0 and dy == 0 and dz == 0:
                                    continue
                                neighbour_key = State(voxel.x + dx, voxel.y + dy, voxel.z + dz, current_time)
                                if not (0 <= neighbour_key.x < environment.world_data.shape[0] and
                                        0 <= neighbour_key.y < environment.world_data.shape[1] and
                                        0 <= neighbour_key.z < environment.world_data.shape[2]):
                                    continue  
                                if neighbour_key in reservations:
                                    if ((uav.id not in reservations[neighbour_key] and len(reservations[neighbour_key]) > 0) or 
                                    (uav.id in reservations[neighbour_key] and len(reservations[neighbour_key]) > 1)):
                                        break
                                free_neighbours += 1
                if free_neighbours == 0:
                    schedule_times[uav.id] += 1
                    delay_counts[uav.id] += 1
                    continue
                for i in range(0,2):
                    voxel_key = (voxel.x, voxel.y, voxel.z, current_time + i)
                    occupied_by = reservations[voxel_key] if voxel_key in reservations else []
                    if any(other_id != uav.id for other_id in occupied_by):
                        occupied = True
                        break
                #delay to next time step
                if occupied == True or free_neighbours == 0:
                        schedule_times[uav.id] += 1
                        delay_counts[uav.id] += 1
                        continue

                # perform search to build path for the uav
                full_path: List[State] = []
                #segmented search for each goal
                for idx, dest in enumerate(uav.destinations):
                    if idx == 0:
                        full_path.append(State(dest.x, dest.y, dest.z, current_time))
                    else:
                        start_state = full_path[-1]
                        segment,segment_searched = self.a_star_search(
                            start_state, dest, environment.world_data,
                            max_sim_time, candidate_paths,
                            uav, obstacles, reservations,
                            goals, starts)
                        if segment is None:
                            # If no path is found, remove any reservations made for this UAV
                            footprint = self.compute_footprint(uav, spawn_pos)
                            for time in range(0,2):
                                for voxel in footprint:
                                    key = (voxel.x, voxel.y, voxel.z, current_time + time)
                                    if key in reservations and uav.id in reservations[key]:
                                        reservations[key].remove(uav.id)
                                    try:
                                        if reservations[key] == []:
                                            del reservations[key]
                                    except KeyError:
                                        pass
                            # delay the UAV and continue to the next one
                            schedule_times[uav.id] += 1
                            delay_counts[uav.id] += 1
                            full_path = None
                            break
                        # If a path is found, add the segment to the full path
                        full_path.extend(segment[1:])
                        searched_counts[uav.id] += segment_searched
                if not full_path:
                    continue
                # Remove oscillations from the path
                full_path = remove_same_timestep_oscillations(full_path)
                # Reserve all footprint voxels for this path
                reservations = self.add_reservation(reservations, uav, full_path)
                # Assign the computed path
                candidate_paths[uav.id] = full_path

        return candidate_paths, delay_counts,searched_counts

    def a_star_search(
        self,
        start: State,
        goal: Pos,
        grid: np.ndarray,
        max_time: int,
        candidate_paths: Dict[int,List[State]],
        uav: UAV,
        obstacles: Dict[tuple,bool],
        reservations: Dict[tuple,List[int]],
        goals: List[Pos],
        starts: List[Pos]
    ):
        """
        A* search from start (State) to goal (Pos) using TMState.
        Returns a list of State (dropping moves_used) representing the found path, or None if no valid path is found.
        """
        # TMState = (x, y, z, time, moves_used)
        beam_width = self.beam_width
        start_state = (start.x, start.y, start.z, start.time, uav.max_speed)
        open_set = [(0, start_state)]
        came_from = {}
        g_score = {start_state: 0.00}
        f_score = {start_state: 0.00} # for tracking the cost of the path over time
        searched = 0

        def euclidean_heuristic(state):
            x, y, z, t, used = state
            return np.linalg.norm(np.array((x, y, z)) - np.array(goal.to_array()))
        
        def manhattan_with_conflicts(state, goal: Pos, reservations, conflict_weight=10.0):
            """Inadmissable heuristic that adds a penalty for each conflict between neighbour and goal."""
            x, y, z, t, _ = state
            gx, gy, gz = goal.x, goal.y, goal.z
            # Basic Manhattan distance
            man_dist = abs(x - gx) + abs(y - gy) + abs(z - gz)
            # Conflict penalty: sample along straight path
            penalty = 0.0
            steps = int(man_dist)
            dx, dy, dz = np.sign(gx - x), np.sign(gy - y), np.sign(gz - z)
            cx, cy, cz = x, y, z
            for step in range(1, steps + 1):
                if abs(cx - gx) > 0: cx += dx
                elif abs(cy - gy) > 0: cy += dy
                else: cz += dz
                key = (int(cx), int(cy), int(cz), t + step)
                if key in reservations:
                    penalty += conflict_weight * len(reservations[key])
            return man_dist + penalty
        
        def manhattan(state, goal: Pos):
            x, y, z, t, _ = state
            gx, gy, gz = goal.x, goal.y, goal.z
            # Basic Manhattan distance
            man_dist = abs(x - gx) + abs(y - gy) + abs(z - gz)
            
            return man_dist
        def reverse_manhattan(state, goal: Pos):
            x, y, z, t, _ = state
            gx, gy, gz = goal.x, goal.y, goal.z
            # Basic Manhattan distance
            man_dist = abs(x - gx) + abs(y - gy) + abs(z - gz)
            
            return -man_dist
        
        def oscillation_penalty(state, came_from, weight=5.0):
            """
            Penalize moves that reverse the last displacement, 
            encouraging the planner to hover instead of zig-zag.
            """
            # state and its parent
            x, y, z, t, _ = state
            parent = came_from.get(state)
            if parent is None or came_from.get(parent) is None:
                return 0.0
            # grandparent to compute last displacement
            grand = came_from[parent]
            # vectors
            dx1, dy1, dz1 = parent[0]-grand[0], parent[1]-grand[1], parent[2]-grand[2]
            dx2, dy2, dz2 = x-parent[0], y-parent[1], z-parent[2]
            # dot product
            dot = dx1*dx2 + dy1*dy2 + dz1*dz2
            if dot < 0:  
                # moving opposite the previous direction
                return weight
            return 0.0


        
        def traffic_density_penalty(state, reservations, neighborhood=1, time_horizon=3, weight=1.0):
            """
            Penalize high-traffic regions by counting reserved cells in spatiotemporal neighborhood.
            """
            x, y, z, t, _ = state
            count = 0
            for (vx, vy, vz, tv), uavs in reservations.items():
                if abs(vx - x) <= neighborhood and abs(vy - y) <= neighborhood and abs(vz - z) <= neighborhood:
                    if t <= tv <= t + time_horizon:
                        count += len(uavs)
            return weight * count
        
        def avoid_indirect_collisions_heuristic(state, reservations, obstacles, uav) -> int:
            x, y, z, t, used = state
            # Note: compute_footprint expects a position tuple.
            nodes = self.compute_footprint(uav, Pos(x, y, z))
            uav_collisions = 0
            world_collisions = 0
            for node in nodes:
                for i in range(0,1):
                    key = (node.x, node.y, node.z, t + i)
                    if key in reservations:
                        if uav.id in reservations[key]:
                            uav_collisions += len(reservations[key]) - 1
                        else:
                            uav_collisions += len(reservations[key])
                    if (node.x, node.y, node.z) in obstacles:
                        world_collisions += 1
                    if used == uav.max_speed - 1:
                        key_next = (node.x, node.y, node.z, t +  1)
                        if key_next in reservations:
                            if uav.id in reservations[key_next]:
                                uav_collisions += len(reservations[key_next]) - 1
                            else:
                                uav_collisions += len(reservations[key_next])
            if self.enable_indirect_world_collisions is True:
                return (uav_collisions * 10000) + (world_collisions * 50)
            else:
                return (uav_collisions * 10000) + (world_collisions * 10000)
        
        def avoid_indirect_collisions_bool(state, reservations, obstacles, uav) -> bool:
            x, y, z, t, used = state
            # Note: compute_footprint expects a position tuple.
            nodes = self.compute_footprint(uav, Pos(x, y, z))
            for node in nodes:
                for i in range(0,1):
                    key = (node.x, node.y, node.z, t + i)
                    if key in reservations:
                        if uav.id in reservations[key]:
                            if (len(reservations[key]) - 1 > 0 ):
                                return True
                        else: return True
                    if self.enable_indirect_world_collisions is False:
                        if (node.x, node.y, node.z) in obstacles:
                            return True
                    if used == uav.max_speed - 1:
                        key_next = (node.x, node.y, node.z, t +  1)
                        if key_next in reservations:
                            if uav.id in reservations[key_next]:
                                if (len(reservations[key]) - 1 > 0 ):
                                    return True
                            else:
                                return True
            return False
        
        def avoid_direct_collisions(state, reservations, uav):
            if state in reservations:
                if uav.id in reservations[state]:
                    return (len(reservations[state]) - 1) * 10000
                else:
                    return len(reservations[state]) * 10000
            else:
                if obstacles.get((state[0], state[1], state[2]), False):
                    return 1
            return 0
        
        def move_from_goals(state, goals):
            x, y, z, t, used = state
            distance = 0
            for goal in goals:
                distance += abs(x - goal.x) + abs(y - goal.y) + abs(z - goal.z)
            return distance
        
        def move_from_starts(state, starts):
            x, y, z, t, used = state
            distance = 0
            for start in starts:
                distance -= abs(x - start.x) + abs(y - start.y) + abs(z - start.z)
            return -distance
        
        def same_y(state,goal):
            x, y, z, t, used = state
            return abs(y - goal.y) * 2
        
        def incentivise_waiting(state,came_from):
            x, y, z, t, used = state
            px, py, pz, pt, unused = came_from
            if (x == px and y == py and z ==pz):
                return 1
            return 0

        def higher_inaccuracy_penalty(state, uav):
            """
            Incentivise inaccurate UAVs to maintain a higher altitude
            """
            x, y, z, t, used = state
            return - y * uav.inaccuracy[0]
        
        def move_in_straight_line(state,came_from,goal,start):
            sx, sy, sz, st = start
            x, y, z, t, used = state
            px, py, pz, pt, pu = came_from
            gx, gy, gz = goal.x, goal.y, goal.z
            if gx - sx > gz - sz:
                if x == gx:
                    return 0
                if x - px == 0:
                    return 0.5
            elif gz - sz > gx - sx:
                if z == gz:
                    return 0
                if z - pz == 0:
                    return 0.5
            else:
                return 0
            return 0
        
        def minimum_time_lower_bound(state, goal, max_speed):
            """
            returns lower bound on time-to-go: distance divided by max_speed
            """
            x, y, z, t, _ = state
            dist = np.linalg.norm(np.array((x, y, z)) - np.array(goal.to_array()))
            return dist / max_speed

        while open_set:
            
            if len(open_set) > beam_width:
                open_set = heapq.nsmallest(beam_width, open_set, key=lambda x: x[0])
                heapq.heapify(open_set)
            f, current = heapq.heappop(open_set)

            x, y, z, t, used = current
            if (x, y, z) == (goal.x, goal.y, goal.z):
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                
                path.reverse()
                total_f = sum(f_score[s] for s in path)
                #print(f"Completed path for UAV {uav.id}: total f-score = {total_f:.2f}")
                return ([State(s[0], s[1], s[2], s[3]) for s in path],searched)
            if t >= max_time:
                continue
            neighbors = []
            if uav.max_speed == 1:
                for dx, dy, dz in [(-1,0,0), (1,0,0), (0,-1,0), (0,1,0), (0,0,-1), (0,0,1)]:
                    nx, ny, nz = x + dx, y + dy, z + dz
                    if 0 <= nx < grid.shape[0] and 0 <= ny < grid.shape[1] and 0 <= nz < grid.shape[2]:
                        if grid[nx, ny, nz] == 0:
                            key = (nx, ny, nz, t)
                            if key not in reservations or uav.id not in reservations[key]:
                                neighbors.append((nx, ny, nz, t + 1, 0))

                neighbors.append((x, y, z, t + 1, 0))
            else:
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        for dz in [-1, 0, 1]:
                            if dx == 0 and dy == 0 and dz == 0:
                                continue
                            if abs(dx) + abs(dy) + abs(dz) != 1:
                                continue
                            nx, ny, nz = x + dx, y + dy, z + dz
                            if 0 <= nx < grid.shape[0] and 0 <= ny < grid.shape[1] and 0 <= nz < grid.shape[2]:
                                if grid[nx, ny, nz] == 0:
                                    if used < uav.max_speed - 1:
                                        neighbor = (nx, ny, nz, t, used + 1)
                                    else:
                                        neighbor = (nx, ny, nz, t + 1, 0)
                                    neighbors.append(neighbor)
                neighbors.append((x, y, z, t + 1, 0))
            if self.disable_collisions is False:
                filtered_neighbors = neighbors
            else:
                filtered_neighbors = []
                for nbr in neighbors:
                    # first, rule out any UAV–UAV conflict at this time step:
                    
                    if avoid_indirect_collisions_bool(nbr, reservations, obstacles, uav) is True:
                        # would overlap footprints in t or t+1
                        continue
                    # otherwise it’s safe w.r.t. UAV–UAV collisions
                    filtered_neighbors.append(nbr)
            
            for neighbor in filtered_neighbors:
                tentative_g = (g_score[current] + self.g_score_increment) * self.g_score_multiplier
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    h = 0.0
                    if self.heuristics.get("euclidean") is True:
                        h += euclidean_heuristic(neighbor)
                    if self.heuristics.get("move_from_goals") is True:
                        h += move_from_goals(neighbor, goals)
                    if self.heuristics.get("move_from_starts") is True:
                        h += move_from_starts(neighbor, starts)
                    if self.heuristics.get("avoid_indirect_collisions") is True:
                        collision_score = avoid_indirect_collisions_heuristic(neighbor, reservations, obstacles, uav)
                        if collision_score > 0 and self.disable_collisions is True:
                            break
                        h += collision_score
                    if self.heuristics.get("avoid_direct_collisions") is True:
                        collision_score = avoid_direct_collisions(neighbor, reservations, uav)
                        if collision_score > 0 and self.disable_collisions is True:
                            break
                        h += collision_score
                    if self.heuristics.get("move_in_straight_line") is True:
                        h += move_in_straight_line(neighbor,came_from[neighbor],goal,start)
                    if self.heuristics.get("same_y") is True:
                        h += same_y(neighbor,goal)
                    if self.heuristics.get("higher_inaccuracy_penalty") is True:
                        h += higher_inaccuracy_penalty(neighbor, uav)
                    if self.heuristics.get("traffic_density_penalty") is True:
                        h += traffic_density_penalty(neighbor, reservations)
                    if self.heuristics.get("manhattan") is True:
                        h += manhattan(neighbor, goal)
                    if self.heuristics.get("reverse_manhattan") is True:
                        h += reverse_manhattan(neighbor, goal)
                    if self.heuristics.get("manhattan_conflicts") is True:
                        h += manhattan_with_conflicts(neighbor, goal, reservations)
                    if self.heuristics.get("oscillation_penalty") is True:
                        h += oscillation_penalty(neighbor, came_from)
                    if self.heuristics.get("incentivise_waiting"):
                        h += incentivise_waiting(neighbor,came_from[neighbor])
                    if self.heuristics.get("minimum_time_lower_bound") is True:
                        h += minimum_time_lower_bound(neighbor, goal, uav.max_speed)
                    f = tentative_g + h
                    f_score[neighbor] = f 
                    heapq.heappush(open_set, (f, neighbor))
                searched += 1
        return None,searched