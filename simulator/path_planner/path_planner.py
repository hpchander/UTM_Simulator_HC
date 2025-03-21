from simulator.utils.shared_imports import np, Math, abstractmethod, ABC
from simulator.uav.uav import UAV
from simulator.environment.environment import Environment
from simulator.asp.asp import ASP
from scipy.ndimage import binary_dilation
import heapq

class PathPlanner(ABC):
    """
    Abstract base class for planning paths.
    """
    @abstractmethod
    def plan_path(self, environment) -> dict:
        """
        Creates routes for UAVs to reach all their destinations whilst avoiding collisions.
        Returns a dictionary mapping UAV IDs to a list of waypoints (candidate path).
        """
        pass



class AStarPlanner(PathPlanner):
    """
    A* path planner.
    """
    def __init__(self, heuristics: dict):
        self.heuristics = heuristics


    def plan_path(self, environment) -> dict:
        """
        For each UAV in the environment, compute a candidate path from its current position
        to its final destination using A* on a time-expanded grid.
        Returns a dictionary mapping UAV IDs to their candidate path (a list of waypoints).
        """
        candidate_paths = {}
        latest_start_time = max([uav.start_time for uav in environment.uav_list])
        current_time = 0
        planned_uavs = []
        while current_time <= latest_start_time:
            current_time += 1
            #skip uavs that have not started yet
            for uav in environment.uav_list:
                if uav.id in planned_uavs:
                    continue
                if uav.start_time == current_time:
                    planned_uavs.append(uav.id)
                
                
                path = []
                steps = uav.destinations.__len__()
                world = environment.world_data

                
                for i in range(steps):
                    start = uav.destinations[i-1] if i > 0 else uav.destinations[0]
                    start = (start[0],start[1],start[2],uav.start_time)
                    goal = uav.destinations[i]
                    full_path = self.a_star_search(
                        start,
                        goal,
                        world,
                        max_time=100,
                        candidate_paths=candidate_paths,
                        uav_list=environment.uav_list
                        )
                    if i == 0:
                        path += full_path
                    else:
                        if full_path is not None:
                            path += full_path[1:]
                        else:
                            path = None
                            break
                if path is None:
                    print(f"Path planning failed for UAV {uav.id} from {start} to {goal}")
                else:
                    candidate_paths[uav.id] = path
            return candidate_paths

    def a_star_search(self, start: tuple, goal: tuple, grid: np.ndarray, max_time: int, candidate_paths,uav_list) -> list:
        """
        Perform A* search on a time-expanded grid.
        Each node is represented as (x, y, z, t) where t is the timestep.
        Returns a list of nodes representing the path or None if no path is found.
        """
        start_node = start
        #start_node = (start[0], start[1], start[2], 0)
        open_set = [(0, start_node)]
        came_from = {}
        g_score = {start_node: 0}
        
        def euclidean_heuristic(node):
            """
            Euclidean distance heuristic.
            """
            x, y, z, t = node
                
            return np.linalg.norm(np.array((x, y, z)) - np.array(goal))
        
        def avoid_collision(node, candidate_paths):
            """
            Check if the node is occupied by another UAV at the same time.
            """
            x, y, z, t = node
            for uav in uav_list:
                if uav.id in candidate_paths:
                    path = candidate_paths[uav.id]
                    relative_t = t - uav.start_time
                    if relative_t >= 0 and len(path) > relative_t and path[relative_t] == (x, y, z, t):
                        #print(f"Collision detected at {node} for UAV {uav.id}")
                        return 50
            return 0
        
        def incentivise_waiting(node):
            """
            Incentivise waiting in place.
            """
            x, y, z, t = node
            if t > 0 and (x, y, z, t-1) not in came_from:
                return 1
            return 0
        
        
        
        
        while open_set:
            current_f, current = heapq.heappop(open_set)
            x, y, z, t = current
            
            if (x, y, z) == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return list(reversed(path))
            if t >= max_time:
                continue
            # Neighbor moves (and waiting in place)
            neighbors = []
            for dx, dy, dz in [(1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1),(0,0,0)]:
                nx, ny, nz = x+dx, y+dy, z+dz
                nt = t+1
                neighbor = (nx, ny, nz, nt)

                if 0 <= nx < grid.shape[0] and 0 <= ny < grid.shape[1] and 0 <= nz < grid.shape[2]:
                    # Check if the cell is free (0 indicates free)
                    if grid[nx, ny, nz] == 0:
                        neighbors.append((nx, ny, nz, nt))
            for neighbor in neighbors:
                tentative_g = g_score[current] + 1
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g
                    if self.heuristics.get("euclidean") is True:
                        f +=  euclidean_heuristic(neighbor)
                    if self.heuristics.get("avoid") is True:
                        f +=  avoid_collision(neighbor, candidate_paths)
                    if self.heuristics.get("waiting") is True:
                        f +=  incentivise_waiting(neighbor)
                    heapq.heappush(open_set, (f, neighbor))
        return None
