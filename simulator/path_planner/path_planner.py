from simulator.utils.shared_imports import np, Math, abstractmethod, ABC
from simulator.uav.uav import UAV
from simulator.environment.environment import Environment
from simulator.asp.asp import ASP

class PathPlanner(ABC):
    @abstractmethod
    def plan_path(self,environment) -> list:
        """
        Creates routes for UAVs to reach all their destinations whilst avoiding collisions.
        """
        environment.leased_airspace
        pass