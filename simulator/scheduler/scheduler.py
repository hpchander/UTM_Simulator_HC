from simulator.utils.shared_imports import np, Math, abstractmethod, ABC
from simulator.uav.uav import UAV
from simulator.environment.environment import Environment
from simulator.asp.asp import ASP

class Scheduler(ABC):
    @abstractmethod
    def schedule(self, uav_list, environment, current_time: int) -> dict:
        """
        Given a list of UAVs and the environment (including leased airspace and conflict information),
        decide the next move for each UAV. The returned dictionary maps UAV IDs (or UAV objects) to a
        planned move (which could be a waypoint or a command such as "wait" or "move").
        """
        pass