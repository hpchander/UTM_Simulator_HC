from simulator.environment.environment import Environment
from simulator.map.map import Map
from simulator.uav.uav import UAV
from simulator.utils.shared_imports import np
from simulator.path_planner.path_planner import AStarPlanner
from simulator.tester.tester import run_tests

class Scenario():
    """
    Class to represent a scenario in the simulation.
    It contains the map, the UAVs, and the environment.

    """
    def __init__(self,name, map, uav_list, output_mode=0,reservations=[],planners = {}):
        self.name = name
        self.map = map
        self.uav_list = uav_list
        self.output_mode = output_mode
        self.reservations = reservations
        self.env = Environment(map.world_data, output_mode)
        self.env.set_reservations(reservations)
        for uav in uav_list:
            self.env.register_uav(uav)
        self.all_candidate_paths = {}
        self.planners = {}

    def assign_planners(self,planners:dict):
        """
        Assign planners to the scenario.
        """
        self.planners = planners

    def run(self,planners:dict = {}):
        """
        Run the scenario with the given planners.
        """
        run_tests(self.env, self.output_mode, planners=self.planners, all_candidate_paths=self.all_candidate_paths)

    
