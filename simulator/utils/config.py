# Configuration file for the simulator
# This file contains all the constants and default values used in the simulator
# It is recommended to keep the default values in this file and use them in the simulator
# This file is not meant to be modified by the user, but it can be used as a reference for the default values

MAX_SEARCH_DEPTH = 1000
DEFAULT_BEAM_WIDTH = 1000
ENABLE_INDIRECT_WORLD_COLLISIONS = False
DEFAULT_ORDERING = {"id": 0,"delay": 1, "inaccuracy": 2, "max_speed": 3, "start_time": 4, "distance": 5}
MAX_SIM_TIME = 1000
DEFAULT_MAX_SPEED = 1
DEFAULT_G_SCORE_MULTIPLIER = 1.0
DEFAULT_G_SCORE_INCREMENT = 1.0
DEFAULT_UAV_INACCURACY = [0, 0]
ENABLE_PARTIAL_COLLISION_DISABLER = True
MAX_DISPLAYED_NODES = 125000
DEFAULT_HEURISTICS = {
        "euclidean": False,
        "avoid_indirect_collisions": False,
        "move_in_straight_line": False,
        "same_y": False,
        "higher_inaccuracy_penalty": False,
        "move_from_goals": False,
        "move_from_starts": False,
        "combined_admissable": False,
        "manhattan_conflicts": False,
        "traffic_density_penalty": False,
        "oscillation_penalty": False,
        "incentivise_waiting": False
}
