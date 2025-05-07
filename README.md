# individualproj
UTM simulator

# To run
run "pip install -r requirements.txt"
followed by "py main.py <scenario_config_name.json> --output_mode <int>"
0 is good for the display, otherwise use 3. 1 & 2 give exact movement.
Use --help as an argument for more info.

# Scenarios
Look at "scenarios.json" for the format a scenario file should be in.

# Config Settings
In utils/config.py there are a variety of configuration settings for default values.

# Designing maps
Maps can be created in the maps.py file as numpy arrays (see examples in there)
When importing into a scenario they can be scaled up and have the map height overridden (by repetitions).
If this value is anything but 0, the bottom layer be repeated in the place of all layers for that many repetitions.

# Ordering
Ordering – The outcome of cooperative pathfinding is heavily affected by the priority of ordering agents. Therefore, an input for the planner of an ordering dictionary was added with the following values by default: {"id": 0,"delay": 1,"inaccuracy": 2,"max_speed": 3,"start_time": 4,"distance": 5}
The priority of UAV ordering is sorted in ascending order of the keys in the dictionary, with lower numbers applied first (i.e. 0 highest priority), and if you assign a negative value (e.g. "distance": -5), that field is sorted in descending rather than ascending order.

Planners in situations can take an ordering. Include a "ordering": {"id": 0,"delay": 1, "inaccuracy": 2, "max_speed": 3, "start_time": 4, "distance": 5}

# Heuristics
Planners can take a "heuristic":{"manhattan_scaled":false,"same_y":true}

# Other planner inputs
"beam_width": int,
"g_score_multiplier": float,
"g_score_increment": float,
"disable_collisions": bool,
"enable_indirect_world_collisions": bool

