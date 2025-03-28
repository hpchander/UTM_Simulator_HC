# main.py
import argparse
import json
from simulator import Map, Environment, UAV
from simulator.maps import maps
from simulator.utils.shared_imports import Pos,State
from simulator.path_planner.path_planner import AStarPlanner, ObliviousPlanner
from simulator.scenario.scenario import Scenario
from simulator.tester.tester import run_tests
import simulator.utils.config as cfg

def load_config(path):
    """
    Load the scenario configuration from the JSON file provided by arg 1.
    """

    if path.lower().endswith('.json'):
        with open(path) as f:
            return json.load(f)
    else:
        raise ValueError("Unsupported scenario format; use .json")

def build_planners(planner_defs):
    """
    Build planners from the definitions provided in scenario config.
    """
    planners = {}
    for name, config in planner_defs.items():
        ptype = config.get('type', 'AStarPlanner')
        if ptype == 'AStarPlanner':
            planners[name] = AStarPlanner(
                heuristics   = config.get('heuristics', cfg.DEFAULT_HEURISTICS),
                beam_width   = config.get('beam_width', cfg.DEFAULT_BEAM_WIDTH),
                ordering     = config.get('ordering', cfg.DEFAULT_ORDERING),
                **{k: v for k, v in config.items()
                if k not in ('heuristics','beam_width','ordering','type')}
            )
        elif ptype == 'Oblivious':
            planners[name] = ObliviousPlanner()
        else:
            raise ValueError(f"Unsupported planner type: {config['type']}")
    return planners

def build_uav(uav_def):
    """
    Build a UAV from the definition provided in scenario config.
    """
    if 'name' not in uav_def:
        raise ValueError("UAV definition must include 'name' key.")
    if 'destinations' not in uav_def:
        raise ValueError("UAV definition must include 'destinations' key.")
    if len(uav_def['destinations']) < 2:
        raise ValueError("UAV definition must include at least 2 destinations.")
    dests = [Pos(*coords) for coords in uav_def['destinations']]
    return UAV(
        uav_type   = uav_def.get('uav_type', 0),
        destinations=dests,
        inaccuracy = uav_def.get('inaccuracy', [0,0]),
        start_time = uav_def.get('start_time', 0),
        max_speed  = uav_def.get('max_speed', 1),
        name       = uav_def.get('name', '')
    )

def build_scenario(sdef, planners, output_mode):
    """
    Build a scenario from the json definition."""
    map_name = sdef['map']['name']
    map_array = getattr(maps, map_name)
    S_map = Map(
        sdef['map'].get('name', map_name),
        map_array,
        scale       = sdef['map'].get('scale', 1),
        repetitions = sdef['map'].get('repetitions', 0)
    )

    # build uav list
    uavs = [build_uav(u) for u in sdef['uavs']]

    # adds reservations to map
    res_temp = sdef.get('reservations', [])
    reservations = []
    for res in res_temp:
        reservations.append((res[0], res[1], res[2], res[3]))
    
    if output_mode == -1:
        output_mode = sdef.get('output_mode', 0)
    #creates scenario obj
    scen = Scenario(
        sdef.get('name', 'Unnamed'),
        map=S_map,
        uav_list=uavs,
        output_mode=output_mode,
        reservations=reservations
    )
    scen.assign_planners(planners)
    return scen

def main():
    #parse command line arguments
    p = argparse.ArgumentParser()
    p.add_argument('scenario', help="Path to JSON (or TXT) scenario+planner config")
    p.add_argument('--output_mode', type=int, default=-1,
                help= "0: Only results. " \
                "1: Full tex no display.    2: Full text and display.   " \
                "3: Display+Results -1: use scenario config (default)")
    args = p.parse_args()

    config = load_config(args.scenario)

    #build planners
    planners = build_planners(config['planners'])
    #print(planners.keys())
    #build + run scenarios
    for sdef in config['scenarios']:
        scen = build_scenario(sdef, planners, args.output_mode)
        print(f"\n=== Running {scen.name} ===")
        scen.run(planners=planners)

if __name__ == '__main__':
    main()
