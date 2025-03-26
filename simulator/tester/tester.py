from simulator import Environment
from simulator.utils.shared_imports import np,PrettyTable
from simulator.path_planner.path_planner import AStarPlanner

def run_tests(environment: Environment, output_mode: int = 1, uav_mode: str = "wait-on-pass",planners = {},all_candidate_paths = {}):
    results = {}
    table = PrettyTable()
    table.field_names = ["Planner", "Timesteps", "Movements","Waited","Collisions","Success"]
    for name, planner in planners.items():
        candidate = planner.plan_path(environment)
        all_candidate_paths[name] = candidate
    for name, candidate in all_candidate_paths.items():
        environment.candidate_paths[name] = candidate
        environment.set_uav_mode(uav_mode)
        environment.set_output_mode(output_mode)
        if output_mode in [1,2]: print(f"{name} candidate path: {candidate}")
        environment.set_active_candidate_path(name)
        if output_mode in [1,2]: print(f"Running {name} path")
        results[name] =  environment.run()
        environment.reset_environment()
        environment.set_active_candidate_path(name)
        if output_mode in [2,3]:
            environment.display()
            environment.reset_environment()
    lowest_names = []
    lowest = np.inf
    for name, results in results.items():
        if results["timesteps"] < lowest:
            lowest_names = [name]
            lowest = results["timesteps"]
        elif results["timesteps"] == lowest:
            lowest_names.append(name)
        table.add_row([name, results["timesteps"], results["movements"],results["waited"],results["collisions"],results["success"]])
    print(f"Results for {uav_mode}")
    print(table)
    print(f"Lowest timesteps: {lowest} for {lowest_names}")