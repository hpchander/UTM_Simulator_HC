from simulator import Environment
from simulator.utils.shared_imports import np,PrettyTable,get_ansi_colour,ansi_colours
from simulator.path_planner.path_planner import AStarPlanner

def run_tests(environment: Environment, output_mode: int = 0, uav_mode: str = "wait-on-pass",planners = {},all_candidate_paths = {}):
    """
    Run the tests for the given environment and planners.
    The results are printed in a table format.
    """
    results = {}
    table = PrettyTable()
    table.field_names = ["Planner", "Timesteps", "Movements","Total Delayed","Collisions","Success","Score","Total Searched"]
    delay_dict = {}
    searched_dict = {}
    searched_totals = {}
    #Generate plans
    for name, planner in planners.items():
        environment.reset_environment()
        candidate,delay,searched = planner.plan_path(environment)
        all_candidate_paths[name] = candidate
        delay_dict[name] = delay
        searched_dict[name] = searched
    #Assign plans to environment
    for name, candidate in all_candidate_paths.items():
        environment.candidate_paths[name] = candidate
        environment.set_output_mode(output_mode)
        searched_totals[name] = 0
        if output_mode in [0,1, 2, 3]:
            #Build table
            path_table = PrettyTable()
            path_table.field_names = ["UAV ID", "UAV Name", "Path by Time Step","Times Delayed","Nodes Searched"]
            for uav in environment.uav_list:
                searched_totals[name] += searched_dict[name][uav.id]
                color_code = get_ansi_colour(uav.id)
                reset      = ansi_colours["reset"]
                try:
                    path = environment.candidate_paths[name][uav.id]
                    # build a dict time → list of positions
                    path_by_time = {}
                    for state in path:
                        path_by_time.setdefault(state.time, []).append(f"[{state.x},{state.y},{state.z}]")
                    if uav.max_speed <= 4:
                        # chunk into runs of 2 timesteps
                        times = sorted(path_by_time.keys())
                        chunks = [times[i:i+2] for i in range(0, len(times), 2)]
                    else:
                        # chunk into runs of 3 timesteps
                        times = sorted(path_by_time.keys())
                        chunks = [times[i:i+1] for i in range(0, len(times), 1)]
                    # build one line per chunk
                    lines = []
                    for chunk in chunks:
                        segs = []
                        for t in chunk:
                            segs.append(f"T{t}: " + " ".join(path_by_time[t]))
                        lines.append(" | ".join(segs))

                    # join with newline
                    path_repr = f"{reset}\n{color_code}".join(lines)
                except Exception:
                    path_repr = "No Path Found"
                path_table.add_row([
                    f"{color_code}{uav.id}{reset}",
                    f"{color_code}{uav.name}{reset}",
                    f"{color_code}{path_repr}{reset}",
                    f"{color_code}{delay_dict[name][uav.id]}{reset}",
                    f"{color_code}{searched_dict[name][uav.id]}{reset}"
                ])
            if output_mode in [0,1,2]:
                print(f"\nCandidate Path: {name}")
                print(path_table)
                print("\n")
        # Run the simulation
        environment.set_active_candidate_path(name)
        if output_mode in [1,2]:
            print(f"Running {name} path")
        results[name] =  environment.run()
        environment.reset_environment()
        environment.set_active_candidate_path(name)
    if output_mode in [0,2]:
            environment.display()
            environment.reset_environment()
    # Keep track of the lowest timesteps
    # and the corresponding planner names
    lowest_names = []
    lowest = np.inf
    for name, results in results.items():
        if results["timesteps"] < lowest:
            lowest_names = [name]
            lowest = results["timesteps"]
        elif results["timesteps"] == lowest:
            lowest_names.append(name)
        score = candidate_evaluator(name, results,searched_totals=searched_totals)
        table.add_row([name, results["timesteps"], results["movements"],results["waited"],results["collisions"],results["success"],score, searched_totals[name]])
    #Print the results
    print(f"Results for {uav_mode}")
    print(table)
    print(f"Lowest timesteps: {lowest} for {lowest_names}")

def candidate_evaluator(name: str, results: dict,searched_totals: dict = {}):
    """
    Evaluates the candidate path based on the results of the simulation.
    The score is calculated based on the following criteria:
    - success: 10000 points if the path fails
    - timesteps: 10 points per timestep
    - collisions: 10 points for each collision with a UAV, 1000 points for each collision with an obstacle
    - movements: 1 point for each movement
    - waited: 1 point for each time step waited
    - searched: 0.001 points for each node searched
    Lower scores are better.
    """
    score = 0
    if results["success"] == False:
        score += 10000
    score += results["timesteps"] * 10
    collisions = results["collisions"]

    score += collisions[0] * 10
    score += collisions[1] * 1000
    score += results["movements"] * 1
    score += results["waited"] * 1
    score += searched_totals[name] * 0.001
    return round(score,3)



