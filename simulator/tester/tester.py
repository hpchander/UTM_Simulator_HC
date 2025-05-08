import tracemalloc
import matplotlib.pyplot as plt
import time
import csv
from simulator import Environment
from simulator.utils.shared_imports import np, PrettyTable, get_ansi_colour, ansi_colours

def run_tests(environment: Environment,
              output_mode: int = 0,
              uav_mode: str = "wait-on-pass",
              planners: dict = {},
              all_candidate_paths: dict = {},scenario_name: str = "Scenario"):
    """
    Run the tests for the given environment and planners.
    If output_mode == 5, append results.csv with headers+rows for this scenario.
    """
    # storage
    delay_dict = {}
    searched_dict = {}
    searched_totals = {}
    time_dict = {}
    memory_dict = {}
    results = {}

    # prepare the final table
    table = PrettyTable()
    table.field_names = [
        "Scenario", "Planner", "Timesteps", "Movements", "Total Delayed",
        "Collisions", "Success", "Score", "Total Searched",
        "Run Time (s)", "Peak Mem (MB)"
    ]

    # prepare measurement dicts
    for name, planner in planners.items():
        environment.reset_environment()
        tracemalloc.start()
        start_time = time.time()

        candidate, delay, searched = planner.plan_path(environment)

        end_time = time.time()
        current, peak = tracemalloc.get_traced_memory()
        tracemalloc.stop()

        all_candidate_paths[name] = candidate
        delay_dict[name]      = delay
        searched_dict[name]   = searched
        time_dict[name]       = end_time - start_time
        memory_dict[name]     = peak / 1024 / 1024

    # assign the candidate paths to the environment
    for name, candidate in all_candidate_paths.items():
        environment.candidate_paths[name] = candidate
        environment.set_output_mode(output_mode)
        searched_totals[name] = 0
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
        # run sim
        environment.set_active_candidate_path(name)
        results[name] = environment.run()
        environment.reset_environment()
        environment.set_active_candidate_path(name)

    if output_mode in [0,2]:
        environment.display()
        environment.reset_environment()

    # collate results
    lowest = np.inf
    lowest_names = []
    rows = []

    for name, sim_res in results.items():
        # track lowest timesteps
        ts = sim_res["timesteps"]
        if ts < lowest:
            lowest = ts
            lowest_names = [name]
        elif ts == lowest:
            lowest_names.append(name)

        score = candidate_evaluator(name, sim_res, searched_totals=searched_totals)

        row = [
            scenario_name,
            name,
            ts,
            sim_res["movements"],
            sim_res["waited"],
            sim_res["collisions"],
            sim_res["success"],
            score,
            searched_totals[name],
            round(time_dict[name], 4),
            round(memory_dict[name], 4)
        ]
        table.add_row(row)
        rows.append(row)

    #append to csv
    if output_mode == 5:
        with open('./results/results.csv', 'a', newline='') as f:
            writer = csv.writer(f)
            # blank line for readability
            writer.writerow([])
            # header
            writer.writerow(table.field_names)
            # data rows
            writer.writerows(rows)
        print("Appended results to results.csv")
    
    elif output_mode == 6:


        # extract per‐planner lists
        planners_list = [r[1] for r in rows]
        timesteps     = [r[2] for r in rows]
        runtimes      = [r[9] for r in rows]
        memories      = [r[10] for r in rows]
        scores        = [r[7] for r in rows]

        # 1) Timesteps
        plt.figure()
        plt.bar(planners_list, timesteps)
        plt.title(f"{scenario_name} – Timesteps by Planner")
        plt.xlabel("Planner")
        plt.ylabel("Timesteps")
        plt.show()

        # 2) Run Time
        plt.figure()
        plt.bar(planners_list, runtimes)
        plt.title(f"{scenario_name} – Run Time (s) by Planner")
        plt.xlabel("Planner")
        plt.ylabel("Run Time (s)")
        plt.show()

        # 3) Peak Memory
        plt.figure()
        plt.bar(planners_list, memories)
        plt.title(f"{scenario_name} – Peak Memory (MB) by Planner")
        plt.xlabel("Planner")
        plt.ylabel("Peak Mem (MB)")
        plt.show()

        # 4) Score
        plt.figure()
        plt.bar(planners_list, scores)
        plt.title(f"{scenario_name} – Score by Planner")
        plt.xlabel("Planner")
        plt.ylabel("Score")
        plt.show()

    # 4b) Otherwise print to console
    else:
        print(table)
        print(f"Lowest timesteps: {lowest} by {lowest_names}")

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



