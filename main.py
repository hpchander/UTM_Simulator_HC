from simulator import ASP, Map, Environment,UAV
from simulator.utils.shared_imports import np
from simulator.maps import maps
from simulator.path_planner.path_planner import AStarPlanner
from prettytable import PrettyTable

def run_tests(environment: Environment, output_mode: int = 1, uav_mode: str = "wait-on-pass"):
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
    

    
    print("Tests complete")

if __name__ == "__main__":
    #load map
    testMap = Map("Test Map", maps.map3,scale=0,repetitions=0)

    

    planners = {
        "AStar": AStarPlanner(heuristics={
        }),

        "AStar: euclidean": AStarPlanner(heuristics={
            "euclidean": True
        }),
        "AStar: avoid paths": AStarPlanner(heuristics={
            "avoid": True
        })
        # ,
        # "AStar: euclidean + avoid paths": AStarPlanner(heuristics={
        #     "avoid": True,
        #     "euclidean": True
        # })
        # ,
        # "AStar: euclidean + avoid paths + waiting": AStarPlanner(heuristics={
        #     "avoid": True,
        #     "euclidean": True,
        #     "waiting": True,
        # })
    }

    all_candidate_paths = {}
    
    testEnv = Environment(testMap.world_data,output_mode=1,uav_mode="follow-candidate-path")
    testASP1 = ASP(name="red", world_data=testMap,origin=(0, 5, 0),radius=20)
    red = UAV(uav_type=0,destinations=[(1,0,2),(3,0,2)],inaccuracy=[0,1],start_time=0,name="red")
    blue = UAV(uav_type=0,destinations=[(2,0,1),(2,0,4)],inaccuracy=[0,0],start_time=0,name="blue")
    green = UAV(uav_type=0,destinations=[(3,0,2),(1,0,2)],inaccuracy=[0,0],start_time=0,name="green")
    yellow = UAV(uav_type=0,destinations=[(0,0,2),(4,0,4)],inaccuracy=[1,0],start_time=0,name="yellow")
    purple = UAV(uav_type=0,destinations=[(3,0,3),(3,0,1)],inaccuracy=[1,0], start_time=1,name="purple")
    testEnv.register_asp(testASP1)
    testEnv.register_uav(red)
    testEnv.register_uav(blue)
    testEnv.register_uav(green)
    # testEnv.register_uav(yellow)
    # testEnv.register_uav(purple)
    
    run_tests(testEnv,0,"follow-candidate-path")
    
    #run_tests(testEnv,0,"wait-on-pass")


    # testMap2 = Map("Test Map", maps.map3,scale=1,repetitions=0)
    # environment2 = Environment(testMap2.world_data)
    # testASP2 = ASP(name="red", world_data=testMap2,origin=(0, 5, 0),radius=20)
    # testUAV3 = UAV(uav_type=0,destinations=[(1,0,1),(3,0,3)],inaccuracy=[1,0])
    # testUAV4 = UAV(uav_type=0,destinations=[(3,0,3),(1,0,1)],inaccuracy=[1,0])
    # testUAV5 = UAV(uav_type=0,destinations=[(4,0,0),(3,0,3)],inaccuracy=[1,0])
    # testUAV6 = UAV(uav_type=0,destinations=[(0,0,0),(0,0,4)],inaccuracy=[1,0],start_time=5)
    # testUAV7 = UAV(uav_type=0,destinations=[(1,0,0),(0,0,0)],inaccuracy=[1,0],start_time=4)
    # testUAV8 = UAV(uav_type=0,destinations=[(0,0,4),(1,0,4)],inaccuracy=[1,0],start_time=9)
    # environment2.register_asp(testASP2)
    # environment2.register_uav(testUAV3)
    # environment2.register_uav(testUAV4)
    # environment2.register_uav(testUAV5)
    # environment2.register_uav(testUAV6)
    # environment2.register_uav(testUAV7)
    # environment2.register_uav(testUAV8)
    # run_tests(environment2,1)
    # environment2.reset_environment()

