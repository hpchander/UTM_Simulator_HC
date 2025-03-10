from simulator import ASP, Map, Environment,UAV
from simulator.utils.shared_imports import np,byte_unpacker
from simulator.maps import maps
from simulator.path_planner.path_planner import AStarPlanner


def run_tests(environment: Environment, mode=1):
    print("Running tests")
    results = {}
    
    for name, planner in planners.items():
        candidate = planner.plan_path(environment)
        all_candidate_paths[name] = candidate
    for name, candidate in all_candidate_paths.items():
        environment.candidate_paths[name] = candidate

        if mode == 1: print(f"{name} candidate path: {candidate}")
        environment.set_active_candidate_path(name)
        print(f"Running {name} path")
        results[name] =  environment.run(output_mode=mode)
        environment.reset_environment()
        environment.set_active_candidate_path(name)
        if mode == 1:
            environment.display()
    lowest_names = []
    lowest = np.inf
    for name, results in results.items():
        if results["timesteps"] < lowest:
            lowest_names = [name]
            lowest = results["timesteps"]
            lowest_name = name
        elif results["timesteps"] == lowest:
            lowest_names.append(name)
        print(f"{name} results: {results}")
        print("\n\n")
    print(f"Lowest timesteps: {lowest} for {lowest_names}")
    

    
    print("Tests complete")

if __name__ == "__main__":
    #load map
    testMap = Map("Test Map", maps.map2,scale=1,repetitions=0)

    

    planners = {
        "AStar": AStarPlanner(heuristics={
        }),

        "AStar: euclidean": AStarPlanner(heuristics={
            "euclidean": True
        }),
        "AStar: avoid paths": AStarPlanner(heuristics={
            "avoid": True
        }),

        "AStar: euclidean + avoid paths": AStarPlanner(heuristics={
            "avoid": True,
            "euclidean": True
        }),

        "AStar: euclidean + avoid paths + waiting": AStarPlanner(heuristics={
            "avoid": True,
            "euclidean": True,
            "waiting": True,
        })



    }

    all_candidate_paths = {}
    
    testEnv = Environment(testMap.world_data)
    testASP1 = ASP(name="red", world_data=testMap,origin=(0, 5, 0),radius=20)
    testUAV1 = UAV(uav_type=0,destinations=[(0,0,1),(2,0,1)],horizontal_accuracy=0,vertical_accuracy=0)
    testUAV2 = UAV(uav_type=0,destinations=[(1,0,0),(1,0,2)],horizontal_accuracy=0,vertical_accuracy=0,start_time=0)
    testUAV3 = UAV(uav_type=0,destinations=[(1,0,1),(1,0,2)],horizontal_accuracy=0,vertical_accuracy=0,start_time=1)
    testEnv.register_asp(testASP1)
    testEnv.register_uav(testUAV1)
    testEnv.register_uav(testUAV3)
    testEnv.register_uav(testUAV2)
    
    run_tests(testEnv,1)
    testEnv.reset_environment()


    # testMap2 = Map("Test Map", maps.map3,scale=1,repetitions=0)
    # environment2 = Environment(testMap2.world_data)
    # testASP2 = ASP(name="red", world_data=testMap2,origin=(0, 5, 0),radius=20)
    # testUAV3 = UAV(uav_type=0,destinations=[(1,0,1),(3,0,3)],horizontal_accuracy=0,vertical_accuracy=0)
    # testUAV4 = UAV(uav_type=0,destinations=[(3,0,3),(1,0,1)],horizontal_accuracy=0,vertical_accuracy=0)
    # testUAV5 = UAV(uav_type=0,destinations=[(4,0,0),(3,0,3)],horizontal_accuracy=0,vertical_accuracy=0)
    # testUAV6 = UAV(uav_type=0,destinations=[(0,0,0),(0,0,4)],horizontal_accuracy=0,vertical_accuracy=0,start_time=5)
    # testUAV7 = UAV(uav_type=0,destinations=[(1,0,0),(0,0,0)],horizontal_accuracy=0,vertical_accuracy=0,start_time=4)
    # testUAV8 = UAV(uav_type=0,destinations=[(0,0,4),(1,0,4)],horizontal_accuracy=0,vertical_accuracy=0,start_time=9)
    # environment2.register_asp(testASP2)
    # environment2.register_uav(testUAV3)
    # environment2.register_uav(testUAV4)
    # environment2.register_uav(testUAV5)
    # environment2.register_uav(testUAV6)
    # environment2.register_uav(testUAV7)
    # environment2.register_uav(testUAV8)
    # run_tests(environment2,1)
    # environment2.reset_environment()

