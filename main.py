from simulator import ASP, Map, Environment,UAV
from simulator.utils.shared_imports import np
from simulator.maps import maps
from simulator.path_planner.path_planner import AStarPlanner
from simulator.tester.tester import run_tests

if __name__ == "__main__":
    all_candidate_paths = {}
    planners = {
        "AStar": AStarPlanner(heuristics={
        }),

        "AStar: euclidean": AStarPlanner(heuristics={
            "euclidean": True
        }),
        "AStar: avoid paths": AStarPlanner(heuristics={
            "avoid": True
        })
    }

    testMap = Map("Test Map", maps.map3,scale=0,repetitions=0)
    
    testEnv = Environment(testMap.world_data,output_mode=1,uav_mode="follow-candidate-path")
    testASP1 = ASP(name="red", world_data=testMap,origin=(0, 5, 0),radius=20)
    red = UAV(uav_type=0,destinations=[(1,0,2),(3,0,2)],inaccuracy=[0,0],start_time=0,max_speed=2,name="red")
    blue = UAV(uav_type=0,destinations=[(2,0,1),(2,0,4)],inaccuracy=[1,0],start_time=0,max_speed=1,name="blue")
    green = UAV(uav_type=0,destinations=[(3,0,2),(1,0,2)],inaccuracy=[0,0],start_time=0,name="green")

    testEnv.register_asp(testASP1)
    testEnv.register_uav(red)
    testEnv.register_uav(blue)
    testEnv.register_uav(green)
    
    run_tests(testEnv,3,"follow-candidate-path",planners=planners,all_candidate_paths=all_candidate_paths)
   
