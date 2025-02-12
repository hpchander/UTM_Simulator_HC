from simulator import ASP, Map, Environment,UAV
from simulator.utils.shared_imports import np,byte_unpacker
from simulator import CollisionDetector
from simulator.maps import maps
if __name__ == "__main__":
    testMap = Map("Test Map", maps.map1,scale=2,repetitions=1)
    print("Shape:", testMap.world_data.shape)
    testEnv = Environment(testMap.world_data)
    testASP1 = ASP(name="red", world_data=testMap,origin=(0, 5, 0),radius=20)
    testUAV1 = UAV(uav_type=0,destinations=[(4,0,4),(3,0,1),(0,0,0)],horizontal_accuracy=1)
    testUAV2 = UAV(uav_type=0,destinations=[(1,0,3),(2,0,2),(2,0,1),(1,0,1),(0,0,0)])
    testEnv.register_asp(testASP1)
    testEnv.register_uav(testUAV1)
    testEnv.register_uav(testUAV2)
    testEnv.map_asp_coverage()
    testASP1.connected_uav_positions(testEnv.uav_list)
    testEnv.run()

