from simulator import ASP, Map, Environment,UAV
from simulator.utils.shared_imports import np,byte_unpacker
from simulator import CollisionDetector
from simulator.maps import maps
if __name__ == "__main__":

    asp_list = []
    uav_list = []

    testMap = Map("Test Map", maps.map1, 0)
    print("Shape:", testMap.world_data.shape)
    testASP = ASP("red", (0, 0, 0), 10, asp_list)
    testASP2 = ASP("blue", (10, 10, 4), 5, asp_list)
    
    
    testUAV = UAV(uav_list, 0,[(4,4,4),(7,8,7),(0,0,0)],0.0)
    testUAV.connect_to_asp(asp_list)
    testEnv = Environment(testMap.world_data,uav_list=uav_list)
    testEnv.set_asp_list(asp_list)
    testEnv.set_uav_list(uav_list)
    testEnv.display_coverage()
    print(testUAV)