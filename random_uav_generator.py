import random



def generate_random_uav(id: int,max_x: int, max_y: int,max_z: int = 0,max_spawntime: int = 0) -> None:
    """"
    Generate a random UAV with 10 random destination.
    """
    name= f"UAV_{id}"
    destinations = []
    for _ in range(5):
        random_x = random.randint(0, max_x)
        random_y = random.randint(0, max_y)
        random_z = random.randint(0, max_z)
        destinations.append([random_x, random_y, random_z])
    inaccuracy = [random.randint(0, 2), random.randint(0, 1)]
    max_speed = random.randint(1, 5)
    start_time = random.randint(0, max_spawntime)
    uav_type = 0
    
    print('{ "name":"'+name+'", "uav_type":'+str(uav_type)+', "destinations":'+str(destinations)+', "inaccuracy":'+str(inaccuracy)+', "start_time":'+str(start_time)+', "max_speed":'+str(max_speed)+' },')
    
for i in range(8):
    generate_random_uav(i,9, 0, 9,0)