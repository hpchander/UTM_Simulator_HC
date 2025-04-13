import numpy as np

def print_city_map_literal(name: str, city: np.ndarray):
    """
    Prints:
        name = np.array([
          [  # layer y=0
            [row0],
            [row1],
            ...
          ],
          [  # layer y=1
            ...
          ],
          ...
        ], dtype=int)
    """
    Y, X, Z = city.shape
    print(f"{name} = np.array([")
    for y in range(Y):
        print("  [")  # start layer y
        for x in range(X):
            # build comma-separated row of Z values
            row_vals = ", ".join(str(city[y, x, z]) for z in range(Z))
            comma = "," if x < X - 1 else ""
            print(f"    [{row_vals}]{comma}")
        layer_comma = "," if y < Y - 1 else ""
        print(f"  ]{layer_comma}")
    print("], dtype=int)")

def generate_city_map(
    x_size: int = 10,
    z_size: int = 10,
    y_size: int = 8,
    road_positions: tuple = (3, 6),
    min_bldg: int = 1,
    max_bldg: int = 7,
    seed: int = None
) -> np.ndarray:
    """
    Generate a city map with shape (y, x, z), so you can index as city[y][x][z].
    
    - 0 = empty (road or air)
    - 1 = building
    Roads run along any x or z in `road_positions`.
    Each non‐road cell gets a random building height in [min_bldg..max_bldg] along y.
    """
    if seed is not None:
        np.random.seed(seed)

    # Note: shape is (y, x, z)
    city = np.zeros((y_size, x_size, z_size), dtype=int)

    for x in range(x_size):
        for z in range(z_size):
            # carve roads at ground and through all heights
            if x in road_positions or z in road_positions:
                # leave city[:, x, z] == 0
                continue

            # random building height
            h = np.random.randint(min_bldg, max_bldg + 1)
            # fill from ground (y=0) up to y=h-1
            city[:h, x, z] = 1

    return city

# Example: generate five different seeds
city_maps = {seed: generate_city_map(seed=seed) for seed in range(5)}

# Access shape and data:
for seed, cmap in city_maps.items():
    # print(f"Ground layer y=0 for seed {seed}:\n")
    # print(f"city_{seed} = np.array(object=[")
    # for i,y in enumerate(cmap):
    #     print("[", end="")
    #     for j,x in enumerate(y):
    #         print(f"{x}",end="")
    #         if j == cmap.shape[1]-1:
    #             print("", end="")
    #         else:
    #             print(",", end="")
    #     if i == cmap.shape[0]-1:
    #      print("]")
    #     else:
    #         print("],")
    
    # print("],dtype=int)")

    print_city_map_literal(f"city_{seed}", cmap)
