class ASP:
    def __init__(self, name: str, origin: tuple, radius: float, asp_list: list):
        """
        Initialise an Airspace Service Provider (ASP) object.

        :param name: Name of the ASP (string)
        :param origin: 3D coordinates of the origin point (tuple)
        :param shape: Shape of the ASP's area (e.g., "square") (string)
        :param radius: Radius defining the ASP's coverage (int or float)
        :param asp_list: List of current ASPs to which this ASP will be added
        """
        self.name: str = name
        self.position: tuple = origin
        self.radius: float = radius
        self.horizontal_velocity: float = 0
        self.vertical_velocity: float = 0
        taken_ids = [asp.id for asp in asp_list]
        for i in range(10000):
            if i not in taken_ids:
                self.id = i
                break
        if self.id == None:
            raise ValueError("Could not assign a unique ID to the ASP.")
            self.id = -1
        asp_list.append(self)

    def collision_radius(self,weather):
        return 1


print("ASP Loaded")