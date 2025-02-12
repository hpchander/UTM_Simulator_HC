from simulator.utils.shared_imports import np

class Map:
    def __init__(self, name, world_data: np.array,scale: int = 0, repetitions: int = 0):
        """
        Initialise a Map object.

        :param name: Name of the map (string)
        :param data: 3d Array of map. input in y,x,z and transposed to x,y,z (np.array)
        :param repetitions: Number of times to replicate the first 2D array across the 3D dimension (int)
        """
        self.name = name

        #Scale world data
        if scale != 0:
            world_data = np.repeat(world_data, scale, axis=2)
            world_data = np.repeat(world_data, scale, axis=1)
            world_data = np.repeat(world_data, scale, axis=0)

        # Validate repetitions
        if repetitions < 0:
            raise ValueError("Repetitions must be 0 or a positive integer.")
        
        # Handle repetitions.
        if repetitions == 0:
            self.world_data = world_data
        else:
            first_layer = world_data[0, :, :]
            # Repeat along axis=0 to replicate the first 2D array vertically (in the y direction).
            self.world_data = np.repeat(first_layer[np.newaxis, :, :], repetitions, axis=0)

        self.world_data = np.array(self.world_data.transpose(1, 0, 2))
