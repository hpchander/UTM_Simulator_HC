from simulator.utils.shared_imports import np

class Map:
    def __init__(self, name, world_data: np.array, repetitions):
        """
        Initialise a Map object.

        :param name: Name of the map (string)
        :param data: 3D array representing the world (list of lists or numpy array)
        :param repetitions: Number of times to replicate the first 2D array across the 3D dimension (int)
        """
        self.name = name

        # Validate repetitions
        if repetitions < 0:
            raise ValueError("Repetitions must be 0 or a positive integer.")
        
        # Handle repetitions
        if repetitions == 0:
            self.world_data = world_data
        else:
            first_layer = world_data[0, :, :]
            self.world_data = np.repeat(first_layer[np.newaxis, :, :], repetitions, axis=1)