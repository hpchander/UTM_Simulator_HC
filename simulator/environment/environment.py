from typing import Any, List, Optional
from simulator.utils.shared_imports import np, Math, PLT, colour_chart,WDG
import matplotlib.colors as mcolors


class Environment:
    def __init__(
        self,
        world_data: np.ndarray
    ) -> None:
        """
        Initialize the simulation environment.

        Args:
            world_data (np.ndarray): A 3D numpy array representing the simulation world.
            asp_list (Optional[List[Any]]): List of ASP objects (default is None).
            uav_list (Optional[List[Any]]): List of UAV objects (default is None).
        """
        self.world_data: np.ndarray = world_data
        self.render_world_data: np.ndarray = np.swapaxes(self.world_data, 1, 2)
        self.asp_list: List[Any] = []
        self.uav_list: List[Any] = []

        # Initialize asp_coverage as an array of lists (one list per voxel)
        self.asp_coverage: np.ndarray = np.empty(self.world_data.shape, dtype=object)
        for idx in np.ndindex(self.asp_coverage.shape):
            self.asp_coverage[idx] = []

        self.map_asp_coverage()
        self.render_asp_coverage: np.ndarray = np.swapaxes(self.asp_coverage, 1, 2)
        self.uav_map: np.ndarray = np.empty(self.world_data.shape, dtype=object)
        self.states: List[np.ndarray] = []
        self.leased_airspace: List[np.ndarray] = []
        self.collisions_map = None
        self.timestep:int = 0

        self._uav_location_plots = None
        self._planned_routes_plots = None
        self._destination_plots = None
        self._world_voxels = None
        self._collision_voxels = None
        self._uav_voxels = None
        self._asp_coverage_voxels = None


    def reset_environment(self) -> None:
        """
        Reset the environment's simulation state to its initial conditions.
        (This method does not close the figure window; that is handled in the reset callback.)
        """
        self.timestep = 0
        for uav in self.uav_list:
            uav.reset_uav()
        self.states = []
        self.leased_airspace = []
        self.collisions_map = None
        
        self._uav_location_plots = None
        self._planned_routes_plots = None
        self._asps_plots = None
        self._destination_plots = None
        self._world_voxels = None
        self._collision_voxels = None
        self._uav_voxels = None
        self._asp_coverage_artists = None



    def run(self) -> None:
        """
        Run the simulation.
        """
        self._uav_map()
        self.display()

    def register_asp(self, new_asp: Any) -> None:
        """Register an ASP object."""
        taken_ids = [asp.id for asp in self.asp_list]
        for i in range(100):
            if i not in taken_ids:
                new_asp.id = i
                break
        if new_asp.id == -1:
            raise ValueError("Could not assign a unique ID to the ASP.")
        self.asp_list.append(new_asp)

    def register_uav(self, new_uav: Any) -> None:
        """Register a UAV object."""
        taken_ids = [uav.id for uav in self.uav_list]
        for i in range(100):
            if i not in taken_ids:
                new_uav.id = i
                break
        if new_uav.id == -1:
            raise ValueError("Could not assign a unique ID to the UAV.")
        self.uav_list.append(new_uav)

    def display(self) -> None:
        """
        Display the simulation, including the world, ASP coverage, and ASP positions.
        """
        # Create the figure and axis for display
        fig = PLT.figure(figsize=(15, 20))
        axis = self._setup_display(fig, self.render_world_data)
        #next button
        next_button_axes = fig.add_axes([0.1, 0.25, 0.2, 0.1])
        next_button_axes.set_xticks([])
        next_button_axes.set_yticks([])
        next_button = WDG.Button(
            next_button_axes,
            'Next Timestep',
            color='lightblue',
            hovercolor='lightgreen'
        )
        next_button.on_clicked(lambda event: self.on_next_button_click(axis,fig))
        #reset button
        reset_button_axes = fig.add_axes([0.1, 0.5, 0.2, 0.1])
        reset_button_axes.set_xticks([])
        reset_button_axes.set_yticks([])
        reset_button = WDG.Button(
            reset_button_axes,
            'Reset Environment',
            color='lightblue',
            hovercolor='lightgreen'
        )
        reset_button.on_clicked(lambda event: self.on_reset_button_click(axis,fig))
        # Display different simulation elements
        self._display_world(axis)
        self._display_uavs(axis)
        self._display_planned_routes(axis)
        self._display_destinations(axis)
        self._display_asp_coverage(axis)
        
        self._uav_map()
        self.detect_collisions()
        self._display_collisions(axis)
        self._display_uav_map(axis)
        
        PLT.show()
        
    def on_next_button_click(self,axis,fig) -> None:
        """
        Progress the simulation by one timestep when the button is clicked.
        """
        self.next_timestep()
        self._uav_map()
        self.detect_collisions()
        self._update_display(axis,fig)

    def on_reset_button_click(self, axis,fig) -> None:
        """
        Reset the environment when the reset button is clicked.
        This callback closes the current figure, resets internal state, and re-displays the simulation.
        """
        self.reset_environment()
        self._update_display(axis,fig)

    def _update_display(self,axis: Any,fig:Any) -> None:
        """
        Update the display with the current simulation state.
        """
        self._display_world(axis)
        self._display_uavs(axis)
        self._display_planned_routes(axis)
        self._display_destinations(axis)
        self._display_asp_coverage(axis)
        

        self.detect_collisions()
        self._display_collisions(axis)
        self._uav_map()
        self._display_uav_map(axis)

        fig.canvas.draw()


    def next_timestep(self) -> None:
        """
        Progress time by one timestep.
        """
        self.states.append(np.copy(self.uav_map))
        self.timestep += 1
        for uav in self.uav_list:
            uav.connect_to_asp(self.asp_list)
            uav.move()
    
    def map_asp_coverage(self) -> None:
        """
        For each ASP, mark the voxels within its coverage radius.
        Each voxel within range will have the ASP's id appended to its list.
        """
        for asp in self.asp_list:
            position = asp.position  # Expected to be a tuple like (x, y, z)
            radius = asp.radius

            # Calculate the bounding box limits for the ASP's coverage
            x_min = max(0, int(position[0] - radius))
            x_max = min(self.world_data.shape[0], int(position[0] + radius) + 1)
            y_min = max(0, int(position[1] - radius))
            y_max = min(self.world_data.shape[1], int(position[1] + radius) + 1)
            z_min = max(0, int(position[2] - radius))
            z_max = min(self.world_data.shape[2], int(position[2] + radius) + 1)

            for x in range(x_min, x_max):
                for y in range(y_min, y_max):
                    for z in range(z_min, z_max):
                        # Compute Euclidean distance from the voxel to the ASP's position
                        distance = Math.sqrt(
                            (x - position[0]) ** 2 +
                            (y - position[1]) ** 2 +
                            (z - position[2]) ** 2
                        )
                        if distance <= radius:
                            self.asp_coverage[x, y, z].append(asp.id)

    def _display_world(self, axis: Any) -> None:
        """
        Display the simulation world (obstacles) using voxels.
        Clears any previous world drawings before redrawing.
        """
        # Remove previously drawn world voxels, if any.
        if hasattr(self, '_world_voxels') and self._world_voxels is not None:
            for artist in self._world_voxels.values():
                artist.remove()
            self._world_voxels = None

        face_color = mcolors.to_rgba('grey')
        edge_color = face_color[:3] + (0.01,)
        # Draw the world voxels and store the returned dictionary of artists.
        self._world_voxels = axis.voxels(
            self.render_world_data,
            facecolors=face_color,
            edgecolors=edge_color,
            alpha=0.8
        )

    def _display_asp_coverage(self, axis: Any) -> None:
        """
        Display the ASP coverage map.
        Clears any previous ASP coverage drawings before redrawing.
        """
        if hasattr(self, '_asp_coverage_artists') and self._asp_coverage_artists is not None:
            for art_dict in self._asp_coverage_artists:
                for artist in art_dict.values():
                    artist.remove()
            self._asp_coverage_artists = []

        self._asp_coverage_artists = []
        for asp in self.asp_list:
            mask = np.zeros(self.render_world_data.shape, dtype=bool)
            for idx in np.ndindex(self.render_asp_coverage.shape):
                if asp.id in self.render_asp_coverage[idx]:
                    mask[idx] = True

            if mask.any():
                face_color = mcolors.to_rgba(colour_chart[asp.id])
                edge_color = face_color[:3] + (0.01,)
                art_dict = axis.voxels(mask, facecolors=face_color, edgecolors=edge_color, alpha=0.1)
                self._asp_coverage_artists.append(art_dict)


    def _display_asps(self, axis: Any) -> None:
        """
        Plot the positions of ASPs on the axis.
        Clears any previous ASP drawings before redrawing.
        """
        if hasattr(self, '_asps_plots') and self._asps_plots is not None:
            for artist in self._asps_plots:
                artist.remove()
            self._asps_plots = []

        self._asps_plots = []
        for asp in self.asp_list:
            # Note: swapping y and z to match the display conventions
            x, y, z = asp.position
            line, = axis.plot(
                [x],
                [z],
                [y],
                marker='o',
                markersize=5,
                color=colour_chart[asp.id]
            )
            self._asps_plots.append(line)
    
    def _display_uavs(self,axis: Any) -> None:
        """
        Display the UAVs in the simulation.
        """
        if self._uav_location_plots is not None:
            self._uav_location_plots.remove()
        positions = np.array([uav.current_position for uav in self.uav_list])
        colours = [colour_chart[uav.id] for uav in self.uav_list]
        self._uav_location_plots = axis.scatter(positions[:, 0] + 0.5, positions[:, 2] + 0.5,positions[:, 1] + 0.5,
                                    c=colours, s=50)
    
    def _display_planned_routes(self,axis: Any) -> None:
        """
        Display the planned routes of the UAVs.
        """
        if self._planned_routes_plots is not None:
            for line in self._planned_routes_plots:
                line.remove()
        self._planned_routes_plots = []
        for uav in self.uav_list:
            if uav.planned_route:
                route = [uav.destinations[0]] + uav.planned_route
                planned_route = np.array(route)
                line = axis.plot(
                    planned_route[:, 0] + 0.5,
                    planned_route[:, 2] + 0.5,
                    planned_route[:, 1] + 0.5,
                    color=colour_chart[uav.id],
                    linestyle='--'
                )
                self._planned_routes_plots.extend(line)
        
            
    def _display_destinations(self,axis: Any) -> None:
        """
        Display the destinations of the UAVs as numbers.
        """
        if self._destination_plots is not None:
            for text in self._destination_plots:
                text.remove()
        self._destination_plots = []
        for uav in self.uav_list:
            for idx, destination in enumerate(uav.destinations):
                text = axis.text(
                    destination[0] + 0.6,
                    destination[2] + 0.6,
                    destination[1] + 0.5,
                    f"{idx}",
                    color=colour_chart[uav.id],
                    fontsize=8
                )
                self._destination_plots.append(text)

    def _uav_map(self) -> None:
        """
        Clear and repopulate the UAV map based on the current positions and inaccuracy of each UAV.
        
        For each UAV in self.uav_list, determine its footprint (i.e. the set of voxels it may occupy)
        based on its current_position and its accuracy attributes. Then, for each voxel in that footprint,
        append the UAV's identifier to the list stored in that voxel.
        """
        for idx in np.ndindex(self.uav_map.shape):
            self.uav_map[idx] = []

        for uav in self.uav_list:
            x, y, z = uav.current_position
            
            horiz = uav.horizontal_accuracy if hasattr(uav, 'horizontal_accuracy') else 0
            vert = uav.vertical_accuracy if hasattr(uav, 'vertical_accuracy') else 0

            x_min = max(0, x - horiz)
            x_max = min(self.uav_map.shape[0] - 1, x + horiz)
            y_min = max(0, y - vert)
            y_max = min(self.uav_map.shape[1] - 1, y + vert)
            z_min = max(0, z - horiz)
            z_max = min(self.uav_map.shape[2] - 1, z + horiz)
            for i in range(x_min, x_max + 1):
                for j in range(y_min, y_max + 1):
                    for k in range(z_min, z_max + 1):
                        self.uav_map[i, j, k].append(uav.id)

    def detect_collisions(self) -> None:
        """
        Check for collisions between UAVs and the world.
        A collision is marked in self.collisions_map (a boolean array)
        if either:
        - More than one UAV occupies the same voxel, or
        - A UAV occupies a voxel that contains an obstacle in the world data.
        """
        self.collisions_map = np.zeros(self.world_data.shape, dtype=bool)

        for idx in np.ndindex(self.uav_map.shape):
            if len(self.uav_map[idx]) > 1:
                self.collisions_map[idx] = True
            elif self.world_data[idx] == 1 and len(self.uav_map[idx]) > 0:
                self.collisions_map[idx] = True
            else:
                self.collisions_map[idx] = False

    def _display_uav_map(self, axis: Any) -> None:
        """
        Display the UAV map by overlaying a semi-transparent voxel plot on the regions
        where UAVs are detected. Each voxel is colored according to the first UAV present.
        """
        if hasattr(self, '_uav_voxels') and self._uav_voxels is not None:
            for artist_dict in self._uav_voxels.values():
                for artist in artist_dict.values():
                    artist.remove()
            self._uav_voxels = {}
        
        has_uav = any(len(self.uav_map[idx]) > 0 for idx in np.ndindex(self.uav_map.shape))
        if not has_uav:
            return

        map_copy = np.copy(self.uav_map).transpose(0, 2, 1)
        self._uav_voxels = {}
        for idx in np.ndindex(self.uav_map.shape):
            if not self.uav_map[idx]:
                continue

            colour = colour_chart[self.uav_map[idx][0]]

            mask = np.zeros(self.uav_map.shape, dtype=bool)
            mask[idx] = True

            base_rgba = mcolors.to_rgba(colour)
            edge_rgba = (base_rgba[0], base_rgba[1], base_rgba[2], 0.1)

            voxel_artists = axis.voxels(
                mask.transpose(0, 2, 1),
                facecolors=base_rgba,
                edgecolors=edge_rgba,
                alpha=0.5
            )
            self._uav_voxels[idx] = voxel_artists



    def _display_collisions(self, axis: Any) -> None:
        """
        Display the collisions in the simulation by overlaying a semi-transparent
        red voxel plot on the regions where collisions are detected.

        This function uses the boolean array self.collisions_map, which should be
        computed (for example, by calling self.detect_collisions()) prior to display.
        Voxels with a collision (True values) are highlighted in red.
        """
        if not hasattr(self, 'collisions_map'):
            self.detect_collisions()

        collision_mask = self.collisions_map.transpose(0, 2, 1)
        

        if hasattr(self, '_collisions_voxels') and self._collision_voxels is not None:
            for artist in self._collision_voxels.values():
                artist.remove()
            self._collision_voxels = None
        
        if collision_mask.any():
            self._collision_voxels = axis.voxels(
                collision_mask,
                facecolors='red',
                edgecolors='darkred',
                alpha=0.5
            )


    @staticmethod
    def _setup_display(fig: Any, render_data: np.ndarray) -> Any:
        """
        Set up the 3D display axis.
        
        Args:
            fig (Any): The matplotlib figure.
            render_data (np.ndarray): Data used to set axis limits.
        
        Returns:
            Any: The configured 3D axis.
        """
        axis = fig.add_subplot(111, projection='3d')
        axis.set_title("ASP Coverage Map")
        axis.set_xlabel("X")
        axis.set_ylabel("Z")
        axis.set_zlabel("Y")
        axis.set_xlim(0, render_data.shape[0])
        axis.set_ylim(0, render_data.shape[1])
        axis.set_zlim(0, render_data.shape[2])
        axis.view_init(elev=90, azim=-90)
        return axis

            