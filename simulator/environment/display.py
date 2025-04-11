import matplotlib.pyplot as PLT
import matplotlib.colors as mcolors
from simulator.utils.shared_imports import WDG, np, get_uav_colour
from simulator.utils.config import MAX_DISPLAYED_NODES
class DisplayManager:
    """
    A class to manage the display of the environment and its elements.
    It provides a 3D visualization of the environment, including UAVs, planned routes,
    destinations, collisions, and reservations.
    It also includes control buttons for navigating through different planners and timesteps.
    """
    def __init__(self, environment):
        """
        Initialize the display manager with the given environment.
        """
        self.env = environment
        if not hasattr(self.env, 'render_world_data'):
            raise ValueError("Environment must have render_world_data attribute.")
        if (self.env.render_world_data.shape[0] *
            self.env.render_world_data.shape[1] * self.env.render_world_data.shape[2]) > MAX_DISPLAYED_NODES:
            print(f"Scenario is too large to display. Max size is {MAX_DISPLAYED_NODES} voxels. ")
            return
        self.planner_ids = list(environment.candidate_paths.keys())
        self.current_planner_index = 0

        # UI elements
        self.fig = PLT.figure(figsize=(35, 35))
        self.axis = self.setup_display(self.env.render_world_data)
        self.next_planner_button = None
        self.prev_planner_button = None
        self.reset_button = None
        self.next_timestep_button = None
        self.title_text = None

        self._uav_location_plots = None
        self._planned_routes_plots = []
        self._destination_plots = []
        self._world_voxels = None
        self._collision_voxels = None
        self._reservation_plots = []
        self._uav_footprints = []

        self.add_control_buttons()
        self.select_planner(self.planner_ids[self.current_planner_index])
        self.start_display()

    def add_control_buttons(self):
        """
        Add control buttons to the display.
        """
        # Prev Planner
        ax_prev = self.fig.add_axes([0.01, 0.9, 0.1, 0.05])
        self.prev_planner_button = WDG.Button(ax_prev, 'Prev Planner')
        self.prev_planner_button.on_clicked(lambda event: self.on_prev_planner())
        # Next Planner
        ax_next = self.fig.add_axes([0.12, 0.9, 0.1, 0.05])
        self.next_planner_button = WDG.Button(ax_next, 'Next Planner')
        self.next_planner_button.on_clicked(lambda event: self.on_next_planner())
        # Reset & Step
        ax_next_step = self.fig.add_axes([0.01, 0.8, 0.1, 0.05])
        self.next_timestep_button = WDG.Button(ax_next_step, 'Next Step')
        self.next_timestep_button.on_clicked(lambda event: self.on_next_button_click())
        ax_reset = self.fig.add_axes([0.12, 0.8, 0.1, 0.05])
        self.reset_button = WDG.Button(ax_reset, 'Reset')
        self.reset_button.on_clicked(lambda event: self.on_reset_button_click())

    def select_planner(self, planner_id):
        """
        Select a planner by its ID and update the display accordingly.
        """
        self.env.set_active_candidate_path(planner_id)
        # update title - reuse if exists, else create
        title_str = f"Planner: {planner_id}"
        if self.title_text is None:
            self.title_text = self.fig.suptitle(title_str, fontsize=16)
        else:
            try:
                self.title_text.set_text(title_str)
            except Exception:
                try:
                    self.title_text.remove()
                except Exception:
                    pass
                self.title_text = self.fig.suptitle(title_str, fontsize=16)
        self.update_display()

    def on_next_planner(self):
        """
        Cycle to the next planner in the list.
        """
        self.current_planner_index = (self.current_planner_index + 1) % len(self.planner_ids)
        self.on_reset_button_click()

    def on_prev_planner(self):
        """
        Cycle to the previous planner in the list.
        """
        self.current_planner_index = (self.current_planner_index - 1) % len(self.planner_ids)
        self.on_reset_button_click()

    def on_next_button_click(self) -> None:
        """
        Handle the next button click event.
        """
        self.env.next_timestep()
        self.update_display()

    def on_reset_button_click(self) -> None:
        """
        Handle the reset button click event.
        """
        self.env.reset_environment()
        self.select_planner(self.planner_ids[self.current_planner_index])
        self.update_display()

    def setup_display(self, render_data: np.ndarray):
        """
        Set up the 3D display for the environment.
        """

        axis = self.fig.add_subplot(111, projection='3d')
        axis.set_xlabel("X")
        axis.set_ylabel("Z")
        axis.set_zlabel("Y")
        axis.set_xlim(0, render_data.shape[0])
        axis.set_ylim(0, render_data.shape[2])
        axis.set_zlim(0, render_data.shape[1])
        axis.view_init(elev=90, azim=-90)
        return axis

    def _display_world(self):
        """
        Display the world data in the 3D axis.
        """

        if self._world_voxels is not None:
            for artist in self._world_voxels.values():
                artist.remove()
            self._world_voxels = None
        face_color = mcolors.to_rgba('grey')
        edge_color = face_color[:3] + (0.01,)
        self._world_voxels = self.axis.voxels(
            self.env.render_world_data,
            facecolors=face_color,
            edgecolors=edge_color,
            alpha=0.8,shade=None
        )

    def _display_uavs(self):
        """
        Display the UAVs in the 3D axis.
        """
        if self._uav_location_plots is not None:
            try:
                self._uav_location_plots.remove()
            except Exception:
                pass
        active_uavs = [
            uav for uav in self.env.uav_list 
            if uav.start_time <= self.env.timestep and (not uav.finished or self.env.timestep <= uav.time_finished)
        ]
        positions = np.array([list(uav.current_position) for uav in active_uavs])
        colours = [get_uav_colour(uav.id) for uav in active_uavs]
        if positions.size == 0:
            return
        self._uav_location_plots = self.axis.scatter(
            positions[:, 0] + 0.5,
            positions[:, 2] + 0.5,
            positions[:, 1] + 0.5,
            c=colours, s=50
        )

    def _display_uav_footprints(self):
        """
        Display the footprints of the UAVs in the 3D axis.
        """
        self._uav_footprints = []
        uav_map = self.env.uav_map
        world_shape = self.env.world_data.shape

        # Build a dictionary mapping each UAV id to the list of voxel indices it occupies
        footprints = {}
        for voxel, cell in uav_map.items():
            for uav_id in cell:
                if uav_id not in footprints:
                    footprints[uav_id] = []
                footprints[uav_id].append(voxel)

        # For each UAV, create a boolean mask from its footprint and display it
        for uav_id, voxel_indices in footprints.items():
            mask = np.zeros(world_shape, dtype=bool)
            for coord in voxel_indices:
                mask[coord] = True
            # Flip the axis to match the display
            mask = np.swapaxes(mask, 1, 2)
            self._uav_footprints.append(self.axis.voxels(
                mask,
                facecolors=get_uav_colour(uav_id),
                edgecolors=get_uav_colour(uav_id),
                alpha=0.03,
                shade=None
            ))

    def _remove_displayed_world(self):
        """
        Remove any previously drawn world-voxels.
        """
        if hasattr(self, '_world_voxels') and self._world_voxels is not None:
            for artist in self._world_voxels.values():
                artist.remove()
            self._world_voxels = None


    def _remove_displayed_uavs(self):
        """
        Remove any previously drawn UAVs.
        """
        if self._uav_location_plots is not None:
            try:
                self._uav_location_plots.remove()
            except ValueError:
                pass
            self._uav_location_plots = None

    def _display_planned_routes(self):
        """
        Display the planned routes of the UAVs in the 3D axis.
        """
        self._remove_displayed_planned_routes()
        active_uavs = [uav for uav in self.env.uav_list]
        for uav in active_uavs:
            if uav.planned_route:
                planned_route = np.array([list(s) for s in uav.planned_route])
                offset = (-1 if (uav.id % 2 == 1) else 1) *(uav.id % 10) / 20
                line = self.axis.plot(
                    planned_route[:, 0] + 0.5 + offset,
                    planned_route[:, 2] + 0.5 + offset,
                    planned_route[:, 1] + 0.5,
                    color=get_uav_colour(uav.id),
                    linestyle='--'
                )
                self._planned_routes_plots.extend(line)

    def _remove_displayed_planned_routes(self):
        """
        Remove any previously drawn planned-routes.
        """
        if self._planned_routes_plots is not None:
            for line in self._planned_routes_plots:
                line.remove()
        self._planned_routes_plots = []

    def _display_destinations(self):
        """
        Display the destinations of the UAVs in the 3D axis.
        """
        self._remove_displayed_destination_plots()
        active_uavs = [uav for uav in self.env.uav_list]
        for uav in active_uavs:
            for idx, destination in enumerate(uav.destinations):
                text = self.axis.text(
                    destination.x + 0.6,
                    destination.z + 0.6,
                    destination.y + 0.5,
                    f"{idx}",
                    color=get_uav_colour(uav.id),
                    fontsize=8
                )
                self._destination_plots.append(text)

    def _remove_displayed_destination_plots(self):
        """
        Remove any previously drawn destination-markers.
        """

        if self._destination_plots is not None:
            for text in self._destination_plots:
                text.remove()
        self._destination_plots = []

    def _display_collisions(self):
        """
        Display the collision voxels in the 3D axis.
        """

        self._remove_displayed_collisions()
        if hasattr(self.env, 'collisions_map') and self.env.collisions_map:
            # Create a boolean mask for collisions with the same shape as world_data.
            collision_mask = np.zeros(self.env.world_data.shape, dtype=bool)
            red_color = mcolors.to_rgba('red')
            for voxel, _ in self.env.collisions_map.items():
                collision_mask[voxel] = True

            # Create a transparent facecolors array with shape
            facecolors_array = np.empty(self.env.world_data.shape + (4,), dtype=float)
            facecolors_array[:] = (0, 0, 0, 0)  # fully transparent

            # Set the collision voxels to red.

            facecolors_array[collision_mask] = red_color

            # Swap axes to match the display orientation.
            transposed_mask = np.swapaxes(collision_mask, 1, 2)
            transposed_facecolors = np.swapaxes(facecolors_array, 1, 2)

            self._collision_voxels = self.axis.voxels(
                transposed_mask,
                facecolors=transposed_facecolors,
                edgecolors='k',
                alpha=0.5,
                shade=None
            )

    def _remove_displayed_reservations(self):
        """
        Remove any previously drawn reservation-markers.
        """
        for artist in self._reservation_plots:
            try:
                artist.remove()
            except Exception:
                pass
        self._reservation_plots = []

    def _display_reservations(self):
        """
        Draw black ‘X’ markers at any (x,y,z) whose reservation time t matches
        the current timestep.
        """
        self._remove_displayed_reservations()
        current_t = self.env.timestep
        current = [r for r in self.env.reservations if r[3] == current_t]
        if not current:
            return

        # extract coords and shift by 0.5 so the 'X' sits in the voxel center
        coords = np.array([[x, y, z] for x, y, z, _ in current])
        xs = coords[:, 0] + 0.5
        ys = coords[:, 2] + 0.5
        zs = coords[:, 1] + 0.5

        scatter = self.axis.scatter(
            xs, ys, zs,
            marker='x',
            color='k',
            s=100,
            depthshade=False
        )
        self._reservation_plots = [scatter]

    def _remove_displayed_collisions(self):
        """
        Remove any previously drawn collision-markers.
        """
        if hasattr(self, '_collision_voxels') and self._collision_voxels is not None:
            for artist in self._collision_voxels.values():
                artist.remove()
            self._collision_voxels = None

    def start_display(self) -> None:
        """
        Start the display loop.
        """
        self.add_control_buttons()
        self._display_world()
        self._display_uavs()
        self._display_uav_footprints()
        self._display_planned_routes()
        self._display_destinations()
        self._display_collisions()
        self._display_reservations()
        PLT.show()

    def update_display(self) -> None:
        """
        Update the display with the current state of the environment.
        """
        self._remove_displayed_world()
        self._remove_displayed_uavs()
        self._remove_displayed_planned_routes()
        self._remove_displayed_destination_plots()
        self._remove_displayed_collisions()
        self._remove_displayed_reservations()
        self.axis.clear()
        self.axis.set_xlim(0, self.env.world_data.shape[0])
        self.axis.set_ylim(0, self.env.world_data.shape[2])
        self.axis.set_zlim(0, self.env.world_data.shape[1])
        self.axis.set_title(f"Time: {self.env.timestep}")
        self._display_world()
        self._display_planned_routes()
        self._display_destinations()
        self._display_uav_footprints()
        self._display_collisions()
        self._display_uavs()
        self._display_reservations()
        self.fig.canvas.draw()
