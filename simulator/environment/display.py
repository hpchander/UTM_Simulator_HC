import matplotlib.pyplot as PLT
import matplotlib.colors as mcolors
import skimage as skim
from simulator.utils.shared_imports import colour_chart, WDG, np

class DisplayManager:
    def __init__(self, environment):
        self.env = environment
        self._uav_location_plots = None
        self._planned_routes_plots = []
        self._destination_plots = []
        self._world_voxels = None
        self._collision_voxels = None
        self._asp_coverage_artists = []
        self._asps_plots = None
        self.render_asp_coverage: np.ndarray = np.swapaxes(self.env.asp_coverage, 1, 2)
        self._uav_footprints = []
        self.fig = PLT.figure(figsize=(15, 20))
        self.axis = self.setup_display(self.env.render_world_data)
        self.start_display()
        self.next_button_axes = None
        self.reset_button_axes = None
        self.next_button = None
        self.reset_button = None


    def add_next_button(self):
        self.next_button_axes = self.fig.add_axes([0.1, 0.25, 0.2, 0.1])
        self.next_button_axes.set_xticks([])
        self.next_button_axes.set_yticks([])
        self.next_button = WDG.Button(
            self.next_button_axes,
            'Next Timestep',
            color='lightblue',
            hovercolor='lightgreen'
        )
        self.next_button.on_clicked(lambda event: self.on_next_button_click())

    def add_reset_button(self):
        self.reset_button_axes = self.fig.add_axes([0.1, 0.5, 0.2, 0.1])
        self.reset_button_axes.set_xticks([])
        self.reset_button_axes.set_yticks([])
        self.reset_button = WDG.Button(
            self.reset_button_axes,
            'Reset Environment',
            color='lightblue',
            hovercolor='lightgreen'
        )
        self.reset_button.on_clicked(lambda event: self.on_reset_button_click())

    def on_next_button_click(self) -> None:
        self.env.next_timestep()
        self.update_display()

    def on_reset_button_click(self) -> None:
        self.env.reset_environment()
        self.env.set_active_candidate_path(self.env.active_candidate_path)
        self.update_display()

    def setup_display(self, render_data: np.ndarray):
        axis = self.fig.add_subplot(111, projection='3d')
        axis.set_title("ASP Coverage Map")
        axis.view_init(elev=90, azim=-90)
        axis.set_xlabel("X")
        axis.set_ylabel("Z")
        axis.set_zlabel("Y")
        axis.set_xlim(0, render_data.shape[0])
        axis.set_ylim(0, render_data.shape[1])
        axis.set_zlim(0, render_data.shape[2])
        axis.view_init(elev=90, azim=-90)
        return axis

    def _display_world(self):
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
            alpha=0.8
        )

    def _display_uavs(self):
        if self._uav_location_plots is not None:
            try:
                self._uav_location_plots.remove()
            except Exception:
                pass
        active_uavs = [uav for uav in self.env.uav_list if uav.start_time <= self.env.timestep and not uav.finished]
        positions = np.array([uav.current_position for uav in active_uavs])
        colours = [colour_chart[uav.id] for uav in active_uavs]
        if positions.size == 0:
            return
        self._uav_location_plots = self.axis.scatter(
            positions[:, 0] + 0.5,
            positions[:, 2] + 0.5,
            positions[:, 1] + 0.5,
            c=colours, s=50
        )

    def _display_uav_footprints(self):
        self._uav_footprints = []
        uav_map = self.env.uav_map
        world_shape = self.env.world_data.shape  # Use world data shape for the masks

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
                facecolors=colour_chart[uav_id],
                edgecolors=colour_chart[uav_id],
                alpha=0.03
            ))

    def _remove_displayed_world(self):
        if hasattr(self, '_world_voxels') and self._world_voxels is not None:
            for artist in self._world_voxels.values():
                artist.remove()
            self._world_voxels = None

    def _remove_displayed_asp_coverage(self):
        if hasattr(self, '_asp_coverage_artists') and self._asp_coverage_artists is not None:
            for art_dict in self._asp_coverage_artists:
                for artist in art_dict.values():
                    artist.remove()
            self._asp_coverage_artists = []

    def _display_asp_coverage(self):
        self._remove_displayed_asp_coverage()
        for asp_id, asp_coverage in enumerate(self.render_asp_coverage):
            face_color = mcolors.to_rgba(colour_chart[asp_id])
            edge_color = face_color[:3] + (0.01,)
            self._asp_coverage_artists.append(self.axis.voxels(
                self.render_asp_coverage,
                facecolors=face_color,
                edgecolors=edge_color,
                alpha=0.5
            ))

    def _removed_displayed_asp_coverage(self):
        if hasattr(self, '_asp_coverage_artists') and self._asp_coverage_artists is not None:
            for art_dict in self._asp_coverage_artists:
                for artist in art_dict.values():
                    artist.remove()
            self._asp_coverage_artists = []

    def _remove_displayed_asps(self):
        if self._asps_plots is not None:
            try:
                self._asps_plots = None
            except ValueError:
                pass

    def _display_asps(self):
        self._remove_displayed_asps()
        colours = [colour_chart[asp.id] for asp in self.env.asp_list]
        positions = np.array([asp.position for asp in self.env.asp_list])
        if positions.size == 0:
            return
        self._asps_plots = self.axis.scatter(
            positions[:, 0],
            positions[:, 2],
            positions[:, 1],
            c=colours, s=100, marker='x'
        )

    def _remove_displayed_uavs(self):
        if self._uav_location_plots is not None:
            try:
                self._uav_location_plots.remove()
            except ValueError:
                pass
            self._uav_location_plots = None

    def _display_planned_routes(self):
        self._remove_displayed_planned_routes()
        active_uavs = [uav for uav in self.env.uav_list]
        for uav in active_uavs:
            if uav.planned_route:
                route = np.array(uav.planned_route)
                planned_route = np.array(route)
                line = self.axis.plot(
                    planned_route[:, 0] + 0.5,
                    planned_route[:, 2] + 0.5,
                    planned_route[:, 1] + 0.5,
                    color=colour_chart[uav.id],
                    linestyle='--'
                )
                self._planned_routes_plots.extend(line)

    def _remove_displayed_planned_routes(self):
        if self._planned_routes_plots is not None:
            for line in self._planned_routes_plots:
                line.remove()
        self._planned_routes_plots = []

    def _display_destinations(self):
        self._remove_displayed_destination_plots()
        active_uavs = [uav for uav in self.env.uav_list]
        for uav in active_uavs:
            for idx, destination in enumerate(uav.destinations):
                text = self.axis.text(
                    destination[0] + 0.6,
                    destination[2] + 0.6,
                    destination[1] + 0.5,
                    f"{idx}",
                    color=colour_chart[uav.id],
                    fontsize=8
                )
                self._destination_plots.append(text)

    def _remove_displayed_destination_plots(self):
        if self._destination_plots is not None:
            for text in self._destination_plots:
                text.remove()
        self._destination_plots = []

    def _display_collisions(self):
        self._remove_displayed_collisions()
        if hasattr(self.env, 'collisions_map') and self.env.collisions_map is not None:
            transposed_map = np.swapaxes(self.env.collisions_map, 1, 2)
            face_color = mcolors.to_rgba('red')
            edge_color = face_color[:3] + (0.01,)
            self._collision_voxels = self.axis.voxels(
                transposed_map,
                facecolors=face_color,
                edgecolors=edge_color,
                alpha=0.5
            )

    def _remove_displayed_collisions(self):
        if hasattr(self, '_collision_voxels') and self._collision_voxels is not None:
            for artist in self._collision_voxels.values():
                artist.remove()
            self._collision_voxels = None

    def start_display(self) -> None:
        self.add_next_button()
        self.add_reset_button()
        self._display_world()
        self._display_uavs()
        self._display_asps()
        self._display_uav_footprints()
        self._display_planned_routes()
        self._display_destinations()
        self._display_collisions()
        PLT.show()

    def update_display(self) -> None:
        self._remove_displayed_world()
        self._remove_displayed_uavs()
        self._remove_displayed_planned_routes()
        self._remove_displayed_destination_plots()
        self._remove_displayed_collisions()
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
        self._display_asps()
        self._display_uavs()
        self.fig.canvas.draw()
