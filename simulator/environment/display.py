import matplotlib.pyplot as PLT
import matplotlib.colors as mcolors
import numpy as np
from simulator.utils.shared_imports import colour_chart, WDG

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

    def setup_display(self, fig, render_data: np.ndarray):
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

    def _display_world(self, axis):
        if self._world_voxels is not None:
            for artist in self._world_voxels.values():
                artist.remove()
            self._world_voxels = None
        face_color = mcolors.to_rgba('grey')
        edge_color = face_color[:3] + (0.01,)
        self._world_voxels = axis.voxels(
            self.env.render_world_data,
            facecolors=face_color,
            edgecolors=edge_color,
            alpha=0.8
        )

    def _display_uavs(self, axis):
        if self._uav_location_plots is not None:
            try:
                self._uav_location_plots.remove()
            except Exception:
                pass
        positions = np.array([uav.current_position for uav in self.env.uav_list])
        colours = [colour_chart[uav.id] for uav in self.env.uav_list]
        self._uav_location_plots = axis.scatter(
            positions[:, 0] + 0.5,
            positions[:, 2] + 0.5,
            positions[:, 1] + 0.5,
            c=colours, s=50
        )

    def _remove_displayed_world(self, axis) -> None:
        """
        Remove the displayed world voxels from the axis.
        """
        if hasattr(self, '_world_voxels') and self._world_voxels is not None:
            for artist in self._world_voxels.values():
                artist.remove()
            self._world_voxels = None
    
    def _remove_displayed_asp_coverage(self, axis) -> None:
        if hasattr(self, '_asp_coverage_artists') and self._asp_coverage_artists is not None:
            for art_dict in self._asp_coverage_artists:
                for artist in art_dict.values():
                    artist.remove()
            self._asp_coverage_artists = []

    def _display_asp_coverage(self,axis) -> None:
        """
        Displays ASP coverage using voxels.
        """
        self._remove_displayed_asp_coverage(axis)
        for asp_id, asp_coverage in enumerate(self.render_asp_coverage):
            asp_coverage = self.render_asp_coverage
            face_color = mcolors.to_rgba(colour_chart[asp_id])
            edge_color = face_color[:3] + (0.01,)
            self._asp_coverage_artists.append(axis.voxels(
                asp_coverage,
                facecolors=face_color,
                edgecolors=edge_color,
                alpha=0.5
            ))

    def _removed_displayed_asp_coverage(self,axis) -> None:
        if hasattr(self, '_asp_coverage_artists') and self._asp_coverage_artists is not None:
            for art_dict in self._asp_coverage_artists:
                for artist in art_dict.values():
                    artist.remove()
            self._asp_coverage_artists = []

    def _remove_displayed_asps(self,axis) -> None:
        if self._asps_plots is not None:
            try:
                self._asps_plots = None
            except ValueError:
                pass

    def _display_asps(self,axis) -> None:
        """
        Displays ASPs as a marker on the map. (if they are within world boundaries)
        """

        self._remove_displayed_asps(axis)
        colours = [colour_chart[asp.id] for asp in self.env.asp_list]
        positions = np.array([asp.position for asp in self.env.asp_list])
        self._asps_plots = axis.scatter(
            positions[:, 0],
            positions[:, 2],
            positions[:, 1],
            c=colours, s=100, marker='x'
        )
        
    
    def _remove_displayed_uavs(self, axis) -> None:
        if self._uav_location_plots is not None:
            try:
                self._uav_location_plots = None
            except ValueError:
                pass
    
    def _display_planned_routes(self,axis) -> None:
        """
        Display the planned routes of the UAVs.
        """
        self._remove_displayed_planned_routes(axis)
        for uav in self.env.uav_list:
            if uav.planned_route:
                route = np.array(uav.planned_route)
                planned_route = np.array(route)
                line = axis.plot(
                    planned_route[:, 0] + 0.5,
                    planned_route[:, 2] + 0.5,
                    planned_route[:, 1] + 0.5,
                    color=colour_chart[uav.id],
                    linestyle='--'
                )
                self._planned_routes_plots.extend(line)

    def _remove_displayed_planned_routes(self,axis) -> None:
        if self._planned_routes_plots is not None:
            for line in self._planned_routes_plots:
                line.remove()
        self._planned_routes_plots = []
            
    def _display_destinations(self,axis) -> None:
        """
        Display the destinations of the UAVs as numbers.
        """
        self._remove_displayed_destination_plots(axis)
        for uav in self.env.uav_list:
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
    
    def _remove_displayed_destination_plots(self,axis) -> None:
        if self._destination_plots is not None:
            for text in self._destination_plots:
                text.remove()
        self._destination_plots = []

    def _display_collisions(self, axis) -> None:
        """
        Display the collisions in the simulation by overlaying a semi-transparent
        red voxel plot on the regions where collisions are detected.

        This function uses the boolean array self.collisions_map, which should be
        computed (for example, by calling self.detect_collisions()) prior to display.
        Voxels with a collision (True values) are highlighted in red.
        """
        if not hasattr(self, 'collisions_map'):
            return

        collision_mask = self.collisions_map.transpose(0, 2, 1)
        self._remove_displayed_collisions(axis)
        
        if collision_mask.any():
            self._collision_voxels = axis.voxels(
                collision_mask,
                facecolors='red',
                edgecolors='darkred',
                alpha=0.5
            )

    def _remove_displayed_collisions(self,axis):
         if hasattr(self, '_collision_voxels') and self._collision_voxels is not None:
            for artist in self._collision_voxels.values():
                artist.remove()
            self._collision_voxels = None

    def _remove_displayed_uavs(self,axis) -> None:
        """
        Remove the displayed UAV locations from the axis.
        """
        if self._uav_location_plots is not None:
            try:
                self._uav_location_plots.remove()
            except ValueError:
                pass
            self._uav_location_plots = None

    def start_display(self,axis,fig) -> None:
        """
        Start the display of the simulation.
        """
        self._display_world(axis)
        self._display_uavs(axis)
        self._display_asps(axis)
        self._display_asp_coverage(axis)
        self._display_planned_routes(axis)
        self._display_destinations(axis)
        self._display_collisions(axis)
        fig.canvas.draw()
        PLT.show()


    def update_display(self,axis,fig) -> None:
        """
        Update the display with the current simulation state.
        """
        self._remove_displayed_world(axis)
        self._remove_displayed_uavs(axis)
        self._remove_displayed_planned_routes(axis)
        self._remove_displayed_destination_plots(axis)
        self._remove_displayed_asp_coverage(axis)
        self._remove_displayed_collisions(axis)
        axis.clear()
        axis.set_xlim(0, self.env.world_data.shape[0])
        axis.set_ylim(0, self.env.world_data.shape[2])
        axis.set_zlim(0, self.env.world_data.shape[1])
        axis.view_init(elev=90, azim=-90)
        self._display_asp_coverage(axis)
        self._display_world(axis)
        self._display_planned_routes(axis)
        self._display_destinations(axis)
        self._display_collisions(axis)
        self._display_asps(axis)
        self._display_uavs(axis)
        fig.canvas.draw()
