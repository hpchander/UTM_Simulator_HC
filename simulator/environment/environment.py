from simulator.utils.shared_imports import np,Math,MLP,colour_chart

class Environment:
    def __init__(self,world_data: np.ndarray,asp_list: list = [],uav_list: list = []):
        self.world_data = np.array(world_data)
        self.asp_list = asp_list
        self.uav_list = uav_list

    def set_asp_list(self, asp_list):
        self.asp_list = asp_list

    def set_uav_list(self, uav_list):
        self.uav_list = uav_list
    
    def display_coverage(self):
        """
        Shows coverage map
        """
        fig = MLP.figure(figsize=(15,20))
        transposed_world_data = self.world_data.transpose(1, 2, 0)
        ax = Environment.display_setup(fig,transposed_world_data)
        ax.voxels(transposed_world_data, facecolors='blue', edgecolors='k', alpha=0.8)
        self.display_asp(ax)
       
        
        for uav in self.uav_list: 
            if uav.connected_asp is not None:
                ax.plot(
                    [uav.connected_asp.position[0], uav.position[0]],
                    [uav.connected_asp.position[1], uav.position[1]],
                    [uav.connected_asp.position[2], uav.position[2]],
                    label=f"UAV {uav.id} Path", color=colour_chart(uav.connected_asp.id),zorder =80, alpha=0.2
                )
            counter = 0
            for destination in uav.destinations:
                destination_number = ax.text(destination[0], destination[1], destination[2], str(counter), color=colour_chart(uav.id))
                destination_number.set_clip_on(False)
                destination_number.set_zorder(120)
                counter += 1
            plotted_uav = ax.plot( uav.position[0], uav.position[1], uav.position[2], marker='o', markersize=8, color="black")
            plotted_uav[0].set_clip_on(False)
            plotted_uav[0].set_zorder(121)
            # if len(uav.destinations) > 1:
            #     x_coords = [point[0] for point in uav.destinations]
            #     y_coords = [point[1] for point in uav.destinations]
            #     z_coords = [point[2] for point in uav.destinations]
            #     ax.plot(
            #         x_coords, y_coords, z_coords, label=f"UAV {uav.id} Path", color=colour_chart(uav.id)
            #     )


        elevation = 20
        azimuth = 45
        for i in range(0, 1000):
            ax.view_init(elev=elevation, azim=azimuth)
            #MLP.pause(0.01)
            azimuth += 2

        MLP.show()

    def display_setup(fig,transposed_world_data):
        ax = fig.add_subplot(111, projection='3d')
        ax.set_title("ASP Coverage Map")
        ax.set_xlabel("X")
        ax.set_ylabel("Z")
        ax.set_zlabel("Y")
        ax.set_xlim(0,transposed_world_data.shape[0])
        ax.set_ylim(0,transposed_world_data.shape[1])
        ax.set_zlim(0,transposed_world_data.shape[2])
        return ax
    
    def display_asp(self,ax):
        for asp in self.asp_list:
            position = asp.position
            radius = asp.radius
            ax.plot([position[0]], [position[1]], [position[2]], marker='o', markersize=5, color=colour_chart(asp.id))
            u = np.linspace(0, 2 * np.pi, 100)
            v = np.linspace(0, np.pi, 100)
            x = position[0] + radius * np.outer(np.cos(u), np.sin(v))
            y = position[1] + radius * np.outer(np.sin(u), np.sin(v))
            z = position[2] + radius * np.outer(np.ones(np.size(u)), np.cos(v))
            surface = ax.plot_surface(x, y, z, color=colour_chart(asp.id), alpha=0.05, edgecolor='none')
            surface.set_clip_on(True)