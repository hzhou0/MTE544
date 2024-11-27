from itertools import starmap

from mapUtilities import *
from a_star import *

POINT_PLANNER=0; TRAJECTORY_PLANNER=1

class Planner:
    def __init__(self, type_, mapName="room"):

        self.type=type_
        if "SIM" in os.environ:
            print("Planner running with sim map")
        mapName="./test_map/map.yaml"
        self.mapName=mapName


    def plan(self, startPose, endPose):

        if self.type==POINT_PLANNER:
            return self.point_planner(endPose)

        elif self.type==TRAJECTORY_PLANNER:
            self.costMap=None
            self.initTrajectoryPlanner()
            return self.trajectory_planner(startPose, endPose)


    def point_planner(self, endPose):
        return endPose

    def initTrajectoryPlanner(self):


        # TODO PART 5 Create the cost-map, the laser_sig is
        # the standard deviation for the gaussian for which
        # the mean is located on the occupant grid.

        # The turtlebot claims Distance Accuracy(500mm ~ 3,500mm)	±5.0%.
        # The worst case is 0.175. To be safe, we set this to 0.1.
        self.m_utilites=mapManipulator(self.mapName, laser_sig=0.1)

        self.costMap=self.m_utilites.make_likelihood_field()


    def trajectory_planner(self, startPoseCart, endPoseCart):


        # This is to convert the cartesian coordinates into
        # the pixel coordinates of the map image, remember,
        # the cost-map is in pixels.
        # You can, by the way, convert the pixels
        # to the cartesian coordinates and work by that index; the a_star finds
        # the path regardless.
        startPose=self.m_utilites.position_2_cell(startPoseCart)
        endPose=self.m_utilites.position_2_cell(endPoseCart)

        # TODO PART 5 convert the cell pixels into the cartesian coordinates
        path = search(self.costMap, startPose, endPose)
        path = [self.m_utilites.cell_2_position(point) for point in path]



        # TODO PART 5 return the path as list of [x,y]
        return path




if __name__=="__main__":
    rclpy.init()

    # m_utilites=mapManipulator("./test_map/map.yaml", 0.1)
    #
    # map_likelihood=m_utilites.make_likelihood_field()

    # Testing the planner
    planner=Planner(TRAJECTORY_PLANNER, "./test_map/map.yaml")
    planner.initTrajectoryPlanner()
    print(planner.costMap.shape)
    path=planner.trajectory_planner(planner.m_utilites.cell_2_position((60, 150)), planner.m_utilites.cell_2_position((10,10)))
    path_cells=[planner.m_utilites.position_2_cell(point) for point in path]
    x_coords, y_coords = zip(*path_cells)
    plt.imshow(planner.costMap, cmap="gray")
    plt.scatter(x_coords, y_coords, color='red', s=50, marker='.', label="Points")
    plt.axis("off")
    plt.title("PGM Image")
    plt.legend()
    plt.show()

    # you can use this part of the code to test your
    # search algorithm regardless of the ros2 hassles

