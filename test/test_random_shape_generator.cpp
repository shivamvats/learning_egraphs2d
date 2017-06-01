#include <ros/ros.h>
#include <smpl/occupancy_grid.h>
#include <visualization_msgs/MarkerArray.h>
#include <smpl/debug/visualizer_ros.h>

#include "shape.h"
#include "random_shape_generator.h"
#include "grid_collision_checker_2d.h"


sbpl::OccupancyGrid setupOccupancyGrid(std::string planning_frame) {
    const double grid_res = 0.02;
    const double world_size_x = 5.0;
    const double world_size_y = 5.0;
    const double world_size_z = 1.5 * grid_res;
    const double world_origin_x = 0.0;
    const double world_origin_y = 0.0;
    const double world_origin_z = 0.0;
    const double max_distance_m = 0.2;
    const bool ref_count = false;
    ROS_ERROR("Setting up occupancy grid of size %f x %f and origin %f, %f",
                world_size_x, world_size_y, world_origin_x, world_origin_y);
    sbpl::OccupancyGrid grid(
            world_size_x, world_size_y, world_size_z,
            grid_res,
            world_origin_x, world_origin_y, world_origin_z,
            max_distance_m,
            ref_count);
    grid.setReferenceFrame(planning_frame);
    return grid;
}

int main(int argc, char* argv[]) {
    // Setting  up environment.
    std::string ns = "test_generator";
    ros::init(argc, argv, ns);
    ros::NodeHandle nh;

    sbpl::VisualizerROS visualizer (nh, 100);
    sbpl::viz::set_visualizer(&visualizer);

    ros::Publisher ma_pub = nh.advertise <visualization_msgs::MarkerArray> (
            "visualization_markers", 100 );
    std::string planning_frame = "/map";

    sbpl::OccupancyGrid grid = setupOccupancyGrid(planning_frame);
    ROS_INFO("Occupancy grid setup.");

    GridCollisionChecker2D cc(&grid);

    // Wait required for the occupancy grid to be set up.
    ros::Duration(10).sleep();

    // Start generating random shapes.

    int minNumSides = 6;
    int maxNumSides = 9;
    double minRadius = .5;
    double maxRadius = 1.0;
    RandomShapeGenerator shapeGen( &cc, minNumSides, maxNumSides, minRadius, maxRadius );

    int numShapes = 5;

    std::vector<Shape> shapes = shapeGen.generateRandomShapes(numShapes, grid);

    Vector3d origin = { 2, 2, 0 };
    for( auto shape : shapes ) {
        Shape inflatedShape = shapeGen.inflateShape( shape, grid, 1 );
        std::vector< Vector3d > points = inflatedShape.getBoundary();
        ROS_INFO("Viz");
        for (int i=0; i<points.size(); i++) {
            points[i][0] += origin[0];
            points[i][1] += origin[1];
        }
        grid.addPointsToField( points );
        ma_pub.publish( grid.getOccupiedVoxelsVisualization() );
        ros::Duration(1.).sleep();
        //grid.reset();
    }

    return 0;
}
