#include <chrono>
#include <cmath>
#include <algorithm>

#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/PlanningScene.h>
#include <ros/ros.h>

#include <smpl/graph/manip_lattice_action_space.h>
#include <smpl/graph/manip_lattice_egraph.h>
#include <smpl/heuristic/egraph_bfs_heuristic.h>
#include <smpl/heuristic/euclid_dist_heuristic.h>
#include <smpl/heuristic/generic_egraph_heuristic.h>
#include <smpl/heuristic/joint_dist_heuristic.h>
#include <smpl/occupancy_grid.h>
#include <smpl/robot_model.h>
#include <smpl/search/experience_graph_planner.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <smpl/debug/visualizer_ros.h>

#include "grid_collision_checker_2d.h"
#include "kinematic_vehicle_model_2d.h"
#include "utils.h"
#include "random_shape_generator.h"
#include "start_goal_generator.h"

namespace smpl = sbpl::motion;

std::vector<Eigen::Vector3d>
generateRectangle(sbpl::OccupancyGrid &grid, const Eigen::Vector3d leftTop,
                  const Eigen::Vector3d rightBottom) {
    const int start_x = leftTop.x();
    const int start_y = rightBottom.y();
    const int end_x = rightBottom.x();
    const int end_y = leftTop.y();

    std::vector<Eigen::Vector3d> points;

    ROS_INFO("start: %d, %d \n end: %d, %d", start_x, start_y, end_x, end_y);

    for (int gx = start_x; gx <= end_x; gx++) {
        for (int gy = start_y; gy <= end_y; gy++) {
            double cx, cy, cz;
            grid.gridToWorld(gx, gy, 0, cx, cy, cz);
            ROS_ERROR("%f, %df, %f", cx, cy, cz);
            points.emplace_back(cx, cy, cz);
        }
    }
    return points;
}

/// Add a simple box obstacle in the center of the grid
void SetupOccupancyGrid(sbpl::OccupancyGrid &grid) {
    // Random2DShapeGenerator rShapeGen(0.02, 20, 20);

    const int x_count = grid.numCellsX();
    const int y_count = grid.numCellsY();
    const int z_count = grid.numCellsZ();

    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3d> points2D;

    const int halfLength = 10;
    const int halfBreadth = 5;
    Eigen::Vector3d leftTop = {(x_count >> 1) - halfLength,
                               (y_count >> 1) + halfBreadth, 0};
    Eigen::Vector3d rightBottom = {(x_count >> 1) + halfLength,
                                   (y_count >> 1) - halfBreadth, 0};
    points = generateRectangle(grid, leftTop, rightBottom);
    // points2D = rShapeGen.generateShape(4, 100);
    // for(int i=0;i<Points2D.size();i++) {
    //    points.push_back(Eigen::Vector3d point(points2D[i].x(),
    //    points2D[i].y(),
    //    0));
    //}

    ROS_INFO("Add %zu points to grid", points.size());
    grid.addPointsToField(points);
}

Shape addRandomObstacle(sbpl::OccupancyGrid &grid, GridCollisionChecker2D &cc) {
    double gridSize = std::min(grid.sizeX(), grid.sizeY());

    int minNumSides = 8;
    int maxNumSides = 12;
    double minRadius = gridSize/4;
    double maxRadius = gridSize/3;
    RandomShapeGenerator shapeGen(&cc, minNumSides, maxNumSides, minRadius,
                                  maxRadius);

    int numShapes = 1;
    std::vector<Shape> shapes = shapeGen.generateRandomShapes(numShapes, grid);

    // Keep the obstacle at the center of the map.
    Vector3d origin = {grid.sizeX() / 2, grid.sizeY() / 2, 0};
    Shape shape = shapes[0];

    Shape inflatedShape = shapeGen.inflateShape(shape, grid, 1);
    std::vector<Vector3d> points = inflatedShape.getBoundary();

    // Translating the shape to the origin.
    shape.translateToPoint(origin);
    //XXX Introduce rotation?

    grid.addPointsToField(points);
    // grid.reset();
    return shape;
}

sbpl::OccupancyGrid setupOccupancyGrid(std::string planning_frame,
                                       const double world_size_x = 5,
                                       const double world_size_y = 5) {
    const double grid_res = 0.02;
    const double world_size_z = 1.5 * grid_res;
    const double world_origin_x = 0.0;
    const double world_origin_y = 0.0;
    const double world_origin_z = 0.0;
    const double max_distance_m = 0.2;
    const bool ref_count = false;
    ROS_ERROR("Setting up occupancy grid of size %f x %f and origin %f, %f",
              world_size_x, world_size_y, world_origin_x, world_origin_y);
    sbpl::OccupancyGrid grid(world_size_x, world_size_y, world_size_z, grid_res,
                             world_origin_x, world_origin_y, world_origin_z,
                             max_distance_m, ref_count);
    grid.setReferenceFrame(planning_frame);
    return grid;
}

int main(int argc, char *argv[]) {
    std::string ns = "xytheta";
    ros::init(argc, argv, ns);
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    sbpl::VisualizerROS visualizer(nh, 100);
    sbpl::viz::set_visualizer(&visualizer);

    ros::Publisher ma_pub = nh.advertise<visualization_msgs::MarkerArray>(
        "visualization_markers", 100);
    ros::Publisher m_pub =
        nh.advertise<visualization_msgs::Marker>("visualization_marker", 100);

    // let publishers set up
    ros::Duration(1.0).sleep();

    // everyone needs to know the name of the planning frame for reasons...
    std::string planning_frame;
    if (!ph.getParam("planning_frame", planning_frame)) {
        ROS_ERROR(
            "Failed to retrieve param 'planning_frame' from the param server");
        return 1;
    }

    ROS_INFO("Planning Frame: %s", planning_frame.c_str());

    // 1. Instantiate Robot Model
    KinematicVehicleModel2D robot_model;


    // 2. Instantiate the Environment
    double world_size_x = 5.0;
    double world_size_y = 5.0;
    sbpl::OccupancyGrid grid =
        setupOccupancyGrid(planning_frame, world_size_x, world_size_y);
    ROS_INFO("Occupancy grid set up.");

    GridCollisionChecker2D cc(&grid);

    // Add a random obstacle to the grid.
    Shape shape = addRandomObstacle(grid, cc);

    // Visualize the obstacles and boundary.
    ma_pub.publish(grid.getOccupiedVoxelsVisualization());
    ma_pub.publish(grid.getBoundingBoxVisualization());


    // 3. Define Parameters
    smpl::PlanningParams params;

    // 4. Instantiate Planning Space
    auto pspace =
        std::make_shared<smpl::ManipLatticeEgraph>(&robot_model, &cc, &params);
    if (!pspace->init({0.02, 0.02})) {
        ROS_ERROR("Failed to initialize Manip Lattice");
        return 1;
    }
    pspace->setVisualizationFrameId("map");

    std::string egraph_file;
    if (ph.getParam("egraph_file", egraph_file)) {
        ROS_INFO("Received egraph file path: %s. Loading...",
                 egraph_file.c_str());
        if (!pspace->loadExperienceGraph(egraph_file))
            ROS_ERROR("Failed to load experience graph");
    }

    // 5. Instantiate and Initialize Motion Primitives
    std::string mprim_path;
    if (!ph.getParam("mprim_path", mprim_path)) {
        ROS_ERROR("Failed to retrieve 'mprim_path' from the param server");
        return 1;
    }

    auto aspace = std::make_shared<smpl::ManipLatticeActionSpace>(pspace.get());
    if (!aspace->load(mprim_path)) {
        return 1;
    }
    PrintActionSpace(*aspace);

    // 6. Associate Action Space with Planning Space
    pspace->setActionSpace(aspace);

    // 7. Instantiate Heuristic
    auto h = std::make_shared<smpl::JointDistHeuristic>(pspace, &grid);
    auto hEgraph =
        std::make_shared<smpl::GenericEgraphHeuristic>(pspace, &grid, h);

    // 8. Associate Heuristic with Planning Space (for adaptive motion
    // primitives)
    pspace->insertHeuristic(hEgraph.get());

    // 9. Instantiate and Initialize Search (associated with Planning Space)
    const bool forward = true;
    auto search =
        std::make_shared<smpl::ExperienceGraphPlanner>(pspace, hEgraph);

    const double epsilon = 5.0;
    search->set_initialsolution_eps(epsilon);
    search->set_search_mode(false);

    // 10. Set start state and goal condition in the Planning Space and
    // propagate state IDs to search
    //double start_x = 0.1;
    //double start_y = 0.23;
    //double start_z = 0;
    //double goal_x = 0.9;
    //double goal_y = 0.96;
    //double goal_z = 0;
    smpl::RobotState start_state, goal_state;
    StartGoalGenerator startGoalgenerator;
    double startRadius = 1.2 * shape.getCircumRadius();
    double goalRadius = 1.2 * shape.getCircumRadius();

    startGoalgenerator.generateStartGoalOnCircle(start_state, goal_state,
                                                 Vector3d(0, 0, 0), startRadius);


    moveit_msgs::RobotState start_state_msg;
    start_state_msg.joint_state.name.push_back("x");
    start_state_msg.joint_state.name.push_back("y");
    start_state_msg.joint_state.position.push_back(start_state[0]);
    start_state_msg.joint_state.position.push_back(start_state[1]);

    smpl::GoalConstraint goal;
    goal.type = smpl::GoalType::JOINT_STATE_GOAL;
    goal.angles = goal_state;
    goal.angle_tolerances = {0.02, 0.02};

    // setStartGoal(pspace, start_state, goal);
    if (!pspace->setGoal(goal)) {
        ROS_ERROR("Failed to set goal");
        return 1;
    }

    if (!pspace->setStart(start_state)) {
        ROS_ERROR("Failed to set start");
        return 1;
    }

    int start_id = pspace->getStartStateID();
    if (start_id < 0) {
        ROS_ERROR("Start state id is invalid");
        return 1;
    }

    int goal_id = pspace->getGoalStateID();
    if (goal_id < 0) {
        ROS_ERROR("Goal state id is invalid");
        return 1;
    }

    if (search->set_start(start_id) == 0) {
        ROS_ERROR("Failed to set planner start state");
        return 1;
    }

    if (search->set_goal(goal_id) == 0) {
        ROS_ERROR("Failed to set planner goal state");
        return 1;
    }

    // 11. Plan a path
    ReplanParams search_params(10.0);
    search_params.initial_eps = epsilon;
    search_params.final_eps = 1.0;
    search_params.dec_eps = 0.2;
    search_params.return_first_solution = false;
    search_params.repair_time = 1.0;

    auto then = std::chrono::high_resolution_clock::now();
    std::vector<int> solution;
    int solcost;
    bool bret = search->replan(&solution, search_params, &solcost);
    if (!bret) {
        ROS_ERROR("Search failed to find a solution");
        return 1;
    }
    auto now = std::chrono::high_resolution_clock::now();
    const double elapsed = std::chrono::duration<double>(now - then).count();
    ROS_ERROR("Yipeee. got till here");

    // 12. Extract path from Planning Space
    std::vector<smpl::RobotState> path;
    if (!pspace->extractPath(solution, path)) {
        ROS_ERROR("Failed to extract path");
    }

    // Visualize the path.
    std::vector<geometry_msgs::Point> pathPoints;
    for (auto node : path) {
        geometry_msgs::Point point;
        point.x = node[0];
        point.y = node[1];
        pathPoints.push_back(point);
        // ROS_INFO("%f %f", point.x, point.y);
    }
    m_pub.publish(getLineVisualization(pathPoints, planning_frame, ns));

    // Save the path in a file.
    std::string plan_output_dir = "/home/aries/.ros/plans";

    moveit_msgs::RobotTrajectory rTraj;
    convertJointVariablePathToJointTrajectory(path, rTraj.joint_trajectory,
                                              &robot_model, planning_frame);
    ROS_ERROR("Size= %d", rTraj.joint_trajectory.points.size());
    moveit_msgs::RobotState trajectory_start;
    writePath(start_state_msg, rTraj, &robot_model, plan_output_dir);

    ROS_INFO("Path found!");
    ROS_INFO("  Planning Time: %0.3f", elapsed);
    ROS_INFO("  Expansion Count (total): %d", search->get_n_expands());
    ROS_INFO("  Expansion Count (initial): %d",
             search->get_n_expands_init_solution());
    ROS_INFO("  Solution (%zu)", solution.size());
    for (int id : solution) {
        ROS_INFO("    %d", id);
    }
    ROS_INFO("  Path (%zu)", path.size());
    for (const smpl::RobotState &point : path) {
        ROS_INFO("    (x: %0.3f, y: %0.3f)", point[0], point[1]);
    }

    return 0;
}

/*
void setStartGoal(std::shared_ptr<smpl::ManipLatticeEgraph>& pspace,
                  smpl::RobotState start, smpl::RobotState goal) {
    if (!pspace->setGoal(goal)) {
        ROS_ERROR("Failed to set goal");
        return 1;
    }

    if (!pspace->setStart(start_state)) {
        ROS_ERROR("Failed to set start");
        return 1;
    }

    int start_id = pspace->getStartStateID();
    if (start_id < 0) {
        ROS_ERROR("Start state id is invalid");
        return 1;
    }

    int goal_id = pspace->getGoalStateID();
    if (goal_id < 0)  {
        ROS_ERROR("Goal state id is invalid");
        return 1;
    }

    if (search->set_start(start_id) == 0) {
        ROS_ERROR("Failed to set planner start state");
        return 1;
    }

    if (search->set_goal(goal_id) == 0) {
        ROS_ERROR("Failed to set planner goal state");
        return 1;
    }
}
*/
