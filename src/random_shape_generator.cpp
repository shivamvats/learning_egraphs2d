#include "random_shape_generator.h"
#include <algorithm>
#include <stdlib.h>
//#include <smpl/types.h>

#define PI 3.14

RandomShapeGenerator::RandomShapeGenerator(GridCollisionChecker2D *cc,
                                           int minNumSides, int maxNumSides,
                                           double minRadius, double maxRadius)
    : m_cc(cc) {
    m_maxNumSides = maxNumSides;
    m_minNumSides = minNumSides;
    m_maxRadius = maxRadius;
    m_minRadius = minRadius;
}

int getRandomInt(int min, int max) {
    return min + static_cast<int>(static_cast<float>(rand()) /
                                  (static_cast<float>(RAND_MAX / (max - min))));
}

float getRandomFloat(float min, float max) {
    float random = rand();
    return min +
           static_cast<float>(random) /
               (static_cast<float>(RAND_MAX / (max - min)));
}

smpl::RobotState toRobotState(Vector3d vector) {
    smpl::RobotState state = {vector[0], vector[1], vector[2]};
    return state;
}

std::vector<std::vector<int>> toGrid(std::vector<Vector3d> points,
                                     sbpl::OccupancyGrid &grid) {
    std::vector<std::vector<int>> gridPoints;
    for (auto point : points) {
        int x, y, z;
        grid.worldToGrid(point[0], point[1], point[2], x, y, z);
        gridPoints.push_back(std::vector<int>{x, y, z});
    }
    return gridPoints;
}

std::vector<Vector3d> toWorld(std::vector<std::vector<int>> points,
                              sbpl::OccupancyGrid &grid) {
    std::vector<Vector3d> worldPoints;
    for (auto point : points) {
        double x, y, z;
        grid.gridToWorld(point[0], point[1], point[2], x, y, z);
        worldPoints.push_back(Vector3d(x, y, z));
    }
    return worldPoints;
}

std::vector<Shape>
RandomShapeGenerator::generateRandomShapes(int num, sbpl::OccupancyGrid &grid,
                                           int seed) {
    srand(seed);
    std::vector<Shape> shapes;

    for (int i = 0; i < num; i++) {
        std::vector<Vector3d> points;

        double radius = getRandomFloat(m_minRadius, m_maxRadius);
        int n_sides = getRandomInt(m_minNumSides, m_maxNumSides);

        ROS_INFO("Radius: %f, nSides: %d", radius, n_sides);

        std::vector<Vector3d> vertices;
        std::vector<double> thetas;

        // Sample n_sides angles on the circle and sort them.
        for (int j = 0; j < n_sides; j++)
            thetas.push_back((static_cast<float>(rand()) / RAND_MAX) * 2 * PI);
        std::sort(thetas.begin(), thetas.end());
        // thetas = {.5, 1, 1.5, 2, 2.5};

        // Get vertices of the polygon.
        for (int j = 0; j < n_sides; j++) {
            // Sample n_sides points on a circle of radius 'radius'.
            double theta = thetas[j];
            ROS_INFO("%f", theta);
            vertices.push_back(
                Vector3d{radius * cos(theta), radius * sin(theta), 0});
            ROS_INFO("%f, %f", vertices[j][0], vertices[j][1]);
        }

        // Now, need to interpolate between the vertices to form the boundary.
        for (int j = 0; j < n_sides; j++) {
            std::vector<Vector3d> edge;
            std::vector<smpl::RobotState> edge_;

            ROS_INFO("interpolate called");
            m_cc->interpolatePath(toRobotState(vertices[j]),
                                  toRobotState(vertices[(j + 1) % n_sides]),
                                  edge_);

            for (auto ele : edge_) {
                edge.push_back(Vector3d(ele[0], ele[1], 0));
                // ROS_INFO("%f, %f, %f", ele[0], ele[1], 0);
            }

            points.insert(points.end(), edge.begin(), edge.end());
            // shapes.push_back( Shape(edge) );
        }
        shapes.push_back(Shape(points));
    }
    return shapes;
}

// Instead of checking if a pixel has already been added or not, maintain a set
// of pixels.
std::vector<std::vector<int>> getInflatedWindow(std::vector<int> point,
                                                int inflationRadius = 1) {
    std::vector<std::vector<int>> neighbours;

    for (int i = point[0] - inflationRadius; i <= point[0] + inflationRadius;
         i++) {
        for (int j = point[1] - inflationRadius;
             j <= point[1] + inflationRadius; j++) {
            // if (i == j )
            // continue;
            std::vector<int> currPoint = {i, j, point[2]};
            // Check if it is already part of the shape.
            // if ( find( points.begin(), points.end(), currPoint ) !=
            // points.end() )
            neighbours.push_back(currPoint);
        }
    }
    return neighbours;
}

// Inflate the shape by a certain radius r. This essentially make pixels in an
// r-neighbourhood of each pixel a part of the shape.
Shape RandomShapeGenerator::inflateShape(Shape shape, sbpl::OccupancyGrid &grid,
                                         int inflationRadius = 1) {
    std::vector<std::vector<int>> boundaryGridPoints =
        toGrid(shape.getBoundary(), grid);
    // Set insertions is log n.
    std::set<std::vector<int>> inflatedGridPoints;

    for (auto point : boundaryGridPoints) {
        std::vector<std::vector<int>> neighbours =
            getInflatedWindow(point, inflationRadius);
        for (auto ele : neighbours)
            inflatedGridPoints.insert(ele);
    }
    std::vector<std::vector<int>> inflatedGridPointsV(
        inflatedGridPoints.begin(), inflatedGridPoints.end());
    std::vector<Vector3d> inflatedWorldPoints;
    inflatedWorldPoints = toWorld(inflatedGridPointsV, grid);

    return Shape(inflatedWorldPoints);
}
