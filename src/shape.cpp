#include "shape.h"
#include <queue>

Shape::Shape(std::vector<Vector3d> boundary) {
    m_boundary = boundary;
}

bool isOccupied(Vector3d point, sbpl::OccupancyGrid& grid) {
    double distance = grid.getDistance(point[0], point[1], point[2]);
    return !distance;
}

std::vector<Vector3d> getNeighbours(Vector3d point, sbpl::OccupancyGrid& grid) {
    std::vector<Vector3d> points;

    // 4 Connected.
    points.push_back(Vector3d (point.x()+1 , point.y(), point.z()));
    points.push_back(Vector3d (point.x(), point.y()+1, point.z()));
    points.push_back(Vector3d (point.x()-1, point.y(), point.z()));
    points.push_back(Vector3d (point.x(), point.y()-1, point.z()));

    for(auto it=begin(points);it!=end(points);it++) {
        if(isOccupied(*it, grid))
            points.erase(it);
    }
    return points;
}

// XXX Test this.
void actOnNode(Vector3d point, sbpl::OccupancyGrid& grid) {
    double distance = grid.getDistance(point.x(), point.y(), point.z());
    bool occupied;
    occupied = !distance;
    if(!occupied)
        grid.addPointsToField(std::vector<Vector3d> {point} );
}

std::vector<Vector3d> getInternalPoints(sbpl::OccupancyGrid& grid) {
    std::vector<Vector3d> points;
    int z;

    std::queue <Vector3d> q;
    q.push(Vector3d (0, 0, z));
    while(!q.empty()) {
        Vector3d top;
        top = q.front();
        q.pop();
        std::vector<Vector3d> neighbours;
        neighbours = getNeighbours(top, grid);

        for(auto neighbour : neighbours) {
            q.push(neighbour);
            actOnNode(neighbour, grid);
        }
    }
    grid.getOccupiedVoxels(points);
    return points;
}

std::vector<Vector3d> Shape::getAllPoints(sbpl::OccupancyGrid& grid) {
    std::vector<Vector3d> points;
    // Empty the occupancy grid.
    grid.getOccupiedVoxels(points);
    grid.reset();

    grid.addPointsToField(m_boundary);
    // Do BFS and fill the shape.

    m_points = getInternalPoints(grid);

    // Restore the occupancy grid.
    grid.reset();
    grid.addPointsToField(points);

    return m_points;
}
