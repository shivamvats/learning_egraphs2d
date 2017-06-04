#ifndef SHAPE_H
#define SHAPE_H

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <vector>

#include <smpl/occupancy_grid.h>

using namespace Eigen;

// It represents an arbitrary shape by means of storing the points in it.
// Note that the points are in local frame, i.e, to put the points in a map,
// you need to transform the origin to the global frame.
class Shape {
public:
    Shape(std::vector<Vector3d> boundary);

    std::vector<Vector3d> getBoundary() { return m_boundary; }
    std::vector<Vector3d> getAllPoints(sbpl::OccupancyGrid &grid);
    void translateToPoint(Vector3d origin);
    void setCircumRadius(double radius) { m_circumRadius = radius; }
    double getCircumRadius() { return m_circumRadius; }
    void setCircumCenter(Vector3d center) { m_circumCenter = center; }
    Vector3d getCircumCenter() { return m_circumCenter; }
private:
    // Given a boundary, fill the shape with points.
    void fillShape();

    std::vector<Vector3d> m_boundary;
    std::vector<Vector3d> m_points;
    double m_circumRadius;
    Vector3d m_circumCenter;
};

#endif
