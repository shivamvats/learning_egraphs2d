#include "grid_collision_checker_2d.h"
#include "shape.h"

// Currently only for 2D shapes.
class RandomShapeGenerator {
public:
    RandomShapeGenerator(GridCollisionChecker2D *cc, int minNumSides,
                         int maxNumSides, double minRadius, double maxRadius);
    std::vector<Shape> generateRandomShapes(int num, sbpl::OccupancyGrid &grid,
                                            int seed = 23425);
    Shape inflateShape(Shape, sbpl::OccupancyGrid &, int inflationRadius);

private:
    int m_maxNumSides;
    int m_minNumSides;
    double m_minRadius;
    double m_maxRadius;
    GridCollisionChecker2D *m_cc;
};
