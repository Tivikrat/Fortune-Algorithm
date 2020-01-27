#pragma once

#include <array>
#include <limits>

#include "Vector2D.h"

class Frame {
public:
    enum class Side {
        LEFT, BOTTOM, RIGHT, TOP
    };

    struct Intersection {
        Side side;
        Vector2D point;
    };

    double left;
    double bottom;
    double right;
    double top;

    Frame::Intersection getFirstIntersection(const Vector2D &origin,
                                             const Vector2D &direction) const {
        Intersection intersection;
        double t = std::numeric_limits<double>::infinity();
        if (direction.x > 0.0) {
            t = (right - origin.x) / direction.x;
            intersection.side = Side::RIGHT;
            intersection.point = origin + t * direction;
        } else if (direction.x < 0.0) {
            t = (left - origin.x) / direction.x;
            intersection.side = Side::LEFT;
            intersection.point = origin + t * direction;
        }
        if (direction.y > 0.0) {
            double newT = (top - origin.y) / direction.y;
            if (newT < t) {
                intersection.side = Side::TOP;
                intersection.point = origin + newT * direction;
            }
        } else if (direction.y < 0.0) {
            double newT = (bottom - origin.y) / direction.y;
            if (newT < t) {
                intersection.side = Side::BOTTOM;
                intersection.point = origin + newT * direction;
            }
        }
        return intersection;
    }
};
