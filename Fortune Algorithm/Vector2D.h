#pragma once

#include <ostream>
#include <cmath>

class Vector2D {
public:
    double x;
    double y;

    explicit Vector2D(double x = 0.0, double y = 0.0) : x(x), y(y) {}

    Vector2D operator-() const {
        return Vector2D(-x, -y);
    }

    Vector2D operator+(Vector2D right) const {
        return Vector2D(x + right.x, y + right.y);
    }

    Vector2D operator-(const Vector2D &right) const {
        return Vector2D(x - right.x, y - right.y);
    }

    Vector2D operator*(const double value) const {
        return Vector2D(x * value, y * value);
    }

    Vector2D getOrthogonal() const {
        return Vector2D(-y, x);
    }

    double getLength() const {
        return std::sqrt(x * x + y * y);
    }

    double getDistance(const Vector2D &other) const {
        return (*this - other).getLength();
    }

    double getDeterminate(const Vector2D &other) const {
        return x * other.y - y * other.x;
    }
};

Vector2D operator*(const double value, const Vector2D vector2) {
    return vector2 * value;
}

std::ostream &operator<<(std::ostream &os, const Vector2D &vec) {
    os << "(" << vec.x << ", " << vec.y << ")";
    return os;
}
