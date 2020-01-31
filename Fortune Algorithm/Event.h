#pragma once

#include "Vector2D.h"
#include "VoronoiDiagram.h"

class ArcNode;

class Event {
public:
    enum class Type {
        SITE, CIRCLE
    };

    Event(VoronoiDiagram::Site *site) : type(Type::SITE), x(site->point.x), y(site->point.y), site(site) {}

    Event(double y, Vector2D point, ArcNode *arc) : type(Type::CIRCLE), x(point.x), y(y), point(point), arc(arc) {}

    const Type type;
    double x;
    double y;
    int index = -1;
    VoronoiDiagram::Site *site;
    Vector2D point;
    ArcNode *arc;

    bool operator<(const Event &right_event) {
        if (y == right_event.y) {
            return x < right_event.x;
        }
        return y < right_event.y;
    }
};

std::ostream &operator<<(std::ostream &output_stream, const Event &event) {
    if (event.type == Event::Type::SITE)
        output_stream << "S(" << event.site->index << ", " << event.y << ")";
    else
        output_stream << "C(" << event.arc << ", " << event.y << ", "
                      << event.point << ")";
    return output_stream;
}
