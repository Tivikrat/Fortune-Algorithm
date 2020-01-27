#pragma once

#include "Vector2D.h"
#include "VoronoiDiagram.h"

class ArcNode;

class Event {
public:
    enum class Type {
        SITE, CIRCLE
    };

    Event(VoronoiDiagram::Site *site) : type(Type::SITE), y(site->point.y),
                                        site(site) {}

    Event(double y, Vector2D point, ArcNode *arc) : type(Type::CIRCLE), y(y),
                                                    point(point), arc(arc) {}

    const Type type;
    double y;
    int index = -1;
    VoronoiDiagram::Site *site;
    Vector2D point;
    ArcNode *arc;

    bool operator<(const Event &right_event) {
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
