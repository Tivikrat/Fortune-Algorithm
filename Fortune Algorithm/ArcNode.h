#pragma once

#include "VoronoiDiagram.h"

class Event;

struct ArcNode {
    enum class Color {
        RED, BLACK
    };

    ArcNode *parent;
    ArcNode *left;
    ArcNode *right;
    VoronoiDiagram::Site *site;
    VoronoiDiagram::HalfEdge *leftHalfEdge;
    VoronoiDiagram::HalfEdge *rightHalfEdge;
    Event *event;
    ArcNode *previous;
    ArcNode *next;
    Color color;
};
