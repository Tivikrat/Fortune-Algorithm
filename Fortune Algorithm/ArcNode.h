#pragma once

#include "VoronoiDiagram.h"

class Event;

struct ArcNode {
    enum class Color {
        RED, BLACK
    };

    ArcNode *parent = nullptr;
    ArcNode *left = nullptr;
    ArcNode *right = nullptr;
    VoronoiDiagram::Site *site = nullptr;
    VoronoiDiagram::HalfEdge *leftHalfEdge = nullptr;
    VoronoiDiagram::HalfEdge *rightHalfEdge = nullptr;
    Event *event = nullptr;
    ArcNode *previous = nullptr;
    ArcNode *next = nullptr;
    Color color;
};
