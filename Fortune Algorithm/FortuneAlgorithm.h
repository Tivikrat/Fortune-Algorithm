#pragma once

#include <list>
#include <unordered_map>

#include "PriorityQueue.h"
#include "VoronoiDiagram.h"
#include "BeachLine.h"
#include "ArcNode.h"
#include "Event.h"

class ArcNode;

class Event;

class FortuneAlgorithm {
public:

    explicit FortuneAlgorithm(const std::vector<Vector2D> &points) : diagram(
            points) {}

    void construct() {
        for (std::size_t i = 0; i < diagram.getSitesCount(); ++i)
            events.push(std::make_unique<Event>(diagram.getSite(i)));

        while (!events.empty()) {
            std::unique_ptr<Event> event = events.pop();
            beachLineY = event->y;
            if (event->type == Event::Type::SITE)
                handleSiteEvent(event.get());
            else
                handleCircleEvent(event.get());
        }
    }

    void bound(Frame box) {
        for (const auto &vertex : diagram.getVertices()) {
            box.left = std::min(vertex.point.x, box.left);
            box.bottom = std::min(vertex.point.y, box.bottom);
            box.right = std::max(vertex.point.x, box.right);
            box.top = std::max(vertex.point.y, box.top);
        }
        std::list<LinkedVertex> linkedVertices;
        std::unordered_map<std::size_t, std::array<LinkedVertex *, 8>> vertices(
                diagram.getSitesCount());
        if (!beachLine.isEmpty()) {
            ArcNode *leftArc = beachLine.getLeftmostArc();
            ArcNode *rightArc = leftArc->next;
            while (!beachLine.isNil(rightArc)) {
                Vector2D direction = (leftArc->site->point -
                                      rightArc->site->point).getOrthogonal();
                Vector2D origin =
                        (leftArc->site->point + rightArc->site->point) * 0.5f;
                Frame::Intersection intersection = box.getFirstIntersection(
                        origin, direction);
                VoronoiDiagram::Vertex *vertex = diagram.createVertex(
                        intersection.point);
                setDestination(leftArc, rightArc, vertex);
                if (vertices.find(leftArc->site->index) == vertices.end())
                    vertices[leftArc->site->index].fill(nullptr);
                if (vertices.find(rightArc->site->index) == vertices.end())
                    vertices[rightArc->site->index].fill(nullptr);
                linkedVertices.emplace_back(
                        LinkedVertex{nullptr, vertex, leftArc->rightHalfEdge});
                vertices[leftArc->site->index][
                        2 * static_cast<int>(intersection.side) +
                        1] = &linkedVertices.back();
                linkedVertices.emplace_back(
                        LinkedVertex{rightArc->leftHalfEdge, vertex, nullptr});
                vertices[rightArc->site->index][2 *
                                                static_cast<int>(intersection.side)] = &linkedVertices.back();
                leftArc = rightArc;
                rightArc = rightArc->next;
            }
        }
        for (auto &pair : vertices) {
            auto &cellVertices = pair.second;
            for (std::size_t i = 0; i < 5; ++i) {
                std::size_t side = i % 4;
                std::size_t nextSide = (side + 1) % 4;
                if (cellVertices[2 * side] == nullptr &&
                    cellVertices[2 * side + 1] != nullptr) {
                    std::size_t previousSide = (side + 3) % 4;
                    VoronoiDiagram::Vertex *corner = diagram.createCorner(box,
                                                                          static_cast<Frame::Side>(side));
                    linkedVertices.emplace_back(
                            LinkedVertex{nullptr, corner, nullptr});
                    cellVertices[2 * previousSide + 1] = &linkedVertices.back();
                    cellVertices[2 * side] = &linkedVertices.back();
                } else if (cellVertices[2 * side] != nullptr &&
                           cellVertices[2 * side + 1] == nullptr) {
                    VoronoiDiagram::Vertex *corner = diagram.createCorner(box,
                                                                          static_cast<Frame::Side>(nextSide));
                    linkedVertices.emplace_back(
                            LinkedVertex{nullptr, corner, nullptr});
                    cellVertices[2 * side + 1] = &linkedVertices.back();
                    cellVertices[2 * nextSide] = &linkedVertices.back();
                }
            }
        }
        for (auto &pair : vertices) {
            std::size_t i = pair.first;
            auto &cellVertices = pair.second;
            for (std::size_t side = 0; side < 4; ++side) {
                if (cellVertices[2 * side] != nullptr) {
                    VoronoiDiagram::HalfEdge *halfEdge = diagram.createHalfEdge(
                            diagram.getCell(i));
                    halfEdge->origin = cellVertices[2 * side]->vertex;
                    halfEdge->destination = cellVertices[2 * side + 1]->vertex;
                    cellVertices[2 * side]->nextHalfEdge = halfEdge;
                    halfEdge->previous = cellVertices[2 * side]->prevHalfEdge;
                    if (cellVertices[2 * side]->prevHalfEdge != nullptr)
                        cellVertices[2 * side]->prevHalfEdge->next = halfEdge;
                    cellVertices[2 * side + 1]->prevHalfEdge = halfEdge;
                    halfEdge->next = cellVertices[2 * side + 1]->nextHalfEdge;
                    if (cellVertices[2 * side + 1]->nextHalfEdge != nullptr)
                        cellVertices[2 * side +
                                     1]->nextHalfEdge->previous = halfEdge;
                }
            }
        }
    }

    VoronoiDiagram getDiagram() {
        return std::move(diagram);
    }

private:
    VoronoiDiagram diagram;
    BeachLine beachLine;
    PriorityQueue<Event> events;
    double beachLineY;

    void handleSiteEvent(Event *event) {
        VoronoiDiagram::Site *site = event->site;
        if (beachLine.isEmpty()) {
            beachLine.setRoot(beachLine.createArc(site));
            return;
        }
        ArcNode *arcToBreak = beachLine.locateArcAbove(site->point,
                                                       beachLineY);
        deleteEvent(arcToBreak);
        ArcNode *middleArc = breakArc(arcToBreak, site);
        ArcNode *leftArc = middleArc->previous;
        ArcNode *rightArc = middleArc->next;
        addEdge(leftArc, middleArc);
        middleArc->rightHalfEdge = middleArc->leftHalfEdge;
        rightArc->leftHalfEdge = leftArc->rightHalfEdge;
        if (!beachLine.isNil(leftArc->previous))
            addEvent(leftArc->previous, leftArc, middleArc);
        if (!beachLine.isNil(rightArc->next))
            addEvent(middleArc, rightArc, rightArc->next);
    }

    void handleCircleEvent(Event *event) {
        Vector2D point = event->point;
        ArcNode *arc = event->arc;
        VoronoiDiagram::Vertex *vertex = diagram.createVertex(point);
        ArcNode *leftArc = arc->previous;
        ArcNode *rightArc = arc->next;
        deleteEvent(leftArc);
        deleteEvent(rightArc);
        removeArc(arc, vertex);
        if (!beachLine.isNil(leftArc->previous))
            addEvent(leftArc->previous, leftArc, rightArc);
        if (!beachLine.isNil(rightArc->next))
            addEvent(leftArc, rightArc, rightArc->next);
    }

    ArcNode *breakArc(ArcNode *arc, VoronoiDiagram::Site *site) {
        ArcNode *middleArc = beachLine.createArc(site);
        ArcNode *leftArc = beachLine.createArc(arc->site);
        leftArc->leftHalfEdge = arc->leftHalfEdge;
        ArcNode *rightArc = beachLine.createArc(arc->site);
        rightArc->rightHalfEdge = arc->rightHalfEdge;
        beachLine.replace(arc, middleArc);
        beachLine.insertBefore(middleArc, leftArc);
        beachLine.insertAfter(middleArc, rightArc);
        delete arc;
        return middleArc;
    }

    void removeArc(ArcNode *arc, VoronoiDiagram::Vertex *vertex) {
        setDestination(arc->previous, arc, vertex);
        setDestination(arc, arc->next, vertex);
        arc->leftHalfEdge->next = arc->rightHalfEdge;
        arc->rightHalfEdge->previous = arc->leftHalfEdge;
        beachLine.remove(arc);
        VoronoiDiagram::HalfEdge *prevHalfEdge = arc->previous->rightHalfEdge;
        VoronoiDiagram::HalfEdge *nextHalfEdge = arc->next->leftHalfEdge;
        addEdge(arc->previous, arc->next);
        setOrigin(arc->previous, arc->next, vertex);
        setPrevHalfEdge(arc->previous->rightHalfEdge, prevHalfEdge);
        setPrevHalfEdge(nextHalfEdge, arc->next->leftHalfEdge);
        delete arc;
    }

    bool isMovingRight(const ArcNode *left, const ArcNode *right) const {
        return left->site->point.y < right->site->point.y;
    }

    double getInitialX(const ArcNode *left, const ArcNode *right,
                       bool movingRight) const {
        return movingRight ? left->site->point.x : right->site->point.x;
    }

    void addEdge(ArcNode *left, ArcNode *right) {
        left->rightHalfEdge = diagram.createHalfEdge(left->site->cell);
        right->leftHalfEdge = diagram.createHalfEdge(right->site->cell);
        left->rightHalfEdge->twin = right->leftHalfEdge;
        right->leftHalfEdge->twin = left->rightHalfEdge;
    }

    static void
    setOrigin(ArcNode *left, ArcNode *right, VoronoiDiagram::Vertex *vertex) {
        left->rightHalfEdge->destination = vertex;
        right->leftHalfEdge->origin = vertex;
    }

    static void setDestination(ArcNode *left, ArcNode *right,
                               VoronoiDiagram::Vertex *vertex) {
        left->rightHalfEdge->origin = vertex;
        right->leftHalfEdge->destination = vertex;
    }

    static void setPrevHalfEdge(VoronoiDiagram::HalfEdge *previous,
                                VoronoiDiagram::HalfEdge *next) {
        previous->next = next;
        next->previous = previous;
    }

    void addEvent(ArcNode *left, ArcNode *middle, ArcNode *right) {
        double y;
        Vector2D convergencePoint = getConvergencePoint(left->site->point,
                                                        middle->site->point,
                                                        right->site->point,
                                                        y);
        bool isBelow = y <= beachLineY;
        bool leftBreakpointMovingRight = isMovingRight(left, middle);
        bool rightBreakpointMovingRight = isMovingRight(middle, right);
        double leftInitialX = getInitialX(left, middle,
                                          leftBreakpointMovingRight);
        double rightInitialX = getInitialX(middle, right,
                                           rightBreakpointMovingRight);
        bool isValid =
                ((leftBreakpointMovingRight &&
                  leftInitialX < convergencePoint.x) ||
                 (!leftBreakpointMovingRight &&
                  leftInitialX > convergencePoint.x)) &&
                ((rightBreakpointMovingRight &&
                  rightInitialX < convergencePoint.x) ||
                 (!rightBreakpointMovingRight &&
                  rightInitialX > convergencePoint.x));
        if (isValid && isBelow) {
            std::unique_ptr<Event> event = std::make_unique<Event>(y,
                                                                   convergencePoint,
                                                                   middle);
            middle->event = event.get();
            events.push(std::move(event));
        }
    }

    void deleteEvent(ArcNode *arc) {
        if (arc->event != nullptr) {
            events.remove(arc->event->index);
            arc->event = nullptr;
        }
    }

    Vector2D
    getConvergencePoint(const Vector2D &point1, const Vector2D &point2,
                        const Vector2D &point3, double &y) const {
        Vector2D v1 = (point1 - point2).getOrthogonal();
        Vector2D v2 = (point2 - point3).getOrthogonal();
        Vector2D delta = 0.5 * (point3 - point1);
        double t = delta.getDeterminate(v2) / v1.getDeterminate(v2);
        Vector2D center = 0.5 * (point1 + point2) + t * v1;
        double r = center.getDistance(point1);
        y = center.y - r;
        return center;
    }

    struct LinkedVertex {
        VoronoiDiagram::HalfEdge *prevHalfEdge;
        VoronoiDiagram::Vertex *vertex;
        VoronoiDiagram::HalfEdge *nextHalfEdge;
    };
};
