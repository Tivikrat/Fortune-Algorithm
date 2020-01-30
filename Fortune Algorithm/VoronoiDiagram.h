#pragma once

#include <vector>
#include <list>
#include <unordered_set>

#include "Frame.h"

class FortuneAlgorithm;

class VoronoiDiagram {
public:
    struct HalfEdge;
    struct Cell;

    struct Site {
        std::size_t index;
        Vector2D point;
        Cell *cell;
    };

    struct HalfEdge {
        Vector2D *origin = nullptr;
        Vector2D *destination = nullptr;
        HalfEdge *twin = nullptr;
        Cell *incidentCell;
        HalfEdge *previous = nullptr;
        HalfEdge *next = nullptr;
    };

    struct Cell {
        Site *site;
        HalfEdge *outerComponent;
    };

    explicit VoronoiDiagram(const std::vector<Vector2D> &points) {
        sites.reserve(points.size());
        cells.reserve(points.size());
        for (std::size_t i = 0; i < points.size(); ++i) {
            sites.push_back(VoronoiDiagram::Site{i, points[i], nullptr});
            cells.push_back(VoronoiDiagram::Cell{&sites.back(), nullptr});
            sites.back().cell = &cells.back();
        }
    }

    VoronoiDiagram::Site *getSite(std::size_t index) {
        return &sites[index];
    }

    std::size_t getSitesCount() const {
        return sites.size();
    }

    VoronoiDiagram::Cell *getCell(std::size_t index) {
        return &cells[index];
    }

    std::vector<Cell> getCells() {
        return cells;
    }

    const std::list<Vector2D> &getVertices() const {
        return vertices;
    }

    const std::list<VoronoiDiagram::HalfEdge> &getHalfEdges() const {
        return half_edges;
    }

private:
    std::vector<Site> sites;
    std::vector<Cell> cells;
    std::list<Vector2D> vertices;
    std::list<HalfEdge> half_edges;

    friend FortuneAlgorithm;

    Vector2D *addVertex(Vector2D point) {
        vertices.push_back(point);
        return &vertices.back();
    }

    Vector2D *createCorner(Frame box, Frame::Side side) {
        switch (side) {
            case Frame::Side::LEFT:
                return addVertex(Vector2D(box.left, box.top));
            case Frame::Side::BOTTOM:
                return addVertex(Vector2D(box.left, box.bottom));
            case Frame::Side::RIGHT:
                return addVertex(Vector2D(box.right, box.bottom));
            case Frame::Side::TOP:
                return addVertex(Vector2D(box.right, box.top));
            default:
                return nullptr;
        }
    }

    VoronoiDiagram::HalfEdge *createHalfEdge(Cell *cell) {
        half_edges.emplace_back();
        half_edges.back().incidentCell = cell;
        if (cell->outerComponent == nullptr)
            cell->outerComponent = &half_edges.back();
        return &half_edges.back();
    }
};
