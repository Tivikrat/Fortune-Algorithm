#pragma once

#include <limits>
#include <cmath>

#include "Vector2D.h"
#include "VoronoiDiagram.h"
#include "ArcNode.h"

class ArcNode;

class BeachLine {
public:
    BeachLine() : nil(new ArcNode), root(nil) {
        nil->color = ArcNode::Color::BLACK;
    }

    ~BeachLine() {
        free(root);
        delete nil;
    }

    ArcNode *createArc(VoronoiDiagram::Site *site) {
        return new ArcNode{nil, nil, nil, site, nullptr, nullptr, nullptr, nil,
                           nil,
                           ArcNode::Color::RED};
    }

    bool isEmpty() const {
        return isNil(root);
    }

    bool isNil(const ArcNode *x) const {
        return x == nil;
    }

    void setRoot(ArcNode *x) {
        root = x;
        root->color = ArcNode::Color::BLACK;
    }

    ArcNode *getLeftmostArc() const {
        ArcNode *x = root;
        while (!isNil(x->previous))
            x = x->previous;
        return x;
    }

    ArcNode *locateArcAbove(const Vector2D &point, double line) const {
        ArcNode *node = root;
        bool found = false;
        while (!found) {
            double breakpointLeft = -std::numeric_limits<double>::infinity();
            double breakpointRight = std::numeric_limits<double>::infinity();
            if (!isNil(node->previous))
                breakpointLeft = getBreakpoint(node->previous->site->point,
                                               node->site->point, line);
            if (!isNil(node->next))
                breakpointRight = getBreakpoint(node->site->point,
                                                node->next->site->point,
                                                line);
            if (point.x < breakpointLeft)
                node = node->left;
            else if (point.x > breakpointRight)
                node = node->right;
            else
                found = true;
        }
        return node;
    }

    void insertBefore(ArcNode *tree, ArcNode *arc) {
        if (isNil(tree->left)) {
            tree->left = arc;
            arc->parent = tree;
        } else {
            tree->previous->right = arc;
            arc->parent = tree->previous;
        }
        arc->previous = tree->previous;
        if (!isNil(arc->previous))
            arc->previous->next = arc;
        arc->next = tree;
        tree->previous = arc;
        balance(arc);
    }

    void insertAfter(ArcNode *tree, ArcNode *arc) {
        if (isNil(tree->right)) {
            tree->right = arc;
            arc->parent = tree;
        } else {
            tree->next->left = arc;
            arc->parent = tree->next;
        }
        arc->next = tree->next;
        if (!isNil(arc->next))
            arc->next->previous = arc;
        arc->previous = tree;
        tree->next = arc;
        balance(arc);
    }

    void replace(ArcNode *first_arc, ArcNode *second_arc) {
        transplant(first_arc, second_arc);
        second_arc->left = first_arc->left;
        if (!isNil(second_arc->left))
            second_arc->left->parent = second_arc;
        second_arc->right = first_arc->right;
        if (!isNil(second_arc->right))
            second_arc->right->parent = second_arc;
        second_arc->previous = first_arc->previous;
        if (!isNil(second_arc->previous))
            second_arc->previous->next = second_arc;
        second_arc->next = first_arc->next;
        if (!isNil(second_arc->next))
            second_arc->next->previous = second_arc;
        second_arc->color = first_arc->color;
    }

    void remove(ArcNode *arc) {
        ArcNode *y = arc;
        auto yOriginalColor = y->color;
        ArcNode *x;
        if (isNil(arc->left)) {
            x = arc->right;
            transplant(arc, arc->right);
        } else if (isNil(arc->right)) {
            x = arc->left;
            transplant(arc, arc->left);
        } else {
            y = minimum(arc->right);
            yOriginalColor = y->color;
            x = y->right;
            if (y->parent == arc)
                x->parent = y; // Because x could be Nil
            else {
                transplant(y, y->right);
                y->right = arc->right;
                y->right->parent = y;
            }
            transplant(arc, y);
            y->left = arc->left;
            y->left->parent = y;
            y->color = arc->color;
        }
        if (yOriginalColor == ArcNode::Color::BLACK)
            removeFixup(x);
        if (!isNil(arc->previous))
            arc->previous->next = arc->next;
        if (!isNil(arc->next))
            arc->next->previous = arc->previous;
    }

    std::ostream &print(std::ostream &os) const {
        ArcNode *arc = getLeftmostArc();
        while (!isNil(arc)) {
            os << arc->site->index << ' ';
            arc = arc->next;
        }
        return os;
    }

private:
    ArcNode *nil;
    ArcNode *root;

    ArcNode *minimum(ArcNode *x) const {
        while (!isNil(x->left))
            x = x->left;
        return x;
    }

    void transplant(ArcNode *first, ArcNode *second) {
        if (isNil(first->parent))
            root = second;
        else if (first == first->parent->left)
            first->parent->left = second;
        else
            first->parent->right = second;
        second->parent = first->parent;
    }

    void balance(ArcNode *tree) {
        while (tree->parent->color == ArcNode::Color::RED) {
            if (tree->parent == tree->parent->parent->left) {
                ArcNode *y = tree->parent->parent->right;
                if (y->color == ArcNode::Color::RED) {
                    tree->parent->color = ArcNode::Color::BLACK;
                    y->color = ArcNode::Color::BLACK;
                    tree->parent->parent->color = ArcNode::Color::RED;
                    tree = tree->parent->parent;
                } else {
                    if (tree == tree->parent->right) {
                        tree = tree->parent;
                        leftRotate(tree);
                    }
                    tree->parent->color = ArcNode::Color::BLACK;
                    tree->parent->parent->color = ArcNode::Color::RED;
                    rightRotate(tree->parent->parent);
                }
            } else {
                ArcNode *y = tree->parent->parent->left;
                if (y->color == ArcNode::Color::RED) {
                    tree->parent->color = ArcNode::Color::BLACK;
                    y->color = ArcNode::Color::BLACK;
                    tree->parent->parent->color = ArcNode::Color::RED;
                    tree = tree->parent->parent;
                } else {
                    if (tree == tree->parent->left) {
                        tree = tree->parent;
                        rightRotate(tree);
                    }
                    tree->parent->color = ArcNode::Color::BLACK;
                    tree->parent->parent->color = ArcNode::Color::RED;
                    leftRotate(tree->parent->parent);
                }
            }
        }
        root->color = ArcNode::Color::BLACK;
    }

    void removeFixup(ArcNode *tree) {
        while (tree != root && tree->color == ArcNode::Color::BLACK) {
            ArcNode *arc;
            if (tree == tree->parent->left) {
                arc = tree->parent->right;
                if (arc->color == ArcNode::Color::RED) {
                    arc->color = ArcNode::Color::BLACK;
                    tree->parent->color = ArcNode::Color::RED;
                    leftRotate(tree->parent);
                    arc = tree->parent->right;
                }
                if (arc->left->color == ArcNode::Color::BLACK &&
                    arc->right->color == ArcNode::Color::BLACK) {
                    arc->color = ArcNode::Color::RED;
                    tree = tree->parent;
                } else {
                    if (arc->right->color == ArcNode::Color::BLACK) {
                        arc->left->color = ArcNode::Color::BLACK;
                        arc->color = ArcNode::Color::RED;
                        rightRotate(arc);
                        arc = tree->parent->right;
                    }
                    arc->color = tree->parent->color;
                    tree->parent->color = ArcNode::Color::BLACK;
                    arc->right->color = ArcNode::Color::BLACK;
                    leftRotate(tree->parent);
                    tree = root;
                }
            } else {
                arc = tree->parent->left;
                if (arc->color == ArcNode::Color::RED) {
                    arc->color = ArcNode::Color::BLACK;
                    tree->parent->color = ArcNode::Color::RED;
                    rightRotate(tree->parent);
                    arc = tree->parent->left;
                }
                if (arc->left->color == ArcNode::Color::BLACK &&
                    arc->right->color == ArcNode::Color::BLACK) {
                    arc->color = ArcNode::Color::RED;
                    tree = tree->parent;
                } else {
                    if (arc->left->color == ArcNode::Color::BLACK) {
                        arc->right->color = ArcNode::Color::BLACK;
                        arc->color = ArcNode::Color::RED;
                        leftRotate(arc);
                        arc = tree->parent->left;
                    }
                    arc->color = tree->parent->color;
                    tree->parent->color = ArcNode::Color::BLACK;
                    arc->left->color = ArcNode::Color::BLACK;
                    rightRotate(tree->parent);
                    tree = root;
                }
            }
        }
        tree->color = ArcNode::Color::BLACK;
    }

    void leftRotate(ArcNode *tree) {
        ArcNode *subtree = tree->right;
        tree->right = subtree->left;
        if (!isNil(subtree->left))
            subtree->left->parent = tree;
        subtree->parent = tree->parent;
        if (isNil(tree->parent))
            root = subtree;
        else if (tree->parent->left == tree)
            tree->parent->left = subtree;
        else
            tree->parent->right = subtree;
        subtree->left = tree;
        tree->parent = subtree;
    }

    void rightRotate(ArcNode *tree) {
        ArcNode *subtree = tree->left;
        tree->left = subtree->right;
        if (!isNil(subtree->right))
            subtree->right->parent = tree;
        subtree->parent = tree->parent;
        if (isNil(tree->parent))
            root = subtree;
        else if (tree->parent->left == tree)
            tree->parent->left = subtree;
        else
            tree->parent->right = subtree;
        subtree->right = tree;
        tree->parent = subtree;
    }

    double getBreakpoint(const Vector2D &point1, const Vector2D &point2,
                         double line) const {
        double x1 = point1.x;
        double y1 = point1.y;
        double x2 = point2.x;
        double y2 = point2.y;
        double d1 = 1.0 / (2.0 * (y1 - line));
        double d2 = 1.0 / (2.0 * (y2 - line));
        double a = d1 - d2;
        double b = 2.0 * (x2 * d2 - x1 * d1);
        double c = (y1 * y1 + x1 * x1 - line * line) * d1 -
                   (y2 * y2 + x2 * x2 - line * line) * d2;
        double delta = b * b - 4.0 * a * c;
        return (-b + std::sqrt(delta)) / (2.0 * a);
    }

    void free(ArcNode *tree) {
        if (isNil(tree))
            return;
        else {
            free(tree->left);
            free(tree->right);
            delete tree;
        }
    }

    std::ostream &
    printArc(std::ostream &os, const ArcNode *arc, std::string tabs) const {
        os << tabs << arc->site->index << ' ' << arc->leftHalfEdge << ' '
           << arc->rightHalfEdge << std::endl;
        if (!isNil(arc->left))
            printArc(os, arc->left, tabs + '\t');
        if (!isNil(arc->right))
            printArc(os, arc->right, tabs + '\t');
        return os;
    }
};

std::ostream &operator<<(std::ostream &os, const BeachLine &beachline) {
    return beachline.print(os);
}
