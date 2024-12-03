#ifndef KDTREE_H
#define KDTREE_H

#include "global_body_planner/planning_utils.h"
#include <vector>
#include <memory>
#include <algorithm>

class KDTree
{
public:
    KDTree() : root(nullptr) {}

    void insert(const State &state)
    {
        root = insertRec(root, state, 0);
    }

    bool search(const State &state) const
    {
        return searchRec(root, state, 0);
    }

    State nearestNeighbor(const State &state) const
    {
        return nearestNeighborRec(root, state, 0, root->state);
    }

    // Define the distance function for KD-tree
    double distance(const State &s1, const State &s2) const
    {
        return s1.distance(s2);
    }

private:
    struct Node
    {
        State state;
        std::shared_ptr<Node> left;
        std::shared_ptr<Node> right;

        Node(const State &state) : state(state), left(nullptr), right(nullptr) {}
    };

    std::shared_ptr<Node> root;

    std::shared_ptr<Node> insertRec(std::shared_ptr<Node> node, const State &state, int depth)
    {
        if (!node)
        {
            return std::make_shared<Node>(state);
        }

        int axis = depth % 3;
        if (state.pos[axis] < node->state.pos[axis])
        {
            node->left = insertRec(node->left, state, depth + 1);
        }
        else
        {
            node->right = insertRec(node->right, state, depth + 1);
        }

        return node;
    }

    bool searchRec(std::shared_ptr<Node> node, const State &state, int depth) const
    {
        if (!node)
        {
            return false;
        }

        if (node->state.pos == state.pos)
        {
            return true;
        }

        int axis = depth % 3;
        if (state.pos[axis] < node->state.pos[axis])
        {
            return searchRec(node->left, state, depth + 1);
        }
        else
        {
            return searchRec(node->right, state, depth + 1);
        }
    }

    State nearestNeighborRec(std::shared_ptr<Node> node, const State &target, int depth, State best) const
    {
        if (!node)
        {
            return best;
        }

        if (node->state.distance(target) < best.distance(target))
        {
            best = node->state;
        }

        int axis = depth % 3;
        std::shared_ptr<Node> nextNode = (target.pos[axis] < node->state.pos[axis]) ? node->left : node->right;
        std::shared_ptr<Node> otherNode = (target.pos[axis] < node->state.pos[axis]) ? node->right : node->left;

        best = nearestNeighborRec(nextNode, target, depth + 1, best);
        if (std::abs(target.pos[axis] - node->state.pos[axis]) < best.distance(target))
        {
            best = nearestNeighborRec(otherNode, target, depth + 1, best);
        }

        return best;
    }
};

#endif // KDTREE_H