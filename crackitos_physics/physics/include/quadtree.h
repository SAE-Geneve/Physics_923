#ifndef PHYSICS_923_LIB_PHYSICS_QUADTREE_H_
#define PHYSICS_923_LIB_PHYSICS_QUADTREE_H_

#include <array>
#include <memory>
#include <vector>

#include "collider.h"
#include "shape.h"
#include "commons.h"

namespace physics923::physics
{
    static constexpr int kMaxDepth_ = 6;
    static constexpr int kMaxShapeCount_ = 8;

    struct QuadtreeNode
    {
        math::AABB bounding_box_{};
        std::array<std::unique_ptr<QuadtreeNode>, 4> children_{}; //Unique pointers for automatic memory management
        std::vector<Collider*> colliders_; //Colliders stored in each node
        int depth_;

        explicit QuadtreeNode(const math::AABB& box, const int depth = 0)
            : bounding_box_(box), depth_(depth)
        {
        }

        //Subdivide the node into 4 child quadrants
        void Subdivide()
        {
            math::Vec2f halfSize = (bounding_box_.max_bound() - bounding_box_.min_bound()) * 0.5f;
            math::Vec2f center = bounding_box_.min_bound() + halfSize;

            children_[0] = std::make_unique<QuadtreeNode>(math::AABB(bounding_box_.min_bound(), center), depth_ + 1);
            children_[1] = std::make_unique<QuadtreeNode>(math::AABB(math::Vec2f(center.x, bounding_box_.min_bound().y),
                                                                     math::Vec2f(
                                                                         bounding_box_.max_bound().x, center.y)),
                                                          depth_ + 1);
            children_[2] = std::make_unique<QuadtreeNode>(math::AABB(math::Vec2f(bounding_box_.min_bound().x, center.y),
                                                                     math::Vec2f(
                                                                         center.x, bounding_box_.max_bound().y)),
                                                          depth_ + 1);
            children_[3] = std::make_unique<QuadtreeNode>(math::AABB(center, bounding_box_.max_bound()), depth_ + 1);
        }

        //Insert a collider into this node or its children
        bool Insert(Collider* collider)
        {
            math::AABB shapeAABB = collider->GetBoundingBox();

            if (!bounding_box_.Contains(shapeAABB.min_bound()) || !bounding_box_.Contains(shapeAABB.max_bound()))
            {
                return false;
            }

            if (colliders_.size() < kMaxShapeCount_ || depth_ == kMaxDepth_)
            {
                colliders_.push_back(collider);
                return true;
            }

            if (!children_[0])
            {
                Subdivide();
            }

            for (auto& child : children_)
            {
                if (child && child->Insert(collider))
                {
                    return true;
                }
            }

            colliders_.push_back(collider); //If shape overlaps multiple quadrants, keep it in this node
            return true;
        }

        //Query colliders within a given area
        void Query(const math::AABB& range, std::vector<Collider*>& foundColliders) const
        {
            if (!math::Intersect(bounding_box_, range))
            {
                return;
            }

            for (const auto& collider : colliders_)
            {
                if (math::Intersect(collider->GetBoundingBox(), range))
                {
                    foundColliders.push_back(collider);
                }
            }

            for (const auto& child : children_)
            {
                if (child)
                {
                    child->Query(range, foundColliders);
                }
            }
        }

        // Provide access to bounding boxes for external rendering
        void GetBoundingBoxes(std::vector<math::AABB>& boxes) const
        {
            boxes.push_back(bounding_box_); // Store this node’s bounding box
            for (const auto& child : children_)
            {
                if (child)
                {
                    child->GetBoundingBoxes(boxes);
                }
            }
        }
    };

    class Quadtree
    {
    private:
        std::unique_ptr<QuadtreeNode> root_;
        std::vector<math::AABB> bounding_boxes_; // Preallocated storage for bounding boxes

        // Helper function to compute the maximum number of nodes
        static constexpr int ComputeMaxNodes()
        {
            int total_nodes = 0;
            int nodes_at_depth = 1; // Start with root node

            for (int i = 0; i <= kMaxDepth_; ++i)
            {
                total_nodes += nodes_at_depth;
                nodes_at_depth *= 4; // Each node can split into 4
            }

            return total_nodes;
        }

    public:
        explicit Quadtree(const math::AABB& boundary)
            : root_(std::make_unique<QuadtreeNode>(boundary))
        {
            bounding_boxes_.reserve(ComputeMaxNodes()); // Preallocate memory
        }

        void Insert(Collider* collider)
        {
            if (collider)
            {
                root_->Insert(collider);
            }
        }

        [[nodiscard]] std::vector<Collider*> Query(const math::AABB& range) const
        {
            std::vector<Collider*> foundColliders;
            root_->Query(range, foundColliders);
            return foundColliders;
        }

        void Clear()
        {
            root_ = std::make_unique<QuadtreeNode>(root_->bounding_box_);
        }

        // Fetch all bounding boxes for external rendering
        const std::vector<math::AABB>& GetBoundingBoxes()
        {
            bounding_boxes_.clear(); // Clear but keep capacity
            root_->GetBoundingBoxes(bounding_boxes_);
            return bounding_boxes_;
        }
    };
}


#endif //PHYSICS_923_LIB_PHYSICS_QUADTREE_H_
