#include "quadtree.h"

namespace crackitos_physics::physics
{
    QuadtreeNode::QuadtreeNode(const math::AABB& box, const int depth): bounding_box_(box), depth_(depth)
    {
    }

    void QuadtreeNode::Subdivide()
    {
        math::Vec2f halfSize = (bounding_box_.max_bound() - bounding_box_.min_bound()) * 0.5f;
        math::Vec2f center = bounding_box_.min_bound() + halfSize;

        children_[0] = std::make_unique<QuadtreeNode>(math::AABB(bounding_box_.min_bound(), center), depth_ + 1);
        children_[1] = std::make_unique<QuadtreeNode>(math::AABB({center.x, bounding_box_.min_bound().y},
                                                                 {bounding_box_.max_bound().x, center.y}),
                                                      depth_ + 1);
        children_[2] = std::make_unique<QuadtreeNode>(math::AABB({bounding_box_.min_bound().x, center.y},
                                                                 {center.x, bounding_box_.max_bound().y}),
                                                      depth_ + 1);
        children_[3] = std::make_unique<QuadtreeNode>(math::AABB(center, bounding_box_.max_bound()), depth_ + 1);
    }

    bool QuadtreeNode::Insert(ColliderHandle collider, const math::AABB& shapeAABB)
    {
        if (!Intersect(bounding_box_, shapeAABB))
        {
            return false;
        }

        if (colliders_.size() < kMaxCollidersPerNode || depth_ == kMaxDepth)
        {
            colliders_.push_back(collider);
            return true;
        }

        if (!children_[0])
        {
            Subdivide();
        }

        bool inserted = false;
        for (auto& child : children_)
        {
            if (child && Intersect(child->bounding_box_, shapeAABB))
            {
                inserted |= child->Insert(collider, shapeAABB);
            }
        }

        if (!inserted)
        {
            colliders_.push_back(collider);
        }
        return true;
    }


    void QuadtreeNode::Query(const math::AABB& range,
                             std::vector<ColliderHandle>& foundColliders) const
    {
        if (!Intersect(bounding_box_, range))
        {
            return;
        }

        for (const auto& collider : colliders_)
        {
            foundColliders.push_back(collider);
        }

        for (const auto& child : children_)
        {
            if (child)
            {
                child->Query(range, foundColliders);
            }
        }
    }

    void QuadtreeNode::GetBoundingBoxes(std::vector<math::AABB>& boxes) const
    {
        boxes.push_back(bounding_box_);
        for (const auto& child : children_)
        {
            if (child)
            {
                child->GetBoundingBoxes(boxes);
            }
        }
    }

    Quadtree::Quadtree(const math::AABB& boundary): root_(std::make_unique<QuadtreeNode>(boundary))
    {
        bounding_boxes_.reserve(ComputeMaxNodes()); // Preallocate memory
    }

    void Quadtree::Insert(const ColliderHandle collider, const math::AABB& shapeAABB) const
    {
        if (collider.id >= 0)
        {
            root_->Insert(collider, shapeAABB);
        }
    }

    std::vector<ColliderHandle> Quadtree::Query(
        const math::AABB& range) const
    {
        std::vector<ColliderHandle> foundColliders;
        root_->Query(range, foundColliders);
        return foundColliders;
    }

    void Quadtree::BuildPotentialPairs()
    {

        potential_pairs_.clear();
        std::vector<ColliderHandle> allColliders;
        root_->Query(root_->bounding_box_, allColliders);

        for (size_t i = 0; i < allColliders.size(); ++i)
        {
            for (size_t j = i + 1; j < allColliders.size(); ++j)
            {
                ColliderPair pair{allColliders[i], allColliders[j]};
                potential_pairs_.insert(pair);
            }
        }

    }

    const std::unordered_set<ColliderPair>& Quadtree::GetPotentialPairs() const
    {
        return potential_pairs_;
    }


    void Quadtree::Clear()
    {
        potential_pairs_.clear();
        root_ = std::make_unique<QuadtreeNode>(root_->bounding_box_);
    }

    void Quadtree::UpdateBoundingBoxes()
    {
        bounding_boxes_.clear();
        root_->GetBoundingBoxes(bounding_boxes_);
    }
}
