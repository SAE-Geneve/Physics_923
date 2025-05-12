#include "quadtree.h"

namespace crackitos_physics::physics
{
    QuadtreeNode::QuadtreeNode(const crackitos_core::math::AABB& box, const int depth): bounding_box_(box), depth_(depth)
    {
    }

    QuadtreeNode::QuadtreeNode(const QuadtreeNode& other)
    : bounding_box_(other.bounding_box_),
      depth_(other.depth_),
      colliders_(other.colliders_)
    {
        for (size_t i = 0; i < 4; ++i)
        {
            if (other.children_[i])
            {
                children_[i] = std::make_unique<QuadtreeNode>(*other.children_[i]);
            }
        }
    }


    void QuadtreeNode::Subdivide()
    {
        crackitos_core::math::Vec2f halfSize = (bounding_box_.max_bound() - bounding_box_.min_bound()) * 0.5f;
        crackitos_core::math::Vec2f center = bounding_box_.min_bound() + halfSize;

        children_[0] = std::make_unique<QuadtreeNode>(crackitos_core::math::AABB(bounding_box_.min_bound(), center), depth_ + 1);
        children_[1] = std::make_unique<QuadtreeNode>(crackitos_core::math::AABB({center.x, bounding_box_.min_bound().y},
                                                                 {bounding_box_.max_bound().x, center.y}),
                                                      depth_ + 1);
        children_[2] = std::make_unique<QuadtreeNode>(crackitos_core::math::AABB({bounding_box_.min_bound().x, center.y},
                                                                 {center.x, bounding_box_.max_bound().y}),
                                                      depth_ + 1);
        children_[3] = std::make_unique<QuadtreeNode>(crackitos_core::math::AABB(center, bounding_box_.max_bound()), depth_ + 1);
    }

    bool QuadtreeNode::Insert(ColliderHandle collider, const crackitos_core::math::AABB& shapeAABB)
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


    void QuadtreeNode::Query(const crackitos_core::math::AABB& range,
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

    void QuadtreeNode::GetBoundingBoxes(std::vector<crackitos_core::math::AABB>& boxes) const
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

    Quadtree::Quadtree(const crackitos_core::math::AABB& boundary): root_(std::make_unique<QuadtreeNode>(boundary))
    {
        bounding_boxes_.reserve(ComputeMaxNodes()); // Preallocate memory
    }

    void Quadtree::Insert(const ColliderHandle collider, const crackitos_core::math::AABB& shapeAABB) const
    {
        if (collider.id >= 0)
        {
            root_->Insert(collider, shapeAABB);
        }
    }

    std::vector<ColliderHandle> Quadtree::Query(
        const crackitos_core::math::AABB& range) const
    {
        std::vector<ColliderHandle> foundColliders;
        root_->Query(range, foundColliders);
        return foundColliders;
    }

    void Quadtree::BuildPotentialPairs()
    {
        potential_pairs_.clear();
        root_->BuildPairs(potential_pairs_);
    }



    void QuadtreeNode::BuildPairs(std::unordered_set<ColliderPair>& out_pairs) const
    {
        // Compare each pair within this node
        for (size_t i = 0; i < colliders_.size(); ++i)
        {
            for (size_t j = i + 1; j < colliders_.size(); ++j)
            {
                auto a = colliders_[i];
                auto b = colliders_[j];
                if (a.id > b.id) std::swap(a, b);
                out_pairs.insert(ColliderPair{a, b});
            }
        }

        // Compare this node's colliders with each child's colliders
        for (const auto& child : children_)
        {
            if (child)
            {
                auto child_colliders = child->CollectAllColliders();
                for (const auto& a : colliders_)
                {
                    for (const auto& b : child_colliders)
                    {
                        auto aa = a, bb = b;
                        if (aa.id > bb.id) std::swap(aa, bb);
                        out_pairs.insert(ColliderPair{aa, bb});
                    }
                }
                child->BuildPairs(out_pairs);
            }
        }
    }

    std::vector<ColliderHandle> QuadtreeNode::CollectAllColliders() const
    {
        std::vector<ColliderHandle> result = colliders_;
        for (const auto& child : children_)
        {
            if (child)
            {
                auto child_result = child->CollectAllColliders();
                result.insert(result.end(), child_result.begin(), child_result.end());
            }
        }
        return result;
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

    void Quadtree::CopyFrom(const Quadtree& other)
    {
        if (other.root_)
        {
            root_ = std::make_unique<QuadtreeNode>(*other.root_);
        }
        else
        {
            root_.reset();
        }

        potential_pairs_ = other.potential_pairs_;
        bounding_boxes_ = other.bounding_boxes_;
    }

}
