#ifndef PHYSICS_923_LIB_PHYSICS_QUADTREE_H_
#define PHYSICS_923_LIB_PHYSICS_QUADTREE_H_

#include <SDL_render.h>

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

        //Method to draw the quadtree node
        void Draw(SDL_Renderer* renderer) const
        {
            //Draw the bounding box of this node
            SDL_Rect rect;
            rect.x = static_cast<int>(bounding_box_.min_bound().x);
            rect.y = static_cast<int>(bounding_box_.min_bound().y);
            rect.w = static_cast<int>(bounding_box_.max_bound().x - bounding_box_.min_bound().x);
            rect.h = static_cast<int>(bounding_box_.max_bound().y - bounding_box_.min_bound().y);

            SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
            SDL_RenderDrawRect(renderer, &rect); //Draw the rectangle

            //Recursively draw the children
            for (const auto& child : children_)
            {
                if (child)
                {
                    child->Draw(renderer);
                }
            }
        }
    };

    class Quadtree
    {
    private:
        std::unique_ptr<QuadtreeNode> root_;

    public:
        explicit Quadtree(const math::AABB& boundary)
            : root_(std::make_unique<QuadtreeNode>(boundary))
        {
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

        //Method to draw the quadtree, for testing purposes
        void Draw(SDL_Renderer* renderer) const
        {
            root_->Draw(renderer);
        }
    };
}


#endif //PHYSICS_923_LIB_PHYSICS_QUADTREE_H_
