#ifndef CRACKITOS_PHYSICS_PHYSICS_QUADTREE_H_
#define CRACKITOS_PHYSICS_PHYSICS_QUADTREE_H_

#include <array>
#include <memory>
#include <unordered_set>
#include <vector>

#include "collider.h"
#include "shape.h"

namespace crackitos_physics::physics
{
    static constexpr int kMaxDepth = 6;
    static constexpr int kMaxCollidersPerNode = 8;

    struct QuadtreeNode
    {
        crackitos_core::math::AABB bounding_box_{};
        std::array<std::unique_ptr<QuadtreeNode>, 4> children_{}; //Unique pointers for automatic memory management
        std::vector<ColliderHandle> colliders_; //Colliders stored in each node
        int depth_;

        explicit QuadtreeNode(const crackitos_core::math::AABB& box, int depth = 0);

        //Subdivide the node into 4 child quadrants
        void Subdivide();

        //Insert a collider into this node or its children
        bool Insert(ColliderHandle collider, const crackitos_core::math::AABB& shapeAABB);

        //Query colliders within a given area
        void Query(const crackitos_core::math::AABB& range, std::vector<ColliderHandle>& foundColliders) const;

        // Provide access to bounding boxes for external rendering
        void GetBoundingBoxes(std::vector<crackitos_core::math::AABB>& boxes) const;
        void BuildPairs(std::unordered_set<ColliderPair>& out_pairs) const;
        std::vector<ColliderHandle> CollectAllColliders() const;
    };

    class Quadtree
    {
    private:
        std::unique_ptr<QuadtreeNode> root_;
        std::unordered_set<ColliderPair> potential_pairs_;
        std::vector<crackitos_core::math::AABB> bounding_boxes_; // Preallocated storage for bounding boxes

        // Helper function to compute the maximum number of nodes
        static constexpr int ComputeMaxNodes()
        {
            int total_nodes = 0;
            int nodes_at_depth = 1; // Start with root node

            for (int i = 0; i <= kMaxDepth; ++i)
            {
                total_nodes += nodes_at_depth;
                nodes_at_depth *= 4; // Each node can split into 4
            }

            return total_nodes;
        }

    public:
        explicit Quadtree(const crackitos_core::math::AABB& boundary);

        void Insert(ColliderHandle collider, const crackitos_core::math::AABB& shapeAABB) const;

        [[nodiscard]] std::vector<ColliderHandle> Query(const crackitos_core::math::AABB& range) const;

        void BuildPotentialPairs();

        [[nodiscard]] const std::unordered_set<ColliderPair>& GetPotentialPairs() const;

        void Clear();

        void UpdateBoundingBoxes();

        [[nodiscard]] const std::vector<crackitos_core::math::AABB>& GetBoundingBoxes() const
        {
            return bounding_boxes_;
        }
    };
}


#endif // CRACKITOS_PHYSICS_PHYSICS_QUADTREE_H_
