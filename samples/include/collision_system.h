#ifndef PHYSICS_SAMPLES_COLLISION_SYSTEM_H_
#define PHYSICS_SAMPLES_COLLISION_SYSTEM_H_

#include <unordered_map>

#include "game_object.h"
#include "quadtree.h"
#include <unordered_set>

#include "contact_solver.h"

namespace crackitos_physics::samples
{


    class CollisionSystem
    {
    private: //TODO add this to world if it's kept in (as a cosntant)
        int solver_iterations = 3;
        int number_of_objects_ = 200;
        std::array<GameObject, 200> objects_ = {};
        physics::Quadtree quadtree_;

        std::unordered_map<GameObjectPair, bool> potential_pairs_;
        std::unordered_set<GameObjectPair> active_pairs_;
        //Mapping from Collider to GameObject
        std::unordered_map<physics::Collider*, GameObject*> collider_to_object_map_;

    public:
        CollisionSystem();
        ~CollisionSystem() = default;

        void Initialize();
        void Clear();

        std::array<GameObject, 200> objects() { return objects_; }
        [[nodiscard]] physics::Quadtree& quadtree() { return quadtree_; }

        void CreateObject(size_t index, math::Circle& circle);
        void CreateObject(size_t index, math::AABB& aabb);
        //void CreateObject(size_t index, math::Polygon& polygon);
        void DeleteObject(size_t index);

        void RegisterObject(GameObject& object);
        void UnregisterObject(GameObject& object);

        void Update(crackitos_physics::commons::fp delta_time);
        void UpdateShapes(crackitos_physics::commons::fp delta_time);

        void SimplisticBroadPhase();
        void BroadPhase();
        void NarrowPhase();
        void ResolveCollisionPair(const GameObjectPair& pair, bool is_new_pair);

        static void OnPairCollideEnd(const GameObjectPair& pair);
        void PostResolveContactIterations(int iterations) const;
    };
} // namespace samples
#endif // PHYSICS_SAMPLES_COLLISION_SYSTEM_H_
