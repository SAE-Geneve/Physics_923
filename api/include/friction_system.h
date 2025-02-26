#ifndef PHYSICS_923_API_FRICTION_SYSTEM_H_
#define PHYSICS_923_API_FRICTION_SYSTEM_H_

// Version History :
// 22.02.25 - Modified by Maxence - placed gravity here

#include <unordered_map>
#include <unordered_set>

#include "display.h"
#include "game_object.h"
#include "conversions.h"
#include "quadtree.h"
#include "shape.h"
#include "timer.h"

namespace physics923
{
    class FrictionSystem
    {
    private:
        std::vector<GameObject> objects_;

        physics::Quadtree* quadtree_ = nullptr;

        std::unordered_map<GameObjectPair, bool> potential_pairs_;
        std::unordered_set<GameObjectPair> active_pairs_;

        std::unordered_map<physics::Collider*, GameObject*> collider_to_object_map_;
        //Mapping from Collider to GameObject

        //Create the gravity for the scene
        static constexpr math::Vec2f gravity = conversions::ConvertToPixels(math::Vec2f(0.f, 9.8f));

        timer::Timer* timer_ = nullptr;
        math::AABB frame_bounds_ = math::AABB(math::Vec2f(0, 0), math::Vec2f(kWindowWidth, kWindowHeight));

    public:
        FrictionSystem() = default;
        ~FrictionSystem();

        void Initialize();
        void Clear();

        std::vector<GameObject> objects() { return objects_; }
        [[nodiscard]] physics::Quadtree* quadtree() const { return quadtree_; }

        void SpawnShape(math::Vec2f pos, math::ShapeType type);
        void CreateObject(size_t index, math::Circle& circle);
        void CreateObject(size_t index, math::AABB& aabb);
        //void CreateObject(size_t index, math::Polygon& polygon);
        void CreateGround();
        void DeleteObject(size_t index);
        void RemoveOutOfBoundsObjects();

        void RegisterObject(GameObject& object);
        void UnregisterObject(GameObject& object);

        void Update(physics923::commons::fp delta_time);
        void UpdateShapes(physics923::commons::fp delta_time);

        void SimplisticBroadPhase();
        void BroadPhase();
        void NarrowPhase();

        static void OnPairCollideStart(const GameObjectPair& pair);
        static void OnPairCollideStay(const GameObjectPair& pair);
        static void OnPairCollideEnd(const GameObjectPair& pair);
    };
}
#endif //PHYSICS_923_API_FRICTION_SYSTEM_H_
