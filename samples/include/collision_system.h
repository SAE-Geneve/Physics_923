#ifndef PHYSICS_SAMPLES_COLLISION_SYSTEM_H_
#define PHYSICS_SAMPLES_COLLISION_SYSTEM_H_

#include <unordered_map>

#include "game_object.h"
#include "quadtree.h"
#include <unordered_set>

#include "contact_solver.h"
#include "contact_listener.h"
#include "physics_world.h"

namespace crackitos_physics::samples
{
    //For testing purposes, this would be an implementation specific GameObject
    struct CollisionObject
    {
        physics::BodyHandle body;
        physics::ColliderHandle collider;
        SDL_Color color = {255, 13, 132, 255};

        void OnCollisionEnter() {
            Uint8 r = random::Range(128, 255);
            Uint8 g = random::Range(128, 255);
            Uint8 b = random::Range(128, 255);
            color = SDL_Color{r, g, b, 255};
        }
    };

    class CollisionContactListener final : public physics::ContactListener
    {
    public:
        std::unordered_map<int, CollisionObject*> collider_to_object_;

        void OnCollisionEnter(const physics::ColliderPair& pair) override;

        void OnCollisionExit(const physics::ColliderPair& pair) override{}
        void OnCollisionStay(const physics::ColliderPair& pair) override{}
        void OnTriggerEnter(const physics::ColliderPair& pair) override {}
        void OnTriggerStay(const physics::ColliderPair& pair) override {}
        void OnTriggerExit(const physics::ColliderPair& pair) override {}
    };

    class CollisionSystem
    {
    private:
        physics::PhysicsWorld physics_world_;
        CollisionContactListener contact_listener_;

        static constexpr int kMaxObjects = 200;
        std::vector<CollisionObject> collision_objects_ = {};

    public:
        CollisionSystem();
        ~CollisionSystem();

        void Initialize();
        void Clear();
        void Update(commons::fp delta_time);

        [[nodiscard]] const std::vector<CollisionObject>& collision_objects() const { return collision_objects_; }

        // void CreateObject(const math::Vec2f& pos, math::ShapeType type);
        // void UpdateCollisionObject();
        [[nodiscard]] physics::PhysicsWorld& physics_world() { return physics_world_; }
        [[nodiscard]] const physics::PhysicsWorld& physics_world() const { return physics_world_; } // Const version
    };
} // namespace samples
#endif // PHYSICS_SAMPLES_COLLISION_SYSTEM_H_
