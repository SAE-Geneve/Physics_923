#ifndef PHYSICS_SAMPLES_FRICTION_SYSTEM_H_
#define PHYSICS_SAMPLES_FRICTION_SYSTEM_H_

#include <unordered_map>
#include <unordered_set>

#include "contact_listener.h"
#include "display.h"
#include "game_object.h"
#include "quadtree.h"
#include "shape.h"
#include "timer.h"
#include "distance.h"
#include "physics_world.h"

namespace crackitos_physics::samples
{
    //For testing purposes, this would be an implementation specific GameObject
    struct FrictionObject
    {
        physics::BodyHandle body;
        physics::ColliderHandle collider;
        SDL_Color color = {255, 13, 132, 255};
    };

    // Contact Listener for Logging Events
    class FrictionContactListener final : public physics::ContactListener
    {
    public:
        void OnTriggerEnter(const physics::ColliderPair& pair) override;
        void OnTriggerStay(const physics::ColliderPair& pair) override;
        void OnTriggerExit(const physics::ColliderPair& pair) override;

        void OnCollisionEnter(const physics::ColliderPair& pair) override;
        void OnCollisionStay(const physics::ColliderPair& pair) override;
        void OnCollisionExit(const physics::ColliderPair& pair) override;
    };

    class FrictionSystem
    {
    private:
        physics::PhysicsWorld physics_world_;
        FrictionContactListener contact_listener_;

        std::vector<FrictionObject> friction_objects_;

    public:
        FrictionSystem();
        ~FrictionSystem();

        void Initialize();
        void Clear();

        void SpawnShape(crackitos_core::math::Vec2f pos, crackitos_core::math::ShapeType type);
        void CreateGround();

        void Update(crackitos_core::commons::fp delta_time);
        void SetGravity(const crackitos_core::math::Vec2f& gravity);

        void UpdateTestingObjects();

        [[nodiscard]] const std::vector<FrictionObject>& friction_objects() const { return friction_objects_; }

        [[nodiscard]] physics::PhysicsWorld& physics_world() { return physics_world_; }
        [[nodiscard]] const physics::PhysicsWorld& physics_world() const { return physics_world_; } // Const version
    };
} // namespace samples
#endif // PHYSICS_SAMPLES_FRICTION_SYSTEM_H_
