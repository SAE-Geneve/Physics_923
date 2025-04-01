#ifndef PHYSICS_SAMPLES_TESTING_SYSTEM_H_
#define PHYSICS_SAMPLES_TESTING_SYSTEM_H_

#include <vector>

#include "display.h"
#include "physics_world.h"
#include "shape.h"
#include "contact_listener.h"

namespace crackitos_physics::samples {

    //For testing purposes, this would be an implementation specific GameObject
    struct TestingObject
    {
        physics::BodyHandle body;
        physics::ColliderHandle collider;
        SDL_Color color = {255, 13, 132, 255};
    };

    // Contact Listener for Logging Events
    class TestingContactListener final : public physics::ContactListener
    {
    public:
        void OnTriggerEnter(const physics::ColliderPair& pair) override;
        void OnTriggerStay(const physics::ColliderPair& pair) override;
        void OnTriggerExit(const physics::ColliderPair& pair) override;

        void OnCollisionEnter(const physics::ColliderPair& pair) override;
        void OnCollisionStay(const physics::ColliderPair& pair) override;
        void OnCollisionExit(const physics::ColliderPair& pair) override;
    };

    // Main Testing System
    class TestingSystem {
        physics::PhysicsWorld physics_world_;
        TestingContactListener contact_listener_;

        std::vector<TestingObject> testing_objects_;

    public:
        TestingSystem();
        ~TestingSystem();

        void Initialize();
        void Clear();

        void SpawnShape(const math::Vec2f& pos, math::ShapeType type);
        void CreateGround();

        void Update(commons::fp delta_time);
        void SetGravity(const math::Vec2f& gravity);

        void UpdateTestingObjects();
        [[nodiscard]] const std::vector<TestingObject>& testing_objects() const { return testing_objects_; }

        [[nodiscard]] physics::PhysicsWorld& physics_world() { return physics_world_; }
        [[nodiscard]] const physics::PhysicsWorld& physics_world() const { return physics_world_; } // Const version
    };
} // namespace crackitos_physics::samples

#endif // PHYSICS_SAMPLES_TESTING_SYSTEM_H_
