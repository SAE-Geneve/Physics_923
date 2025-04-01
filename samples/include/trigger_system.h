#ifndef PHYSICS_SAMPLES_TRIGGER_SYSTEM_H_
#define PHYSICS_SAMPLES_TRIGGER_SYSTEM_H_

#include <unordered_map>
#include <unordered_set>

#include "physics_world.h"
#include "game_object.h"
#include "quadtree.h"
#include "contact_listener.h"



namespace crackitos_physics::samples
{
    //For testing purposes, this would be an implementation specific GameObject
    struct TriggerObject
    {
        physics::BodyHandle body;
        physics::ColliderHandle collider;
        SDL_Color color = {255, 13, 132, 255};
        int collisions = 0;

        void OnTriggerEnter() {
            if (++collisions == 1)
                color = SDL_Color{0, 255, 0, 255};
        }

        void OnTriggerExit() {
            if (--collisions <= 0) {
                collisions = 0;
                color = SDL_Color{255, 13, 132, 255};
            }
        }
    };


    class TriggerContactListener final : public physics::ContactListener
    {
    public:
        std::unordered_map<int, TriggerObject*> collider_to_object_;

        void OnTriggerEnter(const physics::ColliderPair& pair) override;
        void OnTriggerExit(const physics::ColliderPair& pair) override;

        void OnTriggerStay(const physics::ColliderPair& pair) override {}
        void OnCollisionEnter(const physics::ColliderPair& pair) override {}
        void OnCollisionStay(const physics::ColliderPair& pair) override {}
        void OnCollisionExit(const physics::ColliderPair& pair) override {}
    };

    class TriggerSystem
    {
    private:
        static constexpr int kMaxObjects = 400;
        physics::PhysicsWorld physics_world_;
        TriggerContactListener contact_listener_;
        std::vector<TriggerObject> trigger_objects_;

    public:
        TriggerSystem();
        ~TriggerSystem();

        void Initialize();
        void Clear();
        void Update(crackitos_core::commons::fp delta_time);

        [[nodiscard]] const std::vector<TriggerObject>& trigger_objects() const { return trigger_objects_; }

        // void CreateObject(math::ShapeType type);
        // void UpdateTriggerObjects();
        [[nodiscard]] physics::PhysicsWorld& physics_world() { return physics_world_; }
        [[nodiscard]] const physics::PhysicsWorld& physics_world() const { return physics_world_; } // Const version
    };
} // namespace samples
#endif // PHYSICS_SAMPLES_TRIGGER_SYSTEM_H_
