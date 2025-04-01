#ifndef CRACKITOS_PHYSICS_PHYSICS_PHYSICS_WORLD_H_
#define CRACKITOS_PHYSICS_PHYSICS_PHYSICS_WORLD_H_

#include "commons.h"

#include <unordered_map>
#include <unordered_set>

#include "body.h"
#include "contact_listener.h"
#include "distance.h"
#include "quadtree.h"
#include "timer.h"

namespace crackitos_physics::physics
{
    static constexpr auto kDefaultGravity = math::Vec2f(
        static_cast<commons::fp>(distance::Convert<distance::Meter, distance::Pixel>(distance::Meter(0.f)).value),
        static_cast<commons::fp>(distance::Convert<distance::Meter, distance::Pixel>(distance::Meter(9.81f)).value)
    );

    class PhysicsWorld
    {
    private:
        math::AABB frame_bounds_;

        bool out_of_bounds_removal_state_;
        float out_of_bounds_margin_left_ = 10.0f;
        float out_of_bounds_margin_right_ = 10.0f;
        float out_of_bounds_margin_top_ = 10.0f;
        float out_of_bounds_margin_bottom_ = 10.0f;

        std::vector<Body> bodies_;
        std::vector<int> body_generations_;

        std::vector<Collider> colliders_;
        std::vector<int> collider_generations_;

        Quadtree quadtree_;

        std::unordered_set<ColliderPair> active_pairs_;

        math::Vec2f gravity_;

        commons::fp time_step_ = 1.0f / 60.0f;
        timer::Timer timer_;

        ContactListener* contact_listener_ = nullptr;

        void UpdateObjects();
        void RemoveOutOfBoundsObjects();
        void BroadPhase();
        void NarrowPhase();
        void ResolveCollisionPair(const ColliderPair& pair, bool is_new_pair);

    public:
        explicit PhysicsWorld();
        ~PhysicsWorld();

        void Initialize(const math::AABB& world_bounds, bool out_of_bounds_removal_state = true, math::Vec2f gravity = kDefaultGravity);
        void Clear();

        void SetContactListener(ContactListener* listener) { contact_listener_ = listener; }

        BodyHandle CreateBody(const Body& body_def);
        ColliderHandle CreateCollider(BodyHandle body, const Collider& collider_def);

        void RemoveBody(BodyHandle body);
        void RemoveCollider(ColliderHandle collider);

        [[nodiscard]] Body& GetBody(BodyHandle body);
        [[nodiscard]] const Collider& GetCollider(ColliderHandle collider) const;

        [[nodiscard]] std::vector<std::pair<BodyHandle, ColliderHandle>> GetBodiesWithColliders() const;

        void Update(commons::fp delta_time);
        void StepSimulation();

        void set_gravity(const math::Vec2f& new_gravity) { gravity_ = new_gravity; }
        void set_time_step(const commons::fp step) { time_step_ = step; }
        void set_out_of_bound_removal_state(const bool enable) { out_of_bounds_removal_state_ = enable; }
        void set_out_of_bounds_margins(float left, float right, float top, float bottom);

        [[nodiscard]] Quadtree& quadtree() { return quadtree_; }
    };
} // crackitos_physics

#endif //CRACKITOS_PHYSICS_PHYSICS_PHYSICS_WORLD_H_
