#include "collision_system.h"

#include "contact_solver.h"
#include "display.h"
#include "random.h"

namespace crackitos_physics::samples
{
    CollisionSystem::CollisionSystem()
    {
        physics_world_.SetContactListener(&contact_listener_);
    }

    CollisionSystem::~CollisionSystem()
    {
        Clear();
    }

    void CollisionSystem::Initialize()
    {
        Clear();
        collision_objects_.reserve(kMaxObjects);
        physics_world_.Initialize(crackitos_core::math::AABB({0, 0}, {kWindowWidth, kWindowHeight}), true);

        for (int i = 0; i < kMaxObjects; ++i)
        {
            crackitos_core::math::Vec2f pos = {
                crackitos_core::random::Range(20.f, kWindowWidth - 20.f),
                crackitos_core::random::Range(20.f, kWindowHeight - 20.f)
            };

            crackitos_core::math::Vec2f vel = {
                crackitos_core::random::Range(-100.f, 100.f),
                crackitos_core::random::Range(-100.f, 100.f)
            };

            bool is_circle = (i % 2 == 0);
            physics::Body body(physics::BodyType::Dynamic, pos, vel, false, crackitos_core::random::Range(1.0f, 50.0f));
            physics::BodyHandle body_handle = physics_world_.CreateBody(body);

            physics::ColliderHandle collider_handle;
            if (is_circle)
            {
                crackitos_core::math::Circle shape(pos, crackitos_core::random::Range(5.f, 10.f));
                physics::Collider collider(shape, 1.f, 0.f, false, body_handle);
                collider_handle = physics_world_.CreateCollider(body_handle, collider);
            }
            else
            {
                crackitos_core::math::Vec2f half_size = {crackitos_core::random::Range(5.f, 10.f), crackitos_core::random::Range(5.f, 10.f)};
                crackitos_core::math::AABB shape(pos, half_size, half_size.Magnitude());
                physics::Collider collider(shape, 1.f, 0.f, false, body_handle);
                collider_handle = physics_world_.CreateCollider(body_handle, collider);
            }

            collision_objects_.push_back({body_handle, collider_handle});
            contact_listener_.collider_to_object_[collider_handle.id] = &collision_objects_.back();
        }
    }

    void CollisionSystem::Clear()
    {
        collision_objects_.clear();
        physics_world_.Clear();
    }


    void CollisionSystem::Update(crackitos_core::commons::fp delta_time)
    {
        // Bounce logic
        for (auto& obj : collision_objects_)
        {
            auto& body = physics_world_.GetMutableBody(obj.body);
            auto pos = body.position();
            auto vel = body.velocity();
            const crackitos_core::commons::fp radius = 10.0f; // approximate for both shape types

            if (pos.x - radius < 0.f)
            {
                pos.x = radius;
                vel.x *= -1.f;
            }
            if (pos.x + radius > kWindowWidth)
            {
                pos.x = kWindowWidth - radius;
                vel.x *= -1.f;
            }
            if (pos.y - radius < 0.f)
            {
                pos.y = radius;
                vel.y *= -1.f;
            }
            if (pos.y + radius > kWindowHeight)
            {
                pos.y = kWindowHeight - radius;
                vel.y *= -1.f;
            }

            body.set_position(pos);
            body.set_velocity(vel);
        }

        physics_world_.FixedUpdate(delta_time);
    }

    void CollisionContactListener::OnCollisionEnter(const physics::ColliderPair& pair)
    {
        for (const auto& id : {pair.colliderA.id, pair.colliderB.id})
        {
            auto it = collider_to_object_.find(id);
            if (it != collider_to_object_.end())
            {
                it->second->OnCollisionEnter();
            }
        }
    }
} // namespace samples
