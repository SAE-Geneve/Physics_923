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
        physics_world_.Initialize(math::AABB({0, 0}, {kWindowWidth, kWindowHeight}), true);

        for (int i = 0; i < kMaxObjects; ++i)
        {
            math::Vec2f pos = {
                random::Range(20.f, kWindowWidth - 20.f),
                random::Range(20.f, kWindowHeight - 20.f)
            };

            math::Vec2f vel = {
                random::Range(-100.f, 100.f),
                random::Range(-100.f, 100.f)
            };

            bool is_circle = (i % 2 == 0);
            physics::Body body(physics::BodyType::Dynamic, pos, vel, false, random::Range(1.0f, 50.0f));
            physics::BodyHandle body_handle = physics_world_.CreateBody(body);

            physics::ColliderHandle collider_handle;
            if (is_circle)
            {
                math::Circle shape(pos, random::Range(5.f, 10.f));
                physics::Collider collider(shape, 1.f, 0.f, false, body_handle);
                collider_handle = physics_world_.CreateCollider(body_handle, collider);
            }
            else
            {
                math::Vec2f half_size = {random::Range(5.f, 10.f), random::Range(5.f, 10.f)};
                math::AABB shape(pos, half_size, half_size.Magnitude());
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

    // void CollisionSystem::CreateObject(const math::Vec2f& pos, math::ShapeType type)
    // {
    //     physics::BodyHandle body;
    //
    //     switch (type)
    //     {
    //     case math::ShapeType::kAABB:
    //         {
    //             math::Vec2f velocity(random::Range(-50.0f, 50.0f), random::Range(-50.0f, 50.0f));
    //
    //             float half_size_x = random::Range(5.f, 20.f);
    //             float half_size_y = random::Range(5.f, 20.f);
    //
    //             math::AABB aabb(pos, math::Vec2f(half_size_x, half_size_y),
    //                             math::Vec2f(half_size_x, half_size_y).Magnitude());
    //
    //             physics::Body body_def(physics::BodyType::Dynamic, pos, velocity,
    //                                    false, random::Range(50.f, 100.f));
    //             body = physics_world_.CreateBody(body_def);
    //
    //             physics::Collider collider_def(aabb, random::Range(0.0f, 0.0f), 0.5f, true, body);
    //             physics_world_.CreateCollider(body, collider_def);
    //             break;
    //         }
    //     case math::ShapeType::kCircle:
    //         {
    //             math::Vec2f velocity(random::Range(-50.0f, 50.0f), random::Range(-50.0f, 50.0f));
    //
    //             float radius = random::Range(5.f, 20.f);
    //             math::Circle circle(pos, radius);
    //
    //             physics::Body body_def(physics::BodyType::Dynamic, pos, velocity, false, random::Range(1.0f, 50.f));
    //             body = physics_world_.CreateBody(body_def);
    //
    //             physics::Collider collider_def(circle, random::Range(0.2f, 0.8f), 0.5f, true, body);
    //             physics_world_.CreateCollider(body, collider_def);
    //
    //             break;
    //         }
    //     case math::ShapeType::kPolygon:
    //     case math::ShapeType::kNone:
    //     default:
    //         return; // Do nothing
    //     }
    // }

    void CollisionSystem::Update(commons::fp delta_time)
    {
        // Bounce logic
        for (auto& obj : collision_objects_)
        {
            auto& body = physics_world_.GetMutableBody(obj.body);
            auto pos = body.position();
            auto vel = body.velocity();
            const float radius = 10.0f; // approximate for both shape types

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

        physics_world_.Update(delta_time);
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

    // void CollisionSystem::UpdateCollisionObject()
    // {
    //     collision_objects_.clear(); // Clear previous frame's objects
    //
    //     auto bodies_with_colliders = physics_world_.GetBodiesWithColliders();
    //
    //     for (const auto& [body_handle, collider_handle] : bodies_with_colliders)
    //     {
    //         const physics::Body& body = physics_world_.GetMutableBody(body_handle);
    //         auto position = body.position(); // Get current position
    //
    //
    //         if (position.x < 0 || position.x > kWindowWidth ||
    //             position.y < 0 || position.y > kWindowHeight)
    //         {
    //             continue; // Skip adding objects that are already out of bounds
    //         }
    //
    //         CollisionObject obj;
    //         obj.body = body_handle;
    //         obj.collider = collider_handle;
    //         obj.color = SDL_Color{255, 13, 132, 255};
    //         collision_objects_.push_back(obj);
    //     }
    // }
} // namespace samples
