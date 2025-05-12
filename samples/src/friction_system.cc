#include "friction_system.h"

#include <iostream>
#include <ranges>

#include "contact_solver.h"
#include "random.h"

namespace crackitos_physics::samples
{
    //Contact Listener Implementations
    void FrictionContactListener::OnTriggerEnter(const physics::ColliderPair& pair)
    {

        // std::cout << "Trigger Enter: " << pair.colliderA.id << " and " << pair.colliderB.id << "\n";
    }

    void FrictionContactListener::OnTriggerStay(const physics::ColliderPair& pair)
    {
        // std::cout << "Trigger Stay: " << pair.colliderA.id << " and " << pair.colliderB.id << "\n";
    }

    void FrictionContactListener::OnTriggerExit(const physics::ColliderPair& pair)
    {
        // std::cout << "Trigger Exit: " << pair.colliderA.id << " and " << pair.colliderB.id << "\n";
    }

    void FrictionContactListener::OnCollisionEnter(const physics::ColliderPair& pair)
    {
        // std::cout << "Collision Enter: " << pair.colliderA.id << " and " << pair.colliderB.id << "\n";
    }

    void FrictionContactListener::OnCollisionStay(const physics::ColliderPair& pair)
    {
        // std::cout << "Collision Stay: " << pair.colliderA.id << " and " << pair.colliderB.id << "\n";
    }

    void FrictionContactListener::OnCollisionExit(const physics::ColliderPair& pair)
    {
        // std::cout << "Collision Exit: " << pair.colliderA.id << " and " << pair.colliderB.id << "\n";
    }

    FrictionSystem::FrictionSystem()
    {
        physics_world_.SetContactListener(&contact_listener_);
        friction_objects_.reserve(1000);
    }

    FrictionSystem::~FrictionSystem()
    {
        Clear();
    }

    void FrictionSystem::Initialize()
    {
        constexpr auto system_gravity = crackitos_core::math::Vec2f(
                static_cast<crackitos_core::commons::fp>(crackitos_core::distance::Convert<crackitos_core::distance::Meter, crackitos_core::distance::Pixel>(crackitos_core::distance::Meter(0.f)).value),
                static_cast<crackitos_core::commons::fp>(crackitos_core::distance::Convert<crackitos_core::distance::Meter, crackitos_core::distance::Pixel>(crackitos_core::distance::Meter(100.f)).value)
            );
        Clear();
        physics_world_.Initialize(crackitos_core::math::AABB(crackitos_core::math::Vec2f(0, 0), crackitos_core::math::Vec2f(kWindowWidth, kWindowHeight)), true, system_gravity);
        CreateGround();
    }

    void FrictionSystem::Clear()
    {
        friction_objects_.clear();
        physics_world_.Clear();
    }


    void FrictionSystem::SpawnShape(const crackitos_core::math::Vec2f pos, const crackitos_core::math::ShapeType type)
    {
        physics::BodyHandle body;

        switch (type)
        {
        case crackitos_core::math::ShapeType::kAABB:
            {
                crackitos_core::commons::fp half_size_x = crackitos_core::random::Range(5.f, 20.f);
                crackitos_core::commons::fp half_size_y = crackitos_core::random::Range(5.f, 20.f);
                crackitos_core::math::AABB aabb(pos, crackitos_core::math::Vec2f(half_size_x, half_size_y),
                                crackitos_core::math::Vec2f(half_size_x, half_size_y).Magnitude());

                physics::Body body_def(physics::BodyType::Dynamic, pos, crackitos_core::math::Vec2f(0, 0),
                                       true, crackitos_core::random::Range(50.f, 100.f));
                body = physics_world_.CreateBody(body_def);

                physics::Collider collider_def(aabb, crackitos_core::random::Range(0.0f, 0.0f), 0.5f, false, body);
                physics_world_.CreateCollider(body, collider_def);
                break;
            }
        case crackitos_core::math::ShapeType::kCircle:
            {
                crackitos_core::commons::fp radius = crackitos_core::random::Range(5.f, 20.f);
                crackitos_core::math::Circle circle(pos, radius);

                physics::Body body_def(physics::BodyType::Dynamic, pos, crackitos_core::math::Vec2f(0, 0),
                                       true, crackitos_core::random::Range(50.f, 100.f));
                body = physics_world_.CreateBody(body_def);

                physics::Collider collider_def(circle, crackitos_core::random::Range(0.2f, 0.8f), 0.2f, false, body);
                physics_world_.CreateCollider(body, collider_def);
                break;
            }
        case crackitos_core::math::ShapeType::kPolygon:
        case crackitos_core::math::ShapeType::kNone:
        default:
            return; // Do nothing
        }
    }

    void FrictionSystem::CreateGround()
    {
        crackitos_core::math::AABB ground(crackitos_core::math::Vec2f(360.0f, 650.0f), crackitos_core::math::Vec2f(840.0f, 750.0f));

        physics::Body body_def(physics::BodyType::Static, ground.GetCentre(), crackitos_core::math::Vec2f(0, 0), false, 0.0f);
        physics::BodyHandle body = physics_world_.CreateBody(body_def);

        physics::Collider collider_def(ground, 0.0f, 0.0f, false, body);
        physics_world_.CreateCollider(body, collider_def);
    }

    void FrictionSystem::Update(const crackitos_core::commons::fp delta_time)
    {
        physics_world_.FixedUpdate(delta_time);
        UpdateTestingObjects();
    }

    //Set Gravity Dynamically
    void FrictionSystem::SetGravity(const crackitos_core::math::Vec2f& gravity)
    {
        physics_world_.set_gravity(gravity);
    }

    void FrictionSystem::UpdateTestingObjects()
    {
        friction_objects_.clear(); // Clear previous frame's objects

        auto bodies_with_colliders = physics_world_.GetBodiesWithColliders();

        for (const auto& [body_handle, collider_handle] : bodies_with_colliders)
        {
            FrictionObject obj;
            obj.body = body_handle;
            obj.collider = collider_handle;
            obj.color = SDL_Color{255, 13, 132, 255};

            friction_objects_.push_back(obj);
        }
    }
} // namespace samples
