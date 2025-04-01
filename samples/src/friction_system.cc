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
        constexpr auto system_gravity = math::Vec2f(
                static_cast<commons::fp>(distance::Convert<distance::Meter, distance::Pixel>(distance::Meter(0.f)).value),
                static_cast<commons::fp>(distance::Convert<distance::Meter, distance::Pixel>(distance::Meter(100.f)).value)
            );
        Clear();
        physics_world_.Initialize(math::AABB(math::Vec2f(0, 0), math::Vec2f(kWindowWidth, kWindowHeight)), true, system_gravity);
        CreateGround();
    }

    void FrictionSystem::Clear()
    {
        friction_objects_.clear();
        physics_world_.Clear();
    }


    void FrictionSystem::SpawnShape(const math::Vec2f pos, const math::ShapeType type)
    {
        physics::BodyHandle body;

        switch (type)
        {
        case math::ShapeType::kAABB:
            {
                float half_size_x = random::Range(5.f, 20.f);
                float half_size_y = random::Range(5.f, 20.f);
                math::AABB aabb(pos, math::Vec2f(half_size_x, half_size_y),
                                math::Vec2f(half_size_x, half_size_y).Magnitude());

                physics::Body body_def(physics::BodyType::Dynamic, pos, math::Vec2f(0, 0),
                                       true, random::Range(50.f, 100.f));
                body = physics_world_.CreateBody(body_def);

                physics::Collider collider_def(aabb, random::Range(0.0f, 0.0f), 0.5f, false, body);
                physics_world_.CreateCollider(body, collider_def);
                break;
            }
        case math::ShapeType::kCircle:
            {
                float radius = random::Range(5.f, 20.f);
                math::Circle circle(pos, radius);

                physics::Body body_def(physics::BodyType::Dynamic, pos, math::Vec2f(0, 0),
                                       true, random::Range(50.f, 100.f));
                body = physics_world_.CreateBody(body_def);

                physics::Collider collider_def(circle, random::Range(0.2f, 0.8f), 0.5f, false, body);
                physics_world_.CreateCollider(body, collider_def);
                break;
            }
        case math::ShapeType::kPolygon:
        case math::ShapeType::kNone:
        default:
            return; // Do nothing
        }
    }

    void FrictionSystem::CreateGround()
    {
        math::AABB ground(math::Vec2f(360.0f, 650.0f), math::Vec2f(840.0f, 750.0f));

        physics::Body body_def(physics::BodyType::Static, ground.GetCentre(), math::Vec2f(0, 0), false, 0.0f);
        physics::BodyHandle body = physics_world_.CreateBody(body_def);

        physics::Collider collider_def(ground, 0.0f, 0.0f, false, body);
        physics_world_.CreateCollider(body, collider_def);
    }

    void FrictionSystem::Update(const crackitos_physics::commons::fp delta_time)
    {
        physics_world_.Update(delta_time);
        UpdateTestingObjects();
    }

    //Set Gravity Dynamically
    void FrictionSystem::SetGravity(const math::Vec2f& gravity)
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
