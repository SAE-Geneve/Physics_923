// #include "trigger_system.h"
//
// #include "contact_solver.h"
// #include "display.h"
// #include "random.h"
//
// namespace crackitos_physics::samples
// {
//     //Contact Listener Implementations
//     void TriggerContactListener::OnTriggerEnter(const physics::ColliderPair& pair)
//     {
//         // std::cout << "Trigger Enter: " << pair.colliderA.id << " and " << pair.colliderB.id << "\n";
//     }
//
//     void TriggerContactListener::OnTriggerStay(const physics::ColliderPair& pair)
//     {
//         // std::cout << "Trigger Stay: " << pair.colliderA.id << " and " << pair.colliderB.id << "\n";
//     }
//
//     void TriggerContactListener::OnTriggerExit(const physics::ColliderPair& pair)
//     {
//         // std::cout << "Trigger Exit: " << pair.colliderA.id << " and " << pair.colliderB.id << "\n";
//     }
//
//     void TriggerContactListener::OnCollisionEnter(const physics::ColliderPair& pair)
//     {
//         // std::cout << "Collision Enter: " << pair.colliderA.id << " and " << pair.colliderB.id << "\n";
//     }
//
//     void TriggerContactListener::OnCollisionStay(const physics::ColliderPair& pair)
//     {
//         // std::cout << "Collision Stay: " << pair.colliderA.id << " and " << pair.colliderB.id << "\n";
//     }
//
//     void TriggerContactListener::OnCollisionExit(const physics::ColliderPair& pair)
//     {
//         // std::cout << "Collision Exit: " << pair.colliderA.id << " and " << pair.colliderB.id << "\n";
//     }
//
//     TriggerSystem::TriggerSystem()
//     {
//         physics_world_.SetContactListener(&contact_listener_);
//         trigger_objects_.reserve(number_of_objects_);
//     }
//
//     TriggerSystem::~TriggerSystem()
//     {
//         Clear();
//     }
//
//     void TriggerSystem::Initialize()
//     {
//         Clear();
//         physics_world_.Initialize(math::AABB(math::Vec2f(0, 0), math::Vec2f(kWindowWidth, kWindowHeight)), true);
//
//
//
//         for (size_t i = 0; i < number_of_objects_ / 2 - 1; i++)
//         {
//
//
//             CreateObject(math::ShapeType::kCircle);
//         }
//         for (size_t i = number_of_objects_ / 2 - 1; i < number_of_objects_; i++)
//         {
//
//             CreateObject(math::ShapeType::kAABB);
//         }
//
//         UpdateTriggerObjects();
//     }
//
//     void TriggerSystem::Clear()
//     {
//         trigger_objects_.clear();
//         physics_world_.Clear();
//     }
//
//     void TriggerSystem::CreateObject(math::ShapeType type)
//     {
//         constexpr commons::fp margin = 20.0f;
//         const math::Vec2f position(random::Range(margin, kWindowWidth - margin),
//                                        random::Range(margin, kWindowHeight - margin));
//         physics::BodyHandle body;
//
//         switch (type)
//         {
//         case math::ShapeType::kAABB:
//             {
//                 math::Vec2f velocity(random::Range(-50.0f, 50.0f), random::Range(-50.0f, 50.0f));
//                 math::Vec2f half_size_vec = math::Vec2f(random::Range(5.f, 10.f), random::Range(5.f, 10.f));
//                 math::AABB aabb(position, half_size_vec, half_size_vec.Magnitude());
//
//                 physics::Body body_def(physics::BodyType::Dynamic, position, velocity,
//                                        false, random::Range(1.f, 50.f));
//                 body = physics_world_.CreateBody(body_def);
//
//                 physics::Collider collider_def(aabb, random::Range(1.0f, 1.0f), 0.f, true, body);
//                 physics::ColliderHandle collider = physics_world_.CreateCollider(body, collider_def);
//                 trigger_objects_.push_back({ body, collider, SDL_Color{255, 13, 132, 255} });
//
//                 break;
//             }
//         case math::ShapeType::kCircle:
//             {
//                 math::Vec2f velocity(random::Range(-50.0f, 50.0f), random::Range(-50.0f, 50.0f));
//                 const commons::fp radius = random::Range(5.f, 10.f);
//                 math::Circle circle(position, radius);
//
//                 physics::Body body_def(physics::BodyType::Dynamic, position, velocity, false, random::Range(1.0f, 50.f));
//                 body = physics_world_.CreateBody(body_def);
//
//                 physics::Collider collider_def(circle, random::Range(1.0f, 1.0f), 0.0f, true, body);
//                 physics::ColliderHandle collider = physics_world_.CreateCollider(body, collider_def);
//                 trigger_objects_.push_back({ body, collider, SDL_Color{255, 13, 132, 255} });
//
//                 break;
//             }
//         case math::ShapeType::kPolygon:
//         case math::ShapeType::kNone:
//         default:
//             return; // Do nothing
//         }
//     }
//
//     void TriggerSystem::Update(const commons::fp delta_time)
//     {
//         physics_world_.Update(delta_time);
//     }
//
//     void TriggerSystem::UpdateTriggerObjects()
//     {
//         trigger_objects_.clear();
//         auto bodies_with_colliders = physics_world_.GetBodiesWithColliders();
//
//         for (const auto& [body_handle, collider_handle] : bodies_with_colliders)
//         {
//
//             TriggerObject obj;
//             obj.body = body_handle;
//             obj.collider = collider_handle;
//             obj.color = SDL_Color{255, 13, 132, 255};
//
//             trigger_objects_.push_back(obj);
//         }
//     }
// } // namespace samples

#include "trigger_system.h"

#include <algorithm>

#include "display.h"

namespace crackitos_physics::samples {

    TriggerSystem::TriggerSystem() {
        physics_world_.SetContactListener(&contact_listener_);
    }

    TriggerSystem::~TriggerSystem() {
        Clear();
    }

    void TriggerSystem::Initialize() {

        Clear();
        trigger_objects_.reserve(kMaxObjects);

        physics_world_.Initialize(math::AABB({0, 0}, {kWindowWidth, kWindowHeight}), true);

        for (int i = 0; i < kMaxObjects; ++i) {
            math::Vec2f pos = {
                random::Range(20.f, kWindowWidth - 20.f),
                random::Range(20.f, kWindowHeight - 20.f)
            };

            math::Vec2f vel = {
                random::Range(-100.f, 100.f),
                random::Range(-100.f, 100.f)
            };

            bool is_circle = (i % 2 == 0);
            physics::Body body(physics::BodyType::Dynamic, pos, vel, false, 10.f);
            physics::BodyHandle body_handle = physics_world_.CreateBody(body);

            physics::ColliderHandle collider_handle;
            if (is_circle) {
                math::Circle shape(pos, random::Range(5.f, 10.f));
                physics::Collider collider(shape, 0.f, 0.f, true, body_handle);
                collider_handle = physics_world_.CreateCollider(body_handle, collider);
            } else {
                math::Vec2f half_size = {random::Range(5.f, 10.f), random::Range(5.f, 10.f)};
                math::AABB shape(pos, half_size, half_size.Magnitude());
                physics::Collider collider(shape, 0.f, 0.f, true, body_handle);
                collider_handle = physics_world_.CreateCollider(body_handle, collider);
            }

            trigger_objects_.push_back({body_handle, collider_handle});
            contact_listener_.collider_to_object_[collider_handle.id] = &trigger_objects_.back();
        }
    }

    void TriggerSystem::Clear() {
        trigger_objects_.clear();
        physics_world_.Clear();
    }

    void TriggerSystem::Update(commons::fp delta_time) {
        // Bounce logic
        for (auto& obj : trigger_objects_) {
            auto& body = physics_world_.GetMutableBody(obj.body);
            auto pos = body.position();
            auto vel = body.velocity();
            const float radius = 10.0f; // approximate for both shape types

            if (pos.x - radius < 0.f) {
                pos.x = radius;
                vel.x *= -1.f;
            }
            if (pos.x + radius > kWindowWidth) {
                pos.x = kWindowWidth - radius;
                vel.x *= -1.f;
            }
            if (pos.y - radius < 0.f) {
                pos.y = radius;
                vel.y *= -1.f;
            }
            if (pos.y + radius > kWindowHeight) {
                pos.y = kWindowHeight - radius;
                vel.y *= -1.f;
            }

            body.set_position(pos);
            body.set_velocity(vel);
        }

        physics_world_.Update(delta_time);
    }

    void TriggerContactListener::OnTriggerEnter(const physics::ColliderPair& pair) {
        for (const auto& id : {pair.colliderA.id, pair.colliderB.id}) {
            auto it = collider_to_object_.find(id);
            if (it != collider_to_object_.end()) {
                it->second->OnTriggerEnter();
            }
        }
    }

    void TriggerContactListener::OnTriggerExit(const physics::ColliderPair& pair) {
        for (const auto& id : {pair.colliderA.id, pair.colliderB.id}) {
            auto it = collider_to_object_.find(id);
            if (it != collider_to_object_.end()) {
                it->second->OnTriggerExit();
            }
        }
    }
} // namespace crackitos_physics::samples
