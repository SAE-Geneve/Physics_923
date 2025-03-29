#include "trigger_system.h"

#include <ranges>
#include <iostream>

#include "contact_solver.h"
#include "display.h"
#include "random.h"

namespace crackitos_physics::samples {

//Contact Listener Implementations
void TriggerContactListener::OnTriggerEnter(const physics::ColliderPair& pair)
{
  // std::cout << "Trigger Enter: " << pair.colliderA.id << " and " << pair.colliderB.id << "\n";
}
void TriggerContactListener::OnTriggerStay(const physics::ColliderPair& pair)
{
  // std::cout << "Trigger Stay: " << pair.colliderA.id << " and " << pair.colliderB.id << "\n";
}
void TriggerContactListener::OnTriggerExit(const physics::ColliderPair& pair)
{
  // std::cout << "Trigger Exit: " << pair.colliderA.id << " and " << pair.colliderB.id << "\n";
}
void TriggerContactListener::OnCollisionEnter(const physics::ColliderPair& pair)
{
  // std::cout << "Collision Enter: " << pair.colliderA.id << " and " << pair.colliderB.id << "\n";
}
void TriggerContactListener::OnCollisionStay(const physics::ColliderPair& pair)
{
  // std::cout << "Collision Stay: " << pair.colliderA.id << " and " << pair.colliderB.id << "\n";
}
void TriggerContactListener::OnCollisionExit(const physics::ColliderPair& pair)
{
  // std::cout << "Collision Exit: " << pair.colliderA.id << " and " << pair.colliderB.id << "\n";
}

TriggerSystem::TriggerSystem(){
  physics_world_.SetContactListener(&contact_listener_);

}

TriggerSystem::~TriggerSystem() {
  Clear();
}

void TriggerSystem::Initialize() {

  //Have to modifie this to do something
  constexpr auto system_gravity = math::Vec2f(0.0f, 0.0f);

  Clear();
  physics_world_.Initialize(math::AABB(math::Vec2f(0, 0), math::Vec2f(kWindowWidth, kWindowHeight)), true, system_gravity);

  constexpr crackitos_physics::commons::fp margin = 20.0f;

  for (size_t i = 0; i < number_of_objects_ / 2 - 1; i++) {
    const math::Vec2f position(random::Range(margin, kWindowWidth - margin),
                               random::Range(margin, kWindowHeight - margin));
    const crackitos_physics::commons::fp radius = random::Range(5.f, 10.f);
    math::Circle circle(position, radius);
    CreateObject(position, math::ShapeType::kCircle);
  }
  for (size_t i = number_of_objects_ / 2 - 1; i < number_of_objects_; i++) {
    const math::Vec2f position(random::Range(margin, kWindowWidth - margin),
                               random::Range(margin, kWindowHeight - margin));
    math::Vec2f half_size_vec = math::Vec2f(random::Range(5.f, 10.f), random::Range(5.f, 10.f));
    const auto half_size_length = half_size_vec.Magnitude();

    math::AABB aabb(position, half_size_vec, half_size_length);
    CreateObject(position, math::ShapeType::kAABB);
  }
}

void TriggerSystem::Clear() {
  physics_world_.Clear();
}

void TriggerSystem::CreateObject(const math::Vec2f& pos, math::ShapeType type){
  physics::BodyHandle body;

  switch (type)
  {
    case math::ShapeType::kAABB:
    {
      math::Vec2f velocity(random::Range(-50.0f, 50.0f), random::Range(-50.0f, 50.0f));

      float half_size_x = random::Range(5.f, 20.f);
      float half_size_y = random::Range(5.f, 20.f);

      math::AABB aabb(pos, math::Vec2f(half_size_x, half_size_y),
                      math::Vec2f(half_size_x, half_size_y).Magnitude());

      physics::Body body_def(physics::BodyType::Dynamic, pos, velocity,
                             false, random::Range(50.f, 100.f));
      body = physics_world_.CreateBody(body_def);

      physics::Collider collider_def(aabb, random::Range(0.0f, 0.0f), 0.5f, false, body);
      physics_world_.CreateCollider(body, collider_def);
      break;
    }
    case math::ShapeType::kCircle:
    {
      math::Vec2f velocity(random::Range(-50.0f, 50.0f), random::Range(-50.0f, 50.0f));

      float radius = random::Range(5.f, 20.f);
      math::Circle circle(pos, radius);

      physics::Body body_def(physics::BodyType::Dynamic, pos, velocity, false, random::Range(1.0f, 50.f));
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

void TriggerSystem::Update(const crackitos_physics::commons::fp delta_time) {
  physics_world_.Update(delta_time);
  UpdateTriggerObjects();
}

void TriggerSystem::UpdateTriggerObjects()
{
  objects_.clear(); // Clear previous frame's objects

  auto bodies_with_colliders = physics_world_.GetBodiesWithColliders();

  for (const auto& [body_handle, collider_handle] : bodies_with_colliders) {
    TriggerObject obj;
    obj.body = body_handle;
    obj.collider = collider_handle;
    obj.color = SDL_Color{255, 13, 132, 255};

    objects_.push_back(obj);
  }
}
} // namespace samples
