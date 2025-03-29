#ifndef PHYSICS_SAMPLES_TRIGGER_SYSTEM_H_
#define PHYSICS_SAMPLES_TRIGGER_SYSTEM_H_

#include <unordered_map>
#include <unordered_set>

#include "physics_world.h"
#include "game_object.h"
#include "quadtree.h"
#include "contact_listener.h"

namespace crackitos_physics::samples {

//For testing purposes, this would be an implementation specific GameObject
struct TriggerObject {
  physics::BodyHandle body;
  physics::ColliderHandle collider;
  SDL_Color color;
};

class TriggerContactListener final : public physics::ContactListener
{
 public:
  void OnTriggerEnter(const physics::ColliderPair& pair) override;
  void OnTriggerStay(const physics::ColliderPair& pair) override;
  void OnTriggerExit(const physics::ColliderPair& pair) override;

  void OnCollisionEnter(const physics::ColliderPair& pair) override;
  void OnCollisionStay(const physics::ColliderPair& pair) override;
  void OnCollisionExit(const physics::ColliderPair& pair) override;
};

class TriggerSystem{
 private:

  physics::PhysicsWorld physics_world_;
  TriggerContactListener contact_listener_;

  //had to lower it, my pc couldn't handle 400
  int number_of_objects_ = 100;
  std::vector<TriggerObject> objects_ = {};

 public:
  TriggerSystem();
  ~TriggerSystem();

  void Initialize();
  void Clear();

  void CreateObject(const math::Vec2f& pos, math::ShapeType type);

  void Update(crackitos_physics::commons::fp delta_time);

  void UpdateTriggerObjects();

  [[nodiscard]] const std::vector<TriggerObject>& trigger_objects() const { return objects_; }
  [[nodiscard]] physics::PhysicsWorld& physics_world() { return physics_world_; }
  [[nodiscard]] const physics::PhysicsWorld& physics_world() const { return physics_world_; } // Const version
};
} // namespace samples
#endif // PHYSICS_SAMPLES_TRIGGER_SYSTEM_H_
