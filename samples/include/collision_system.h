#ifndef PHYSICS_SAMPLES_COLLISION_SYSTEM_H_
#define PHYSICS_SAMPLES_COLLISION_SYSTEM_H_

#include <unordered_map>

#include "game_object.h"
#include "quadtree.h"
#include <unordered_set>

#include "contact_solver.h"
#include "contact_listener.h"
#include "physics_world.h"

namespace crackitos_physics::samples {

//For testing purposes, this would be an implementation specific GameObject
struct CollisionObject {
  physics::BodyHandle body;
  physics::ColliderHandle collider;
  SDL_Color color;
};

class CollisionContactListener final : public physics::ContactListener
{
 public:
  void OnTriggerEnter(const physics::ColliderPair& pair) override;
  void OnTriggerStay(const physics::ColliderPair& pair) override;
  void OnTriggerExit(const physics::ColliderPair& pair) override;

  void OnCollisionEnter(const physics::ColliderPair& pair) override;
  void OnCollisionStay(const physics::ColliderPair& pair) override;
  void OnCollisionExit(const physics::ColliderPair& pair) override;
};

class CollisionSystem {
 private:

  physics::PhysicsWorld physics_world_;
  CollisionContactListener contact_listener_;

  int number_of_objects_ = 200;
  std::vector<CollisionObject> objects_ = {};

  physics::Quadtree quadtree_;

  std::unordered_map<GameObjectPair, bool> potential_pairs_;
  std::unordered_set<GameObjectPair> active_pairs_;

  //Mapping from Collider to GameObject
  std::unordered_map<physics::Collider *, GameObject *> collider_to_object_map_;

 public:
  CollisionSystem();
  ~CollisionSystem() = default;

  void Initialize();
  void Clear();

  std::vector<CollisionObject> objects() { return objects_; }
  [[nodiscard]] physics::Quadtree &quadtree() { return quadtree_; }

  void CreateObject(const math::Vec2f& pos, math::ShapeType type);

  void Update(crackitos_physics::commons::fp delta_time);

  void UpdateCollisionObject();

  [[nodiscard]] const std::vector<CollisionObject>& collision_objects() const { return objects_; }
  [[nodiscard]] physics::PhysicsWorld& physics_world() { return physics_world_; }
  [[nodiscard]] const physics::PhysicsWorld& physics_world() const { return physics_world_; } // Const version


};
} // namespace samples
#endif // PHYSICS_SAMPLES_COLLISION_SYSTEM_H_
