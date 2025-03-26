#ifndef PHYSICS_SAMPLES_TRIGGER_SYSTEM_H_
#define PHYSICS_SAMPLES_TRIGGER_SYSTEM_H_

#include <unordered_map>
#include <unordered_set>

#include "game_object.h"
#include "quadtree.h"

namespace crackitos_physics::samples {

//For testing purposes, this would be an implementation specific GameObject
struct TestingObject {
  physics::BodyHandle body;
  physics::ColliderHandle collider;
  SDL_Color color;
};

class TestingContactListener final : public physics::ContactListener
{
 public:
  void OnTriggerEnter(const physics::ColliderPair& pair) override;
  void OnTriggerStay(const physics::ColliderPair& pair) override;
  void OnTriggerExit(const physics::ColliderPair& pair) override;

  void OnCollisionEnter(const physics::ColliderPair& pair) override;
  void OnCollisionStay(const physics::ColliderPair& pair) override;
  void OnCollisionExit(const physics::ColliderPair& pair) override;
};

class TriggerSystem final{
 private:

  physics::PhysicsWorld physics_world_;
  TestingContactListener contact_listener_;

  std::vector<TestingObject> testing_objects_;

  int number_of_objects_ = 400;
  std::array<GameObject, 400> objects_ = {};
  physics::Quadtree quadtree_;

  std::unordered_map<GameObjectPair, bool> potential_pairs_;
  std::unordered_set<GameObjectPair> active_pairs_;

  //Mapping from Collider to GameObject
  std::unordered_map<physics::Collider *, GameObject *> collider_to_object_map_;

 public:
  TriggerSystem();
  ~TriggerSystem();

  void Initialize();
  void Clear();

  std::array<GameObject, 400> objects() { return objects_; }
  [[nodiscard]] physics::Quadtree &quadtree() { return quadtree_; }

  void CreateObject(size_t index, math::Circle &circle);
  void CreateObject(size_t index, math::AABB &aabb);
  //void CreateObject(size_t index, math::Polygon& polygon);
  void DeleteObject(size_t index);

  void RegisterObject(GameObject &object);
  void UnregisterObject(GameObject &object);

  void Update(crackitos_physics::commons::fp delta_time);
  void UpdateShapes(crackitos_physics::commons::fp delta_time);

  void UpdateTestingObjects();
  [[nodiscard]] const std::vector<TestingObject>& testing_objects() const { return testing_objects_; }

  [[nodiscard]] physics::PhysicsWorld& physics_world() { return physics_world_; }
  [[nodiscard]] const physics::PhysicsWorld& physics_world() const { return physics_world_; } // Const version

  void SimplisticBroadPhase();
  void BroadPhase();
  void NarrowPhase();

  static void OnPairCollideStart(const GameObjectPair &pair);
  static void OnPairCollideStay(const GameObjectPair &pair);
  static void OnPairCollideEnd(const GameObjectPair &pair);
};
} // namespace samples
#endif // PHYSICS_SAMPLES_TRIGGER_SYSTEM_H_
