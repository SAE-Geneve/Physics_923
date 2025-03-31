#ifndef PHYSICS_SAMPLES_FRICTION_SYSTEM_H_
#define PHYSICS_SAMPLES_FRICTION_SYSTEM_H_

#include <unordered_map>
#include <unordered_set>

#include "display.h"
#include "game_object.h"
#include "quadtree.h"
#include "shape.h"
#include "timer.h"
#include "distance.h"

namespace crackitos_physics::samples {
class FrictionSystem {
 private:
  std::vector<GameObject> objects_;

  physics::Quadtree quadtree_;

  std::unordered_map<GameObjectPair, bool> potential_pairs_;
  std::unordered_set<GameObjectPair> active_pairs_;

  std::unordered_map<physics::Collider *, GameObject *> collider_to_object_map_;
  //Mapping from Collider to GameObject

  //Create the gravity for the scene
  crackitos_core::distance::Meter gravity_in_meter_x = crackitos_core::distance::Meter(0.0f);
  crackitos_core::distance::Meter gravity_in_meter_y = crackitos_core::distance::Meter(0.9f);
  const crackitos_core::math::Vec2f
      gravity = crackitos_core::math::Vec2f(static_cast<float>(crackitos_core::distance::Convert<crackitos_core::distance::Meter, crackitos_core::distance::Pixel>(
                                gravity_in_meter_x).value),
                            static_cast<float>(crackitos_core::distance::Convert<crackitos_core::distance::Meter,
                                                                 crackitos_core::distance::Pixel>(gravity_in_meter_y).value));

  crackitos_core::timer::Timer *timer_ = nullptr;
  crackitos_core::math::AABB frame_bounds_ = crackitos_core::math::AABB(crackitos_core::math::Vec2f(0, 0), crackitos_core::math::Vec2f(kWindowWidth, kWindowHeight));

 public:
  FrictionSystem();
  ~FrictionSystem();

  void Initialize();
  void Clear();

  std::vector<GameObject> objects() { return objects_; }
  [[nodiscard]] physics::Quadtree &quadtree() { return quadtree_; }

  void SpawnShape(crackitos_core::math::Vec2f pos, crackitos_core::math::ShapeType type);
  void CreateObject(size_t index, crackitos_core::math::Circle &circle);
  void CreateObject(size_t index, crackitos_core::math::AABB &aabb);
  //void CreateObject(size_t index, crackitos_core::math::Polygon& polygon);
  void CreateGround();
  void DeleteObject(size_t index);
  void RemoveOutOfBoundsObjects();

  void RegisterObject(GameObject &object);
  void UnregisterObject(GameObject &object);

  void Update(crackitos_core::commons::fp delta_time);
  void UpdateShapes(crackitos_core::commons::fp delta_time);

  void SimplisticBroadPhase();
  void BroadPhase();
  void NarrowPhase();

  static void OnPairCollideStart(const GameObjectPair &pair);
  static void OnPairCollideStay(const GameObjectPair &pair);
  static void OnPairCollideEnd(const GameObjectPair &pair);
};
} // namespace samples
#endif // PHYSICS_SAMPLES_FRICTION_SYSTEM_H_
