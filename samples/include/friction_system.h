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
  distance::Meter gravity_in_meter_x = distance::Meter(0.0f);
  distance::Meter gravity_in_meter_y = distance::Meter(9.81f);
  const math::Vec2f
      gravity = math::Vec2f(static_cast<float>(distance::Convert<distance::Meter, distance::Pixel>(
                                gravity_in_meter_x).value),
                            static_cast<float>(distance::Convert<distance::Meter,
                                                                 distance::Pixel>(gravity_in_meter_y).value));

  timer::Timer *timer_ = nullptr;
  math::AABB frame_bounds_ = math::AABB(math::Vec2f(0, 0), math::Vec2f(kWindowWidth, kWindowHeight));

 public:
  FrictionSystem();
  ~FrictionSystem();

  void Initialize();
  void Clear();

  std::vector<GameObject> objects() { return objects_; }
  [[nodiscard]] physics::Quadtree &quadtree() { return quadtree_; }

  void SpawnShape(math::Vec2f pos, math::ShapeType type);
  void CreateObject(size_t index, math::Circle &circle);
  void CreateObject(size_t index, math::AABB &aabb);
  //void CreateObject(size_t index, math::Polygon& polygon);
  void CreateGround();
  void DeleteObject(size_t index);
  void RemoveOutOfBoundsObjects();

  void RegisterObject(GameObject &object);
  void UnregisterObject(GameObject &object);

  void Update(crackitos_physics::commons::fp delta_time);
  void UpdateShapes(crackitos_physics::commons::fp delta_time);

  void SimplisticBroadPhase();
  void BroadPhase();
  void NarrowPhase();
  void ResolveCollisionPair(const GameObjectPair& pair, bool is_new_pair);

  static void OnPairCollideEnd(const GameObjectPair &pair);
};
} // namespace samples
#endif // PHYSICS_SAMPLES_FRICTION_SYSTEM_H_
