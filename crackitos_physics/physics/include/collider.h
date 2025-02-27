#ifndef CRACKITOS_PHYSICS_PHYSICS_COLLIDER_H_
#define CRACKITOS_PHYSICS_PHYSICS_COLLIDER_H_

#include <variant>

#include "shape.h"
#include "commons.h"

namespace crackitos_physics::physics
{
 class Collider
 {
 private:
  std::variant<math::Circle, math::AABB, math::Polygon> shape_ = math::Circle(0);

  crackitos_physics::commons::fp bounciness_ = 0.0f;
  crackitos_physics::commons::fp friction_ = 0.0f;
  crackitos_physics::commons::fp dynamic_friction_ = 0.0f;
  crackitos_physics::commons::fp offset_ = 0.0f;

  bool is_trigger_ = false;

  public:
  Collider() = default;
  Collider(const std::variant<math::Circle, math::AABB, math::Polygon>& shape, const crackitos_physics::commons::fp bounciness, const crackitos_physics::commons::fp friction, const bool is_trigger)
  {
   shape_ = shape;
   bounciness_ = bounciness;
   friction_ = friction;
   dynamic_friction_ = friction * 0.5f;
   is_trigger_ = is_trigger;
  };

  [[nodiscard]] std::variant<math::Circle, math::AABB, math::Polygon> shape() const { return shape_; }
  [[nodiscard]] crackitos_physics::commons::fp bounciness() const { return bounciness_; }
  [[nodiscard]] crackitos_physics::commons::fp friction() const { return friction_; }
  [[nodiscard]] crackitos_physics::commons::fp dynamic_friction() const { return dynamic_friction_; }
  [[nodiscard]] bool is_trigger() const { return is_trigger_; }

  void set_shape(const std::variant<math::Circle, math::AABB, math::Polygon>& shape){ shape_ = shape; }
  void set_bounciness(const crackitos_physics::commons::fp restitution){ bounciness_ = restitution; }
  void set_friction(const crackitos_physics::commons::fp friction){ friction_ = friction; }
  void set_is_trigger(const bool is_trigger){ is_trigger_ = is_trigger; }

  [[nodiscard]] math::AABB GetBoundingBox() const {
   return std::visit([](auto&& shape) {
       return shape.GetBoundingBox();
   }, shape_);
  }

  [[nodiscard]] math::ShapeType GetShapeType() const {
   return std::visit([](auto&& shape) {
       return shape.GetShapeType();
   }, shape_);
  }

  void UpdatePosition(const math::Vec2f position) {
   std::visit([&position](auto&& shape) {
       shape.UpdatePosition(position);
   }, shape_);
  }

  bool operator==(const Collider& other) const
  {
   return shape_ == other.shape_ &&
          bounciness_ == other.bounciness_ &&
          friction_ == other.friction_ &&
          offset_ == other.offset_ &&
          is_trigger_ == other.is_trigger_;
  }
 };

} // namespace physics

#endif // CRACKITOS_PHYSICS_PHYSICS_COLLIDER_H_
