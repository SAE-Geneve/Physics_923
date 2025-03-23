#ifndef CRACKITOS_PHYSICS_PHYSICS_CONTACT_SOLVER_H_
#define CRACKITOS_PHYSICS_PHYSICS_CONTACT_SOLVER_H_

#include <array>
#include <complex>
#include <iostream>
#include <utility>

#include "shape.h"
#include "vec2.h"
#include "commons.h"

namespace crackitos_physics::physics
{
    constexpr commons::fp kPenetrationEpsilon = 0.0001f;
    constexpr commons::fp kSleepVelocityThreshold = 0.5f;

    struct ContactSolver
    {
        std::pair<Body*, Body*> bodies_{};
        std::pair<Collider*, Collider*> colliders_{};

        math::Vec2f contact_point_ = math::Vec2f::Zero();
        math::Vec2f contact_normal_ = math::Vec2f::Zero();

        commons::fp restitution_ = 0.0f;
        commons::fp penetration_ = 0.0f;

        void SetContactObjects(const std::pair<Body*, Body*> bodies, const std::pair<Collider*, Collider*> colliders)
        {
            bodies_ = bodies;
            colliders_ = colliders;
        }

        void SwapObjects()
        {
            std::swap(bodies_.first, bodies_.second);
            std::swap(colliders_.first, colliders_.second);
        }

        void ResolveContact()
        {
            CalculateProperties();
            if (bodies_.first->type() == BodyType::Static || bodies_.first->type() == BodyType::Kinematic)
            {
                SwapObjects();
                contact_normal_ = -contact_normal_;
            }
            ResolveVelocities();
            ResolvePositions();
        }

        void CalculateProperties()
        {
            const auto collider_a = colliders_.first;
            const auto collider_b = colliders_.second;

            contact_point_ = math::Vec2f::Zero();
            contact_normal_ = math::Vec2f::Zero();
            penetration_ = 0.0f;

            // restitution_ = std::max(collider_a->bounciness(), collider_b->bounciness());

            const auto mA = bodies_.first->mass();
            const auto mB = bodies_.second->mass();
            const auto eA = collider_a->bounciness();
            const auto eB = collider_b->bounciness();

            restitution_ = (mA * eA + mB * eB) / (mA + mB);


            const auto type_a = collider_a->GetShapeType();
            const auto type_b = collider_b->GetShapeType();

            // Handle each pair only once, and swap if needed
            if (type_a == math::ShapeType::kAABB && type_b == math::ShapeType::kAABB)
            {
                HandleAABBAABBCollision();
            }
            else if (type_a == math::ShapeType::kAABB && type_b == math::ShapeType::kCircle)
            {
                HandleAABBCircleCollision(*collider_a, *collider_b);
            }
            else if (type_a == math::ShapeType::kCircle && type_b == math::ShapeType::kAABB)
            {
                // Swap so AABB is first (handler expects that)
                SwapObjects();
                HandleAABBCircleCollision(*colliders_.first, *colliders_.second);
                contact_normal_ = -contact_normal_;
            }
            else if (type_a == math::ShapeType::kCircle && type_b == math::ShapeType::kCircle)
            {
                HandleCircleCircleCollision();
            }
            else if (type_a == math::ShapeType::kPolygon && type_b == math::ShapeType::kPolygon)
            {
                HandlePolygonPolygonCollision();
            }
            else if (type_a == math::ShapeType::kAABB && type_b == math::ShapeType::kPolygon)
            {
                HandleAABBPolygonCollision();
            }
            else if (type_a == math::ShapeType::kPolygon && type_b == math::ShapeType::kAABB)
            {
                SwapObjects();
                HandleAABBPolygonCollision(); // Assuming this expects AABB as first
                contact_normal_ = -contact_normal_;
            }
            else if (type_a == math::ShapeType::kCircle && type_b == math::ShapeType::kPolygon)
            {
                HandleCirclePolygonCollision();
            }
            else if (type_a == math::ShapeType::kPolygon && type_b == math::ShapeType::kCircle)
            {
                SwapObjects();
                HandleCirclePolygonCollision(); // Assuming this expects Circle as first
                contact_normal_ = -contact_normal_;
            }
            else
            {
                // Unhandled or ShapeType::kNone
                penetration_ = 0.0f;
            }
        }


        void ResolveVelocities() const
        {
            auto& body_a = bodies_.first;
            auto& body_b = bodies_.second;

            const auto& collider_a = colliders_.first;
            const auto& collider_b = colliders_.second;

            const auto inverse_mass_a = body_a->inverse_mass();
            const auto inverse_mass_b = body_b->inverse_mass();
            const auto total_inverse_mass = inverse_mass_a + inverse_mass_b;
            if (total_inverse_mass == 0.0f)
            {
                return;
            }


            // Calculate the relative velocity between the bodies
            const math::Vec2f relative_velocity = body_a->velocity() - body_b->velocity();

            // Relative velocity along Normal
            const commons::fp separating_velocity = math::Vec2f::Dot(
                relative_velocity, contact_normal_);

            if (separating_velocity > 0.0f) { return; }

            const auto delta_velocity = -separating_velocity * restitution_ - separating_velocity;

            const auto impulse = contact_normal_ * (delta_velocity / total_inverse_mass);

            if (body_a->type() == BodyType::Dynamic)
            {
                body_a->ApplyImpulse(impulse);
                if (body_a->velocity().SquareMagnitude() < kSleepVelocityThreshold)
                    body_a->set_velocity(math::Vec2f::Zero());
            }
            if (body_b->type() == BodyType::Dynamic)
            {
                body_b->ApplyImpulse(-impulse);
                if (body_b->velocity().SquareMagnitude() < kSleepVelocityThreshold)
                    body_b->set_velocity(math::Vec2f::Zero());
            }

            /*
            //Friction
            //Tangent vector
            math::Vec2f tangent = (relative_velocity - separating_velocity * contact_normal_);
            if (tangent.SquareMagnitude() > std::numeric_limits<crackitos_physics::commons::fp>::epsilon())
            {
                tangent = tangent.Normalized();
            }
            else
            {
                tangent = math::Vec2f::Zero(); // No friction applied
            }

            // Magnitude to apply
            const crackitos_physics::commons::fp jt = -math::Vec2f::Dot(relative_velocity, tangent) / (body_a->
                inverse_mass() + body_b->inverse_mass());
            const crackitos_physics::commons::fp mu = std::sqrt(
                collider_a->friction() * collider_a->friction() + collider_b->friction() * collider_b->friction());

            // Clamp friction
            math::Vec2f friction_impulse = math::Vec2f::Zero();
            if (std::abs(jt) < impulse_magnitude * mu)
            {
                friction_impulse = jt * tangent;
            }
            else
            {
                const crackitos_physics::commons::fp dynamic_friction_impulse = std::sqrt(
                    collider_a->dynamic_friction() * collider_a->dynamic_friction() + collider_b->dynamic_friction() *
                    collider_b->dynamic_friction());
                friction_impulse = -impulse_magnitude * tangent * dynamic_friction_impulse;
            }

            // Apply impulse to the dynamic bodies
            if (body_a->type() == BodyType::Dynamic)
            {
                body_a->ApplyImpulse(friction_impulse);
            }
            if (body_b->type() == BodyType::Dynamic)
            {
                body_b->ApplyImpulse(-friction_impulse);
            }
            */
        }

        void ResolvePositions() const
        {
            // if (penetration_ <= 0.01f) { return; } ?
            if (penetration_ <= kPenetrationEpsilon) { return; }

            auto& body_a = bodies_.first;
            auto& body_b = bodies_.second;
            const auto inverse_mass_a = body_a->inverse_mass();
            const auto inverse_mass_b = body_b->inverse_mass();
            const auto total_inverse_mass = inverse_mass_a + inverse_mass_b;
            if (total_inverse_mass == 0.0f)
            {
                return;
            }

            // Find the amount of penetration resolution per unit of inverse mass.
            const auto move_per_inverse_mass = contact_normal_ * (penetration_ / total_inverse_mass);
            // Apply the penetration resolution.

            if (body_a->type() == BodyType::Dynamic)
            {
                body_a->set_position(body_a->position() + move_per_inverse_mass * inverse_mass_a);
            }
            if (body_b->type() == BodyType::Dynamic)
            {
                body_b->set_position(body_b->position() - move_per_inverse_mass * inverse_mass_b);
            }
        }

        void HandleAABBAABBCollision()
        {
            const auto& aabb_a = std::get<math::AABB>(colliders_.first->shape());
            const auto& aabb_b = std::get<math::AABB>(colliders_.second->shape());
            const auto centre_a = bodies_.first->position();
            const auto centre_b = bodies_.second->position();

            const auto delta = centre_a - centre_b;

            contact_point_ = 0.5f * (aabb_a.ClosestPoint(centre_b) + aabb_b.ClosestPoint(centre_a));


            const auto half_size_a = aabb_a.half_size_vec();
            const auto half_size_b = aabb_b.half_size_vec();

            //TODO own function abs, vec2f right, left, up, down
            const auto penetration_x = half_size_a.x + half_size_b.x - std::abs(delta.x);
            const auto penetration_y = half_size_a.y + half_size_b.y - std::abs(delta.y);

            if (penetration_x < penetration_y)
            {
                contact_normal_ = delta.x > 0.0f ? math::Vec2f(1.0f, 0.0f) : math::Vec2f(-1.0f, 0.0f);
                penetration_ = penetration_x;
            }
            else
            {
                contact_normal_ = delta.y > 0.0f ? math::Vec2f(0.0f, 1.0f) : math::Vec2f(0.0f, -1.0f);
                penetration_ = penetration_y;
            }
        }

        void HandleAABBCircleCollision(const Collider& AABB, const Collider& Circle)
        {
            const auto& aabb = std::get<math::AABB>(AABB.shape());
            const auto& circle = std::get<math::Circle>(Circle.shape());
            const auto circle_centre = circle.centre();
            const auto aabb_centre = aabb.GetCentre();
            const auto aabb_half_size = aabb.half_size_vec();
            const auto radius = circle.radius();

            // const auto delta = circle_centre - aabb_centre;
            //
            // // Find the closest point on the AABB to the circle center
            // const math::Vec2f closest_point = {
            //     std::clamp(delta.x, -aabb_half_size.x, aabb_half_size.x),
            //     std::clamp(delta.y, -aabb_half_size.y, aabb_half_size.y)
            // };


            // Find the closest point on the AABB to the circle center
            const math::Vec2f closest_point = {
                std::clamp(circle_centre.x, aabb.min_bound().x, aabb.max_bound().x),
                std::clamp(circle_centre.y, aabb.min_bound().y, aabb.max_bound().y)
            };

            const math::Vec2f distance_vec = closest_point - circle_centre;
            const commons::fp distance_squared = distance_vec.SquareMagnitude();
            const commons::fp radius_squared = radius * radius;

            if (distance_squared >= radius_squared)
            {
                penetration_ = 0.0f;
                return; // No collision
            }

            const commons::fp distance = std::sqrt(distance_squared);

            // Circle is outside the AABB
            contact_normal_ = distance > commons::kEpsilon
                                  ? distance_vec / distance
                                  : math::Vec2f(0.0f, 1.0f);
            penetration_ = radius - distance;

            contact_point_ = closest_point;

            // Closest point on the AABB (in local space)
            // const math::Vec2f closest_point = {
            //     std::clamp(delta.x, -aabb_half_size.x, aabb_half_size.x),
            //     std::clamp(delta.y, -aabb_half_size.y, aabb_half_size.y)
            // };
            //
            // // Convert to world space
            // const auto closest_point_on_aabb = aabb_centre + closest_point;
            // const auto circle_to_aabb = circle_centre - closest_point_on_aabb;
            // const auto distance = circle_to_aabb.Magnitude();
            //
            // // ✅ Early out if there's no collision
            // if (distance >= radius)
            // {
            //     penetration_ = 0.0f;
            //     return;
            // }
            //
            // math::Vec2f normal = circle_to_aabb;
            // if (normal.SquareMagnitude() <= commons::kEpsilon)
            // {
            //     normal = math::Vec2f(0.0f, 1.0f); // Default upward
            // }
            //
            // contact_normal_ = normal.Normalized();
            // contact_point_ = closest_point_on_aabb;
            // penetration_ = radius - distance;


            //  {
            // const auto& aabb = std::get<math::AABB>(AABB.shape());
            // const auto& circle = std::get<math::Circle>(Circle.shape());
            // const auto centre = circle.centre();
            // const auto radius = circle.radius();
            //

            //
            // // Calculate the vector from the circle center to the closest point
            // const math::Vec2f delta = closest_point - centre;
            // const commons::fp distance_squared = delta.SquareMagnitude();
            // const commons::fp radius_squared = radius * radius;
            //
            // if (distance_squared >= radius_squared)
            // {
            //     penetration_ = 0.0f;
            //     return; // No collision
            // }
            //
            // const commons::fp distance = delta.Magnitude();
            //
            // if (distance > std::numeric_limits<commons::fp>::epsilon())
            // {
            //     // Case: Circle's center is outside the AABB
            //     contact_normal_ = delta / distance; // Normalized vector
            //     penetration_ = radius - distance;
            //     contact_point_ = closest_point;
            // }
            // else
            // {
            //     // Case: Circle's center is inside the AABB
            //     // Find the minimum distance to an edge of the AABB
            //     commons::fp left_dist = std::abs(centre.x - aabb.min_bound().x);
            //     commons::fp right_dist = std::abs(aabb.max_bound().x - centre.x);
            //     commons::fp top_dist = std::abs(aabb.min_bound().y - centre.y);
            //     commons::fp bottom_dist = std::abs(aabb.max_bound().y - centre.y);
            //
            //     // Determine the edge with the smallest distance
            //     commons::fp min_dist = std::min({left_dist, right_dist, top_dist, bottom_dist});
            //
            //     if (min_dist == left_dist)
            //     {
            //         contact_normal_ = math::Vec2f(-1, 0); // Left edge
            //         penetration_ = radius - left_dist;
            //         contact_point_ = {aabb.min_bound().x, centre.y};
            //     }
            //     else if (min_dist == right_dist)
            //     {
            //         contact_normal_ = math::Vec2f(1, 0); // Right edge
            //         penetration_ = radius - right_dist;
            //         contact_point_ = {aabb.max_bound().x, centre.y};
            //     }
            //     else if (min_dist == bottom_dist)
            //     {
            //         contact_normal_ = math::Vec2f(0, 1); // Bottom
            //         penetration_ = radius - bottom_dist;
            //         contact_point_ = {centre.x, aabb.max_bound().y};
            //     }
            //     else //default to top
            //     {
            //         contact_normal_ = math::Vec2f(0, -1); // Top
            //         penetration_ = radius - top_dist;
            //         contact_point_ = {centre.x, aabb.min_bound().y};
            //     }
            // }
        }


        void HandleAABBPolygonCollision()
        {
            //Not Implemented
        }

        void HandleCircleCircleCollision()
        {
            const auto radius_a = std::get<math::Circle>(colliders_.first->shape()).radius();
            const auto radius_b = std::get<math::Circle>(colliders_.second->shape()).radius();
            const auto centre_a = std::get<math::Circle>(colliders_.first->shape()).centre();
            const auto centre_b = std::get<math::Circle>(colliders_.second->shape()).centre();

            //Calculate the vector between the centers
            const math::Vec2f delta = centre_a - centre_b;
            commons::fp dist = delta.Magnitude();
            commons::fp total_radius = radius_a + radius_b;

            if (dist >= total_radius)
            {
                penetration_ = 0.0f;
                return; // No collision
            }

            if (dist < std::numeric_limits<commons::fp>::epsilon())
            {
                contact_normal_ = math::Vec2f(1.0f, 0.0f); // Arbitrary normal
                penetration_ = total_radius; // Total overlap
            }
            else
            {
                contact_normal_ = delta / dist;
                penetration_ = total_radius - dist;
            }

            contact_point_ = centre_a + contact_normal_ * radius_a;
        }

        void HandleCirclePolygonCollision()
        {
            //Not Implemented
            //This would involve checking the closest edge of the polygon to the circle and calculating the contact point, normal, and penetration depth
        }

        void HandlePolygonPolygonCollision()
        {
            //Not Implemented
            //This would involve checking for edge intersections and calculating the contact point, normal, and penetration depth
        }
    };
} // namespace physics
#endif // CRACKITOS_PHYSICS_PHYSICS_CONTACT_SOLVER_H_
