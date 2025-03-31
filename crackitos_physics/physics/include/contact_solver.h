#ifndef CRACKITOS_PHYSICS_PHYSICS_CONTACT_SOLVER_H_
#define CRACKITOS_PHYSICS_PHYSICS_CONTACT_SOLVER_H_

#include <array>
#include <iostream>
#include <utility>

#include "shape.h"
#include "vec2.h"
#include "commons.h"

namespace crackitos_physics::physics
{
    struct ContactSolver
    {
        std::pair<Body*, Body*> bodies_{};
        std::pair<Collider*, Collider*> colliders_{};

        crackitos_core::math::Vec2f contact_point_ = crackitos_core::math::Vec2f::Zero();
        crackitos_core::math::Vec2f contact_normal_ = crackitos_core::math::Vec2f::Zero();

        crackitos_core::commons::fp penetration_ = 0.0f;

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
            if(bodies_.first->type() == BodyType::Static)
            {
                SwapObjects();
                contact_normal_ = -contact_normal_;
            }
            ResolveVelocities();
            ResolvePositions();
        }

    private:
        void CalculateProperties()
        {
            const auto collider_a = colliders_.first;
            const auto collider_b = colliders_.second;

            //Reset properties
            contact_point_ = crackitos_core::math::Vec2f::Zero();
            contact_normal_ = crackitos_core::math::Vec2f::Zero();
            penetration_ = 0.0f;

            switch (collider_a->GetShapeType())
            {
            case crackitos_core::math::ShapeType::kAABB:
                switch (collider_b->GetShapeType())
                {
                case crackitos_core::math::ShapeType::kAABB:
                    HandleAABBAABBCollision();
                    break;
                case crackitos_core::math::ShapeType::kCircle:
                    HandleAABBCircleCollision();
                    break;
                case crackitos_core::math::ShapeType::kPolygon:
                    HandleAABBPolygonCollision();
                    break;
                case crackitos_core::math::ShapeType::kNone:
                default:
                    break;
                }
                break;
            case crackitos_core::math::ShapeType::kCircle:
                switch (collider_b->GetShapeType())
                {
                case crackitos_core::math::ShapeType::kAABB:
                    SwapObjects();
                    CalculateProperties();
                    break;
                case crackitos_core::math::ShapeType::kCircle:
                    HandleCircleCircleCollision();
                    break;
                case crackitos_core::math::ShapeType::kPolygon:
                    HandleCirclePolygonCollision();
                    break;
                case crackitos_core::math::ShapeType::kNone:
                default:
                    break;
                }
                break;
            case crackitos_core::math::ShapeType::kPolygon:
                switch (collider_b->GetShapeType())
                {
                case crackitos_core::math::ShapeType::kAABB:
                case crackitos_core::math::ShapeType::kCircle:
                    SwapObjects();
                    CalculateProperties();
                    break;
                case crackitos_core::math::ShapeType::kPolygon:
                    HandlePolygonPolygonCollision();
                    break;
                case crackitos_core::math::ShapeType::kNone:
                default:
                    break;
                }
                break;
            case crackitos_core::math::ShapeType::kNone:
            default:
                break;
            }
        }

        void ResolveVelocities() const
        {
            auto& body_a = bodies_.first;
            auto& body_b = bodies_.second;

            const auto& collider_a = colliders_.first;
            const auto& collider_b = colliders_.second;

            // Calculate the relative velocity between the bodies
            const crackitos_core::math::Vec2f relative_velocity = body_a->velocity() - body_b->velocity();

            // Relative velocity along Normal
            const crackitos_core::commons::fp separating_velocity = crackitos_core::math::Vec2f::Dot(relative_velocity, contact_normal_);

            if (separating_velocity > 0.0f) { return; }

            // Calculate restitution
            crackitos_core::commons::fp restitution = (collider_a->bounciness() * body_a->mass() + collider_b->bounciness() * body_b->mass())
                                / (body_a->mass() + body_b->mass());

            // If velocity is very small, set restitution to zero
            constexpr crackitos_core::commons::fp restitution_threshold = 2.0f;
            if (std::abs(separating_velocity) < restitution_threshold)
            {
                restitution = 0.0f;
            }

            // Calculate impulse scalar
            crackitos_core::commons::fp impulse_magnitude = -(1.0f + restitution) * separating_velocity;
            impulse_magnitude /= (body_a->inverse_mass() + body_b->inverse_mass());
            crackitos_core::math::Vec2f impulse = impulse_magnitude * contact_normal_;

            // Debug output
            std::cout << "ResolveVelocities:" << std::endl;
            std::cout << "  Body A Velocity: " << body_a->velocity().x << " : " << body_a->velocity().y << std::endl;
            std::cout << "  Body B Velocity: " << body_b->velocity().x << " : " << body_b->velocity().y << std::endl;
            std::cout << "  Separating Velocity: " << separating_velocity << std::endl;
            std::cout << "  Impulse Magnitude: " << impulse_magnitude << std::endl;

            crackitos_core::commons::fp sleep_velocity_threshold = 10.0f;
            // Apply impulse to the dynamic bodies
            if (body_a->type() == BodyType::Dynamic)
            {
                body_a->ApplyImpulse(impulse);
                if (body_a->velocity().Magnitude() < sleep_velocity_threshold)
                    body_a->set_velocity(crackitos_core::math::Vec2f::Zero());
            }
            if (body_b->type() == BodyType::Dynamic)
            {
                body_b->ApplyImpulse(-impulse);
                if (body_b->velocity().Magnitude() < sleep_velocity_threshold)
                    body_b->set_velocity(crackitos_core::math::Vec2f::Zero());
            }

            //Friction
            //Tangent vector
            const crackitos_core::math::Vec2f tangent = (relative_velocity - separating_velocity * contact_normal_).Normalized();
            // Magnitude to apply
            const crackitos_core::commons::fp jt = -crackitos_core::math::Vec2f::Dot(relative_velocity, tangent) / (body_a->inverse_mass() + body_b->inverse_mass());
            const crackitos_core::commons::fp mu = std::sqrt(
                collider_a->friction() * collider_a->friction() + collider_b->friction() * collider_b->friction());

            // Clamp friction
            crackitos_core::math::Vec2f friction_impulse = crackitos_core::math::Vec2f::Zero();
            if (std::abs(jt) < impulse_magnitude * mu)
            {
                friction_impulse = jt * tangent;
            }
            else
            {
                const crackitos_core::commons::fp dynamic_friction_impulse = std::sqrt(
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
        }

        void ResolvePositions() const
        {
            if (penetration_ <= 0.0f) { return; }

            auto& body_a = bodies_.first;
            auto& body_b = bodies_.second;
            const auto inverse_mass_a = body_a->inverse_mass();
            const auto inverse_mass_b = body_b->inverse_mass();
            const auto total_inverse_mass = inverse_mass_a + inverse_mass_b;
            if(total_inverse_mass <= std::numeric_limits<crackitos_core::commons::fp>::epsilon()) { return; }

            //Correct positions to avoid objects sinking into each other
            //Only correct position if the penetration is above this threshold
            // physics923::commons::fp penetration_correction = std::max(penetration_, 0.01f);
            constexpr crackitos_core::commons::fp correction_percent = 1.0f;
            // const crackitos_core::math::Vec2f correction = contact_normal_ * (penetration_correction * correction_percent);
            const crackitos_core::math::Vec2f correction = contact_normal_ * (penetration_ * correction_percent);

            //// Debug output
            // std::cout << "ResolvePositions:" << std::endl;
            // std::cout << "  Penetration: " << penetration_ << std::endl;
            // std::cout << "  Correction: " << correction.x << " : " << correction.y << std::endl;
            // std::cout << "  Body A Position Before: " << body_a->position().x << " : " << body_a->position().y << std::endl;
            // std::cout << "  Body B Position Before: " << body_b->position().x << " : " << body_b->position().y << std::endl;

            // only move the dynamic bodies
            if (body_a->type() != BodyType::Static)
            {
                body_a->set_position(body_a->position() - correction * inverse_mass_a);
            }
            if (body_b->type() != BodyType::Static)
            {
                body_b->set_position(body_b->position() + correction * inverse_mass_b);
            }

            // // Debug output after position correction
            // std::cout << "  Body A Position After: " << body_a->position().x << " : " << body_a->position().y << std::endl;
            // std::cout << "  Body B Position After: " << body_b->position().x << " : " << body_b->position().y << std::endl;
            // std::cout << "-------------------------" << std::endl;
        }

        void HandleAABBAABBCollision()
        {
            const auto& aabb_a = std::get<crackitos_core::math::AABB>(colliders_.first->shape());
            const auto& aabb_b = std::get<crackitos_core::math::AABB>(colliders_.second->shape());
            const auto centre_a = bodies_.first->position();
            const auto centre_b = bodies_.second->position();

            //Calculate the overlap on each axis
            crackitos_core::commons::fp overlap_x = std::min(aabb_a.max_bound().x, aabb_b.max_bound().x) - std::max(
                aabb_a.min_bound().x, aabb_b.min_bound().x);
            crackitos_core::commons::fp overlap_y = std::min(aabb_a.max_bound().y, aabb_b.max_bound().y) - std::max(
                aabb_a.min_bound().y, aabb_b.min_bound().y);

            if (overlap_x <= 0.0f || overlap_y <= 0.0f)
            {
                penetration_ = 0.0f;
                return; // No collision detected
            }

            //Determine the smallest overlap direction
            if (overlap_x < overlap_y)
            {
                penetration_ = overlap_x;
                contact_normal_ = centre_a.x < centre_b.x ? crackitos_core::math::Vec2f(-1, 0) : crackitos_core::math::Vec2f(1, 0);
            }
            else
            {
                penetration_ = overlap_y;
                contact_normal_ = centre_a.y < centre_b.y ? crackitos_core::math::Vec2f(0, -1) : crackitos_core::math::Vec2f(0, 1);
            }

            //Calculate the contact point as the midpoint of the overlapping edges
            contact_point_ = {
                std::clamp((centre_a.x + centre_b.x) / 2, aabb_a.min_bound().x, aabb_a.max_bound().x),
                std::clamp((centre_a.y + centre_b.y) / 2, aabb_a.min_bound().y, aabb_a.max_bound().y)
            };

        }

        void HandleAABBCircleCollision()
{
    const auto& aabb = std::get<crackitos_core::math::AABB>(colliders_.first->shape());
    const auto& circle = std::get<crackitos_core::math::Circle>(colliders_.second->shape());
    const auto centre = circle.centre();
    const auto radius = circle.radius();

    // Find the closest point on the AABB to the circle center
    const crackitos_core::math::Vec2f closest_point = {
        std::clamp(centre.x, aabb.min_bound().x, aabb.max_bound().x),
        std::clamp(centre.y, aabb.min_bound().y, aabb.max_bound().y)
    };

    // Calculate the vector from the circle center to the closest point
    const crackitos_core::math::Vec2f delta = closest_point - centre;
    const crackitos_core::commons::fp distance = delta.Magnitude();

    if (distance >= radius)
    {
        penetration_ = 0.0f;
        return; // No collision
    }

    if (distance > std::numeric_limits<crackitos_core::commons::fp>::epsilon())
    {
        // Case: Circle's center is outside the AABB
        contact_normal_ = delta / distance; // Normalized vector
        penetration_ = radius - distance;
        contact_point_ = closest_point;
    }
    else
    {
        // Case: Circle's center is inside the AABB
        // Find the minimum distance to an edge of the AABB
        crackitos_core::commons::fp left_dist = std::abs(centre.x - aabb.min_bound().x);
        crackitos_core::commons::fp right_dist = std::abs(aabb.max_bound().x - centre.x);
        crackitos_core::commons::fp bottom_dist = std::abs(centre.y - aabb.min_bound().y);
        crackitos_core::commons::fp top_dist = std::abs(aabb.max_bound().y - centre.y);

        // Determine the edge with the smallest distance
        crackitos_core::commons::fp min_dist = std::min({left_dist, right_dist, bottom_dist, top_dist});

        if (min_dist == left_dist)
        {
            contact_normal_ = crackitos_core::math::Vec2f(-1, 0); // Left edge
            penetration_ = radius - left_dist;
            contact_point_ = {aabb.min_bound().x, centre.y};
        }
        else if (min_dist == right_dist)
        {
            contact_normal_ = crackitos_core::math::Vec2f(1, 0); // Right edge
            penetration_ = radius - right_dist;
            contact_point_ = {aabb.max_bound().x, centre.y};
        }
        else if (min_dist == bottom_dist)
        {
            contact_normal_ = crackitos_core::math::Vec2f(0, -1); // Bottom edge
            penetration_ = radius - bottom_dist;
            contact_point_ = {centre.x, aabb.min_bound().y};
        }
        else if (min_dist == top_dist)
        {
            contact_normal_ = crackitos_core::math::Vec2f(0, 1); // Top edge
            penetration_ = radius - top_dist;
            contact_point_ = {centre.x, aabb.max_bound().y};
        }
    }
}


        void HandleAABBPolygonCollision()
        {
            //Not Implemented
        }

        void HandleCircleCircleCollision()
        {
            const auto radius_a = std::get<crackitos_core::math::Circle>(colliders_.first->shape()).radius();
            const auto radius_b = std::get<crackitos_core::math::Circle>(colliders_.second->shape()).radius();
            const auto centre_a = std::get<crackitos_core::math::Circle>(colliders_.first->shape()).centre();
            const auto centre_b = std::get<crackitos_core::math::Circle>(colliders_.second->shape()).centre();

            //Calculate the vector between the centers
            const crackitos_core::math::Vec2f delta = centre_a - centre_b;
            crackitos_core::commons::fp dist = delta.Magnitude();
            crackitos_core::commons::fp total_radius = radius_a + radius_b;

            if (dist >= total_radius)
            {
                penetration_ = 0.0f;
                return; // No collision
            }

            if (dist < std::numeric_limits<crackitos_core::commons::fp>::epsilon())
            {
                contact_normal_ = crackitos_core::math::Vec2f(1.0f, 0.0f); // Arbitrary normal
                penetration_ = total_radius;       // Total overlap
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
