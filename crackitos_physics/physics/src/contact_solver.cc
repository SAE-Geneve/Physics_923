#include "contact_solver.h"
#include <algorithm>
#include <cmath>

namespace crackitos_physics::physics
{
    void ContactSolver::SetContactObjects(Body& bodyA, Body& bodyB, Collider& colliderA, Collider& colliderB)
    {
        bodyA_ = &bodyA;
        bodyB_ = &bodyB;
        colliderA_ = &colliderA;
        colliderB_ = &colliderB;
    }

    void ContactSolver::SwapObjects()
    {
        std::swap(bodyA_, bodyB_);
        std::swap(colliderA_, colliderB_);
    }

    void ContactSolver::ResolveContact()
    {
        CalculateProperties();
        if (bodyA_->type() == BodyType::Static || bodyA_->type() == BodyType::Kinematic)
        {
            SwapObjects();
            contact_normal_ = -contact_normal_;
        }
        ResolveVelocities();
        ResolvePositions();
    }

    void ContactSolver::CalculateProperties()
    {
        contact_point_ = crackitos_core::math::Vec2f::Zero();
        contact_normal_ = crackitos_core::math::Vec2f::Zero();
        penetration_ = 0.0f;

        const auto eA = colliderA_->bounciness();
        const auto eB = colliderB_->bounciness();
        const auto mA = bodyA_->mass();
        const auto mB = bodyB_->mass();

        restitution_ = (mA + mB > crackitos_core::commons::kEpsilon) ? (mA * eA + mB * eB) / (mA + mB) : std::max(eA, eB);

        const auto type_a = colliderA_->GetShapeType();
        const auto type_b = colliderB_->GetShapeType();

        if (type_a == crackitos_core::math::ShapeType::kAABB && type_b == crackitos_core::math::ShapeType::kAABB)
        {
            HandleAABBAABBCollision();
        }
        else if (type_a == crackitos_core::math::ShapeType::kAABB && type_b == crackitos_core::math::ShapeType::kCircle)
        {
            HandleAABBCircleCollision();
        }
        else if (type_a == crackitos_core::math::ShapeType::kCircle && type_b == crackitos_core::math::ShapeType::kAABB)
        {
            SwapObjects();
            HandleAABBCircleCollision();
            contact_normal_ = -contact_normal_;
        }
        else if (type_a == crackitos_core::math::ShapeType::kCircle && type_b == crackitos_core::math::ShapeType::kCircle)
        {
            HandleCircleCircleCollision();
        }
        else if (type_a == crackitos_core::math::ShapeType::kPolygon && type_b == crackitos_core::math::ShapeType::kPolygon)
        {
            HandlePolygonPolygonCollision();
        }
        else if (type_a == crackitos_core::math::ShapeType::kAABB && type_b == crackitos_core::math::ShapeType::kPolygon)
        {
            HandleAABBPolygonCollision();
        }
        else if (type_a == crackitos_core::math::ShapeType::kPolygon && type_b == crackitos_core::math::ShapeType::kAABB)
        {
            SwapObjects();
            HandleAABBPolygonCollision();
            contact_normal_ = -contact_normal_;
        }
        else if (type_a == crackitos_core::math::ShapeType::kCircle && type_b == crackitos_core::math::ShapeType::kPolygon)
        {
            HandleCirclePolygonCollision();
        }
        else if (type_a == crackitos_core::math::ShapeType::kPolygon && type_b == crackitos_core::math::ShapeType::kCircle)
        {
            SwapObjects();
            HandleCirclePolygonCollision();
            contact_normal_ = -contact_normal_;
        }
    }

    void ContactSolver::ResolveVelocities() const
    {
        const auto inverse_mass_a = bodyA_->inverse_mass();
        const auto inverse_mass_b = bodyB_->inverse_mass();
        const auto total_inverse_mass = inverse_mass_a + inverse_mass_b;
        if (total_inverse_mass == 0.0f) return;

        const crackitos_core::math::Vec2f relative_velocity = bodyA_->velocity() - bodyB_->velocity();
        const crackitos_core::commons::fp separating_velocity = crackitos_core::math::Vec2f::Dot(relative_velocity, contact_normal_);
        if (separating_velocity > 0.0f) return;

        const auto delta_velocity = -separating_velocity * restitution_ - separating_velocity;
        const auto impulse_magnitude = delta_velocity / total_inverse_mass;
        const auto impulse = contact_normal_ * impulse_magnitude;

        if (bodyA_->type() == BodyType::Dynamic)
        {
            bodyA_->ApplyImpulse(impulse);
        }
        if (bodyB_->type() == BodyType::Dynamic)
        {
            bodyB_->ApplyImpulse(-impulse);
        }

        crackitos_core::math::Vec2f tangent = relative_velocity - separating_velocity * contact_normal_;
        if (tangent.SquareMagnitude() > crackitos_core::commons::kEpsilon) {
            tangent = tangent.Normalized();
        } else {
            tangent = crackitos_core::math::Vec2f::Zero(); // Avoid applying friction if there's no lateral movement
        }

        const float jt = -crackitos_core::math::Vec2f::Dot(relative_velocity, tangent) / (bodyA_->inverse_mass() + bodyB_->inverse_mass());
        const float mu = std::sqrt(
            colliderA_->friction() * colliderA_->friction() + colliderB_->friction() * colliderB_->friction());

        // Clamp friction
        crackitos_core::math::Vec2f friction_impulse = crackitos_core::math::Vec2f::Zero();
        if (std::abs(jt) < impulse_magnitude * mu)
        {
            friction_impulse = jt * tangent;
        }
        else
        {
            const float dynamic_friction_impulse = std::sqrt(
                colliderA_->dynamic_friction() * colliderA_->dynamic_friction() + colliderB_->dynamic_friction() *
                colliderB_->dynamic_friction());
            friction_impulse = -impulse_magnitude * tangent * dynamic_friction_impulse;
        }

        // Apply impulse to the dynamic bodies
        if (bodyA_->type() == BodyType::Dynamic)
        {
            bodyA_->ApplyImpulse(friction_impulse);
        }
        if (bodyB_->type() == BodyType::Dynamic)
        {
            bodyB_->ApplyImpulse(-friction_impulse);
        }
    }

    void ContactSolver::ResolvePositions() const
    {
        if (penetration_ <= 0.0f) return;

        const auto inverse_mass_a = bodyA_->inverse_mass();
        const auto inverse_mass_b = bodyB_->inverse_mass();
        const auto total_inverse_mass = inverse_mass_a + inverse_mass_b;
        if (total_inverse_mass == 0.0f) return;

        const auto move_per_inverse_mass = contact_normal_ * (penetration_ / total_inverse_mass);

        if (bodyA_->type() == BodyType::Dynamic)
        {
            bodyA_->set_position(bodyA_->position() + move_per_inverse_mass * inverse_mass_a);
        }
        if (bodyB_->type() == BodyType::Dynamic)
        {
            bodyB_->set_position(bodyB_->position() - move_per_inverse_mass * inverse_mass_b);
        }
    }

    void ContactSolver::HandleAABBAABBCollision()
    {
        const auto& aabb_a = std::get<crackitos_core::math::AABB>(colliderA_->shape());
        const auto& aabb_b = std::get<crackitos_core::math::AABB>(colliderB_->shape());
        const auto centre_a = bodyA_->position();
        const auto centre_b = bodyB_->position();
        const auto delta = centre_a - centre_b;

        contact_point_ = 0.5f * (aabb_a.ClosestPoint(centre_b) + aabb_b.ClosestPoint(centre_a));

        //TODO own function abs, vec2f right, left, up, down
        const auto penetration_x = aabb_a.half_size_vec().x + aabb_b.half_size_vec().x - std::abs(delta.x);
        const auto penetration_y = aabb_a.half_size_vec().y + aabb_b.half_size_vec().y - std::abs(delta.y);

        // Check for deep overlap (for example if one shape spawns in another)
        const bool is_a_inside_b = (centre_a.x >= aabb_b.min_bound().x && centre_a.x <= aabb_b.max_bound().x &&
            centre_a.y >= aabb_b.min_bound().y && centre_a.y <= aabb_b.max_bound().y);

        //In which case, force an upward resolution
        if (is_a_inside_b && (penetration_x > 0.01f || penetration_y > 0.01f))
        {
            contact_normal_ = crackitos_core::math::Vec2f(0.0f, -1.0f);
            penetration_ = penetration_y;
        }
        else
        {
            if (penetration_x < penetration_y)
            {
                contact_normal_ = delta.x > 0.0f ? crackitos_core::math::Vec2f(1.0f, 0.0f) : crackitos_core::math::Vec2f(-1.0f, 0.0f);
                penetration_ = penetration_x;
            }
            else
            {
                contact_normal_ = delta.y > 0.0f ? crackitos_core::math::Vec2f(0.0f, 1.0f) : crackitos_core::math::Vec2f(0.0f, -1.0f);
                penetration_ = penetration_y;
            }
        }
    }

    void ContactSolver::HandleAABBCircleCollision()
    {
        const auto& aabb = std::get<crackitos_core::math::AABB>(colliderA_->shape());
        const auto& circle = std::get<crackitos_core::math::Circle>(colliderB_->shape());
        const auto circle_centre = circle.centre();
        const auto aabb_centre = aabb.GetCentre();
        const auto aabb_half_size = aabb.half_size_vec();
        const auto radius = circle.radius();


        // Find the closest point on the AABB to the circle center
        const crackitos_core::math::Vec2f closest_point = {
            std::clamp(circle_centre.x, aabb.min_bound().x, aabb.max_bound().x),
            std::clamp(circle_centre.y, aabb.min_bound().y, aabb.max_bound().y)
        };

        const crackitos_core::math::Vec2f distance_vec = closest_point - circle_centre;
        const crackitos_core::commons::fp distance_squared = distance_vec.SquareMagnitude();
        const crackitos_core::commons::fp radius_squared = radius * radius;

        if (distance_squared >= radius_squared)
        {
            penetration_ = 0.0f;
            return; // No collision
        }

        const crackitos_core::commons::fp distance = std::sqrt(distance_squared);

        // Circle is outside the AABB
        contact_normal_ = distance > crackitos_core::commons::kEpsilon
                              ? distance_vec / distance
                              : crackitos_core::math::Vec2f(0.0f, 1.0f);
        penetration_ = radius - distance;

        contact_point_ = closest_point;
    }

    void ContactSolver::HandleCircleCircleCollision()
    {
        const auto radius_a = std::get<crackitos_core::math::Circle>(colliderA_->shape()).radius();
        const auto radius_b = std::get<crackitos_core::math::Circle>(colliderB_->shape()).radius();
        const auto centre_a = std::get<crackitos_core::math::Circle>(colliderA_->shape()).centre();
        const auto centre_b = std::get<crackitos_core::math::Circle>(colliderB_->shape()).centre();

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
            penetration_ = total_radius; // Total overlap
        }
        else
        {
            contact_normal_ = delta / dist;
            penetration_ = total_radius - dist;
        }

        contact_point_ = centre_a + contact_normal_ * radius_a;
    }

    void ContactSolver::HandlePolygonPolygonCollision()
    {
        // Not implemented
    }

    void ContactSolver::HandleAABBPolygonCollision()
    {
        // Not implemented
    }

    void ContactSolver::HandleCirclePolygonCollision()
    {
        // Not implemented
    }
}
