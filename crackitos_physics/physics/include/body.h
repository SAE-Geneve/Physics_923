#ifndef CRACKITOS_PHYSICS_PHYSICS_BODY_H_
#define CRACKITOS_PHYSICS_PHYSICS_BODY_H_

#include "commons.h"
#include "vec2.h"

namespace crackitos_physics::physics
{
    enum class BodyType
    {
        Static,
        Kinematic,
        Dynamic,
        None
    };

    struct BodyHandle
    {
        int id = -1; // Unique ID
        int generation = 0; // Prevents accessing deleted bodies

        bool operator==(const BodyHandle& other) const
        {
            return id == other.id && generation == other.generation;
        }
    };

    class Body
    {
    private:
        BodyType type_ = BodyType::Dynamic;

        //Linear components
        math::Vec2f position_ = math::Vec2f::Zero();
        math::Vec2f velocity_ = math::Vec2f::Zero();
        math::Vec2f acceleration_ = math::Vec2f::Zero();

        //Angular components
        commons::fp orientation_ = 0.0f;
        commons::fp angular_velocity_ = 0.0f;
        commons::fp torque_ = 0.0f;

        bool is_awake_ = true;
        bool gravity_bound_ = false;

        commons::fp mass_ = 1.0f;
        commons::fp inverse_mass_ = 1.0f;

        commons::fp inertia_ = 1.0f;
        commons::fp inverse_inertia_ = 1.0f;

        void ApplyGravity(const math::Vec2f gravity)
        {
            if (type_ == BodyType::Dynamic && gravity_bound_)
            {
                ApplyForce(gravity);
            }
        }

    public:
        Body() = default;

        Body(const BodyType type,
             const math::Vec2f position,
             const math::Vec2f velocity,
             const bool gravity_bound,
             const commons::fp mass)
        {
            type_ = type;
            position_ = position;
            velocity_ = velocity;
            mass_ = mass;

            gravity_bound_ = gravity_bound;

            if (type_ == BodyType::Static)
            {
                velocity_ = math::Vec2f::Zero();
                mass_ = 0.0f;
            }

            if (mass_ == 0.0f)
            {
                inverse_mass_ = 0.0f;
            }
            else
            {
                inverse_mass_ = 1.0f / mass;
            }
        };

        Body(const math::Vec2f position, const commons::fp mass)
        {
            position_ = position;
            mass_ = mass;
            inverse_mass_ = 1.0f / mass;
        };

        //Getters
        [[nodiscard]] BodyType type() const { return type_; }
        [[nodiscard]] math::Vec2f position() const { return position_; }
        [[nodiscard]] math::Vec2f velocity() const { return velocity_; }
        [[nodiscard]] math::Vec2f acceleration() const { return acceleration_; }
        [[nodiscard]] commons::fp orientation() const { return orientation_; }
        [[nodiscard]] commons::fp angular_velocity() const { return angular_velocity_; }
        [[nodiscard]] commons::fp torque() const { return torque_; }
        [[nodiscard]] commons::fp mass() const { return mass_; }
        [[nodiscard]] commons::fp inverse_mass() const { return inverse_mass_; }
        [[nodiscard]] commons::fp inertia() const { return inertia_; }
        [[nodiscard]] commons::fp inverse_inertia() const { return inverse_inertia_; }

        //Setters
        void set_position(const math::Vec2f new_position) { position_ = new_position; }
        void set_velocity(const math::Vec2f new_velocity) { velocity_ = new_velocity; }
        void set_gravity_bound(const bool new_gravity_bound) { gravity_bound_ = new_gravity_bound; }

        void set_mass(const commons::fp new_mass)
        {
            mass_ = new_mass;
            if (new_mass == 0.0f)
            {
                inverse_mass_ = 0.0f;
            }
            else
            {
                inverse_mass_ = 1.0f / new_mass;
            }
        }

        void set_type(const BodyType new_type)
        {
            type_ = new_type;
            if (new_type == BodyType::Static)
            {
                velocity_ = math::Vec2f::Zero();
                mass_ = 0.0f;
                inverse_mass_ = 0.0f;
            }
        }

        void ApplyForce(const math::Vec2f force)
        {
            if (type_ == BodyType::Dynamic)
            {
                if (force.Magnitude() > commons::kEpsilon)
                {
                    is_awake_ = true;
                }
                acceleration_ += force * inverse_mass_;
            }
        }

        void ApplyImpulse(const math::Vec2f& impulse)
        {
            if (type_ == BodyType::Dynamic)
            {
                velocity_ = velocity_ + impulse * inverse_mass_;
            }
        }

        void Update(const commons::fp delta_time, const math::Vec2f& gravity = math::Vec2f::Zero())
        {
            if (type_ != BodyType::Static)
            {
                if (type_ == BodyType::Dynamic && gravity_bound_)
                {
                    ApplyForce(gravity);
                }
                velocity_ += acceleration_ * delta_time;
                position_ += velocity_ * delta_time;
            }
            ResetForce();
        }

        void ResetForce() { acceleration_ = math::Vec2f::Zero(); }
    };
} // namespace physics
#endif // CRACKITOS_PHYSICS_PHYSICS_BODY_H_
