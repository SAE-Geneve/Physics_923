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
        crackitos_core::math::Vec2f position_ = crackitos_core::math::Vec2f::Zero();
        crackitos_core::math::Vec2f velocity_ = crackitos_core::math::Vec2f::Zero();
        crackitos_core::math::Vec2f acceleration_ = crackitos_core::math::Vec2f::Zero();
        crackitos_core::commons::fp damping_factor_ = 1.0f;

        //Angular components
        crackitos_core::commons::fp orientation_ = 0.0f;
        crackitos_core::commons::fp angular_velocity_ = 0.0f;
        crackitos_core::commons::fp torque_ = 0.0f;

        bool is_awake_ = true;
        bool gravity_bound_ = false;

        crackitos_core::commons::fp mass_ = 1.0f;
        crackitos_core::commons::fp inverse_mass_ = 1.0f;

        crackitos_core::commons::fp inertia_ = 1.0f;
        crackitos_core::commons::fp inverse_inertia_ = 1.0f;

        void ApplyGravity(const crackitos_core::math::Vec2f gravity)
        {
            if (type_ == BodyType::Dynamic && gravity_bound_)
            {
                ApplyForce(gravity);
            }
        }

    public:
        Body() = default;

        Body(const BodyType type,
             const crackitos_core::math::Vec2f position,
             const crackitos_core::math::Vec2f velocity,
             const bool gravity_bound,
             const crackitos_core::commons::fp mass)
        {
            type_ = type;
            position_ = position;
            velocity_ = velocity;
            mass_ = mass;

            gravity_bound_ = gravity_bound;

            if (type_ == BodyType::Static)
            {
                velocity_ = crackitos_core::math::Vec2f::Zero();
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

        Body(const crackitos_core::math::Vec2f position, const crackitos_core::commons::fp mass)
        {
            position_ = position;
            mass_ = mass;
            inverse_mass_ = 1.0f / mass;
        };

        //Getters
        [[nodiscard]] BodyType type() const { return type_; }
        [[nodiscard]] crackitos_core::math::Vec2f position() const { return position_; }
        [[nodiscard]] crackitos_core::math::Vec2f velocity() const { return velocity_; }
        [[nodiscard]] crackitos_core::math::Vec2f acceleration() const { return acceleration_; }
        [[nodiscard]] crackitos_core::commons::fp orientation() const { return orientation_; }
        [[nodiscard]] crackitos_core::commons::fp angular_velocity() const { return angular_velocity_; }
        [[nodiscard]] crackitos_core::commons::fp torque() const { return torque_; }
        [[nodiscard]] crackitos_core::commons::fp mass() const { return mass_; }
        [[nodiscard]] crackitos_core::commons::fp inverse_mass() const { return inverse_mass_; }
        [[nodiscard]] crackitos_core::commons::fp inertia() const { return inertia_; }
        [[nodiscard]] crackitos_core::commons::fp inverse_inertia() const { return inverse_inertia_; }

        //Setters
        void set_position(const crackitos_core::math::Vec2f new_position) { position_ = new_position; }
        void set_velocity(const crackitos_core::math::Vec2f new_velocity) { velocity_ = new_velocity; }
        void set_gravity_bound(const bool new_gravity_bound) { gravity_bound_ = new_gravity_bound; }
        void damping_factor(const crackitos_core::commons::fp factor) { damping_factor_ = factor; }

        void set_mass(const crackitos_core::commons::fp new_mass)
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
                velocity_ = crackitos_core::math::Vec2f::Zero();
                mass_ = 0.0f;
                inverse_mass_ = 0.0f;
            }
        }

        void ApplyForce(const crackitos_core::math::Vec2f force)
        {
            if (type_ == BodyType::Dynamic)
            {
                if (force.Magnitude() > crackitos_core::commons::kEpsilon)
                {
                    is_awake_ = true;
                }
                acceleration_ += force * inverse_mass_;
            }
        }

        void ApplyImpulse(const crackitos_core::math::Vec2f& impulse)
        {
            if (type_ == BodyType::Dynamic)
            {
                velocity_ = velocity_ + impulse * inverse_mass_;
            }
        }

        void Update(const crackitos_core::commons::fp delta_time, const crackitos_core::math::Vec2f& gravity = crackitos_core::math::Vec2f::Zero())
        {
            switch (type_)
            {
            case BodyType::Static:
                break;
            case BodyType::Kinematic:
                position_ += velocity_ * delta_time;
                break;
            case BodyType::Dynamic:
                if (gravity_bound_)
                {
                    ApplyForce(gravity);
                }

                velocity_ = velocity_ * damping_factor_;
                velocity_ += acceleration_ * delta_time;
                position_ += velocity_ * delta_time;
                ResetForce();
                break;
            case BodyType::None:
                break;
            }
        }

        void ResetForce() { acceleration_ = crackitos_core::math::Vec2f::Zero(); }
    };
} // namespace physics
#endif // CRACKITOS_PHYSICS_PHYSICS_BODY_H_
