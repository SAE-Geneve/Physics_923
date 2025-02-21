#ifndef PHYSICS_923_LIB_PHYSICS_BODY_H_
#define PHYSICS_923_LIB_PHYSICS_BODY_H_

#include "commons.h"
#include "vec2.h"

namespace physics923::physics
{
    enum class BodyType
    {
        Static,
        Kinematic,
        Dynamic,
        None
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
        float orientation_ = 0.0f;
        float angular_velocity_ = 0.0f;
        float torque_ = 0.0f;

        bool is_awake_ = true;

        float mass_ = 1.0f;
        float inverse_mass_ = 1.0f;

        float inertia_ = 1.0f;
        float inverse_inertia_ = 1.0f;

    public:
        Body() = default;

        Body(const BodyType type, const math::Vec2f position, const math::Vec2f velocity, const float mass)
        {
            type_ = type;
            position_ = position;
            velocity_ = velocity;
            mass_ = mass;

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

        Body(const math::Vec2f position, const float mass)
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
        [[nodiscard]] float orientation() const { return orientation_; }
        [[nodiscard]] float angular_velocity() const { return angular_velocity_; }
        [[nodiscard]] float torque() const { return torque_; }
        [[nodiscard]] float mass() const { return mass_; }
        [[nodiscard]] float inverse_mass() const { return inverse_mass_; }
        [[nodiscard]] float inertia() const { return inertia_; }
        [[nodiscard]] float inverse_inertia() const { return inverse_inertia_; }


        //Setters
        void set_position(const math::Vec2f new_position) { position_ = new_position; }
        void set_velocity(const math::Vec2f new_velocity) { velocity_ = new_velocity; }

        void set_mass(const float new_mass)
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
                if(force.Magnitude() > commons::Epsilon)
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
                velocity_ += impulse * inverse_mass_;
            }
        }

        void ApplyGravity(const math::Vec2f gravity)
        {
            if (type_ == BodyType::Dynamic)
            {
                ApplyForce(gravity);
            }
        }

        void Update(const float delta_time)
        {
            if (type_ != BodyType::Static)
            {
                    velocity_ += acceleration_ * delta_time;

                    position_ += velocity_ * delta_time;

            }
            ResetForce();
        }

        void ResetForce() { acceleration_ = math::Vec2f::Zero(); }
    };
}
#endif //PHYSICS_923_LIB_PHYSICS_BODY_H_
