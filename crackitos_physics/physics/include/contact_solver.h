#ifndef CRACKITOS_PHYSICS_PHYSICS_CONTACT_SOLVER_H_
#define CRACKITOS_PHYSICS_PHYSICS_CONTACT_SOLVER_H_

#include "vec2.h"
#include "commons.h"
#include "body.h"
#include "collider.h"

namespace crackitos_physics::physics
{
    constexpr crackitos_core::commons::fp kPenetrationEpsilon = 0.0001f;
    constexpr crackitos_core::commons::fp kSleepVelocityThreshold = 0.5f;

    struct ContactSolver
    {
        crackitos_core::math::Vec2f contact_point_ = crackitos_core::math::Vec2f::Zero();
        crackitos_core::math::Vec2f contact_normal_ = crackitos_core::math::Vec2f::Zero();

        crackitos_core::commons::fp restitution_ = 0.0f;
        crackitos_core::commons::fp penetration_ = 0.0f;

        void SetContactObjects(Body& bodyA, Body& bodyB, Collider& colliderA, Collider& colliderB);
        void ResolveContact();
        void CalculateProperties();

    private:
        Body* bodyA_ = nullptr;
        Body* bodyB_ = nullptr;
        Collider* colliderA_ = nullptr;
        Collider* colliderB_ = nullptr;

        void SwapObjects();
        void ResolveVelocities() const;
        void ResolvePositions() const;

        void HandleAABBAABBCollision();
        void HandleAABBCircleCollision();
        void HandleCircleCircleCollision();
        void HandlePolygonPolygonCollision();
        void HandleAABBPolygonCollision();
        void HandleCirclePolygonCollision();
    };

} // namespace physics

#endif // CRACKITOS_PHYSICS_PHYSICS_CONTACT_SOLVER_H_