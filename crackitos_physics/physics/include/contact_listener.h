#ifndef CRACKITOS_PHYSICS_PHYSICS_CONTACT_LISTENER_H_
#define CRACKITOS_PHYSICS_PHYSICS_CONTACT_LISTENER_H_

#include "collider.h"

namespace crackitos_physics::physics
{
    class ContactListener
    {
    public:
        virtual ~ContactListener() = default;

        virtual void OnTriggerEnter(const ColliderPair& pair) = 0;
        virtual void OnTriggerStay(const ColliderPair& pair) = 0;
        virtual void OnTriggerExit(const ColliderPair& pair) = 0;

        virtual void OnCollisionEnter(const ColliderPair& pair) = 0;
        virtual void OnCollisionStay(const ColliderPair& pair) = 0;
        virtual void OnCollisionExit(const ColliderPair& pair) = 0;
    };
} // namespace crackitos_physics::physics

#endif // CRACKITOS_PHYSICS_PHYSICS_CONTACT_LISTENER_H_
