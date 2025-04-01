#include <iostream>

#include "physics_world.h"

#include "contact_solver.h"

#ifdef TRACY_ENABLE
#include <tracy/Tracy.hpp>
#endif

namespace crackitos_physics::physics
{
    PhysicsWorld::PhysicsWorld() : frame_bounds_(math::AABB(math::Vec2f(0, 0), math::Vec2f(1000, 1000))),
                                   out_of_bounds_removal_state_(true),
                                   quadtree_(frame_bounds_),
                                   gravity_(kDefaultGravity)
    {
    }

    PhysicsWorld::~PhysicsWorld()
    {
        Clear();
    }

    void PhysicsWorld::Initialize(const math::AABB& world_bounds, const bool out_of_bounds_removal_state,
                                  const math::Vec2f gravity)
    {
#ifdef TRACY_ENABLE
        ZoneScoped;
#endif
        Clear();
        bodies_.reserve(1000);
        colliders_.reserve(1000);
        frame_bounds_ = world_bounds;
        out_of_bounds_removal_state_ = out_of_bounds_removal_state;
        gravity_ = gravity;
        quadtree_ = Quadtree(frame_bounds_); // Reinitialize quadtree
    }

    void PhysicsWorld::Clear()
    {
        bodies_.clear();
        body_generations_.clear();
        colliders_.clear();
        collider_generations_.clear();
        active_pairs_.clear();
        quadtree_.Clear();
    }

    BodyHandle PhysicsWorld::CreateBody(const Body& body_def)
    {
#ifdef TRACY_ENABLE
        ZoneScoped;
#endif
        const int id = static_cast<int>(bodies_.size());
        bodies_.push_back(body_def);
        body_generations_.push_back(0);
        return {id, body_generations_[id]};
    }

    ColliderHandle PhysicsWorld::CreateCollider(const BodyHandle body, const Collider& collider_def)
    {
#ifdef TRACY_ENABLE
        ZoneScoped;
#endif
        const int id = static_cast<int>(colliders_.size());
        colliders_.emplace_back(collider_def);
        colliders_[id].set_body_handle(body);
        collider_generations_.push_back(0);
        return {id, collider_generations_[id]};
    }

    void PhysicsWorld::RemoveBody(const BodyHandle body)
    {
#ifdef TRACY_ENABLE
        ZoneScoped;
#endif
        if (body.id < 0 || body.id >= static_cast<int>(bodies_.size())) return;

        bodies_[body.id] = Body();
        body_generations_[body.id]++;
    }

    void PhysicsWorld::RemoveCollider(const ColliderHandle collider)
    {
#ifdef TRACY_ENABLE
        ZoneScoped;
#endif
        if (collider.id < 0 || collider.id >= static_cast<int>(colliders_.size())) return;

        colliders_[collider.id] = Collider();
        collider_generations_[collider.id]++;
    }

    const Body& PhysicsWorld::GetBody(const BodyHandle body) const
    {
#ifdef TRACY_ENABLE
        ZoneScoped;
#endif
        if (body.id < 0 || body.id >= static_cast<int>(bodies_.size()) || body_generations_[body.id] != body.generation)
        {
            throw std::out_of_range("Invalid BodyHandle");
        }
        return bodies_[body.id];
    }

    Body& PhysicsWorld::GetMutableBody(const BodyHandle body)
    {
#ifdef TRACY_ENABLE
        ZoneScoped;
#endif
        if (body.id < 0 || body.id >= static_cast<int>(bodies_.size()) || body_generations_[body.id] != body.generation)
        {
            throw std::out_of_range("Invalid BodyHandle");
        }
        return bodies_[body.id];
    }

    const Collider& PhysicsWorld::GetCollider(const ColliderHandle collider) const
    {
#ifdef TRACY_ENABLE
        ZoneScoped;
#endif
        if (collider.id < 0 || collider.id >= static_cast<int>(colliders_.size()) || collider_generations_[collider.id]
            != collider.generation)
        {
            throw std::out_of_range("Invalid ColliderHandle");
        }
        return colliders_[collider.id];
    }


    std::vector<std::pair<BodyHandle, ColliderHandle>> PhysicsWorld::GetBodiesWithColliders() const
    {
#ifdef TRACY_ENABLE
        ZoneScoped;
#endif
        std::vector<std::pair<BodyHandle, ColliderHandle>> body_collider_pairs;

        for (size_t i = 0; i < colliders_.size(); ++i)
        {
            ColliderHandle collider_handle{static_cast<int>(i), collider_generations_[i]};
            const auto& collider = colliders_[i];

            BodyHandle body_handle = collider.body_handle();

            if (body_handle.id >= 0 && body_handle.id < static_cast<int>(bodies_.size()))
            {
                body_collider_pairs.emplace_back(body_handle, collider_handle);
            }
        }

        return body_collider_pairs;
    }


    void PhysicsWorld::Update(commons::fp delta_time)
    {
#ifdef TRACY_ENABLE
        ZoneScoped;
#endif
        timer_.Tick();
        StepSimulation();
    }

    void PhysicsWorld::StepSimulation()
    {
#ifdef TRACY_ENABLE
        ZoneScoped;
#endif
        UpdateObjects();
        RemoveOutOfBoundsObjects();
        BroadPhase();
        NarrowPhase();
    }

    void PhysicsWorld::set_out_of_bounds_margins(const float left, const float right, const float top,
                                                 const float bottom)
    {
#ifdef TRACY_ENABLE
        ZoneScoped;
#endif
        out_of_bounds_margin_left_ = left;
        out_of_bounds_margin_right_ = right;
        out_of_bounds_margin_top_ = top;
        out_of_bounds_margin_bottom_ = bottom;
    }


    void PhysicsWorld::UpdateObjects()
    {
#ifdef TRACY_ENABLE
        ZoneScoped;
#endif
        for (size_t i = 0; i < bodies_.size(); ++i)
        {
            auto& body = bodies_[i];
            body.Update(time_step_, gravity_);

            // Ensure collider follows body movement
            if (i < colliders_.size())
            {
                colliders_[i].UpdatePosition(body.position());
            }
        }
    }


    void PhysicsWorld::RemoveOutOfBoundsObjects()
    {
#ifdef TRACY_ENABLE
        ZoneScoped;
#endif
        if (!out_of_bounds_removal_state_) return; //Skip if disabled

        for (size_t i = bodies_.size(); i-- > 0;) //Reverse iteration for safe removal
        {
            bool is_out_of_bounds = false;

            if (i < colliders_.size()) //Ensure collider exists
            {
                const auto& bounding_box = colliders_[i].GetBoundingBox();

                //Check if the collider is fully outside with its respective margin
                if (bounding_box.max_bound().x < frame_bounds_.min_bound().x - out_of_bounds_margin_left_ || // Left
                    bounding_box.min_bound().x > frame_bounds_.max_bound().x + out_of_bounds_margin_right_ || // Right
                    bounding_box.max_bound().y < frame_bounds_.min_bound().y - out_of_bounds_margin_top_ || // Top
                    bounding_box.min_bound().y > frame_bounds_.max_bound().y + out_of_bounds_margin_bottom_) // Bottom
                {
                    is_out_of_bounds = true;
                }
            }
            else //Use body position if collider doesn't exist
            {
                const auto& body_pos = bodies_[i].position();

                if (body_pos.x < frame_bounds_.min_bound().x - out_of_bounds_margin_left_ || // Left
                    body_pos.x > frame_bounds_.max_bound().x + out_of_bounds_margin_right_ || // Right
                    body_pos.y < frame_bounds_.min_bound().y - out_of_bounds_margin_top_ || // Top
                    body_pos.y > frame_bounds_.max_bound().y + out_of_bounds_margin_bottom_) // Bottom
                {
                    is_out_of_bounds = true;
                }
            }

            if (is_out_of_bounds)
            {
                if (i < body_generations_.size()) //Ensure valid index
                {
                    RemoveBody({static_cast<int>(i), body_generations_[i]});
                }
                if (i < collider_generations_.size()) //Ensure valid index
                {
                    RemoveCollider({static_cast<int>(i), collider_generations_[i]});
                }
            }
        }
    }


    void PhysicsWorld::BroadPhase()
    {
#ifdef TRACY_ENABLE
        ZoneScoped;
#endif
        quadtree_.Clear();

        // Insert static objects first
        for (size_t id = 0; id < colliders_.size(); ++id)
        {
            const auto& collider = colliders_[id];
            const auto& body_handle = collider.body_handle();

            if (body_handle.id < 0 || body_handle.id >= static_cast<int>(bodies_.size())) continue;

            const auto& body = bodies_[body_handle.id];

            if (body.type() == BodyType::Static)
            {
                const ColliderHandle collider_handle{static_cast<int>(id), collider_generations_[id]};
                quadtree_.Insert(collider_handle, collider.GetBoundingBox());
            }
        }

        // Insert non-static objects next
        for (size_t id = 0; id < colliders_.size(); ++id)
        {
            const auto& collider = colliders_[id];
            const auto& body_handle = collider.body_handle();

            if (body_handle.id < 0 || body_handle.id >= static_cast<int>(bodies_.size())) continue;

            const auto& body = bodies_[body_handle.id];

            if (body.type() != BodyType::Static)
            {
                const ColliderHandle collider_handle{static_cast<int>(id), collider_generations_[id]};
                quadtree_.Insert(collider_handle, collider.GetBoundingBox());
            }
        }

        quadtree_.BuildPotentialPairs();
    }

    void PhysicsWorld::NarrowPhase()
    {
#ifdef TRACY_ENABLE
        ZoneScoped;
#endif
        const auto& potentialPairs = quadtree_.GetPotentialPairs();
        std::unordered_set<ColliderPair> newActivePairs;

        for (const auto& pair : potentialPairs)
        {
            ContactSolver solver;
            solver.SetContactObjects(
                bodies_[colliders_[pair.colliderA.id].body_handle().id],
                bodies_[colliders_[pair.colliderB.id].body_handle().id],
                colliders_[pair.colliderA.id],
                colliders_[pair.colliderB.id]
            );

            solver.CalculateProperties();

            if (solver.penetration_ > 0.0f)
            {
                const bool is_new_pair = !active_pairs_.contains(pair);
                ResolveCollisionPair(pair, is_new_pair);
                newActivePairs.insert(pair);
            }
        }

        // Handle exited collisions & triggers
        for (auto it = active_pairs_.begin(); it != active_pairs_.end();)
        {
            if (!newActivePairs.contains(*it))
            {
                if (contact_listener_)
                {
                    if (colliders_[it->colliderA.id].is_trigger() || colliders_[it->colliderB.id].is_trigger())
                    {
                        contact_listener_->OnTriggerExit(*it);
                    }
                    else
                    {
                        contact_listener_->OnCollisionExit(*it);
                    }
                }
                it = active_pairs_.erase(it);
            }
            else
            {
                ++it;
            }
        }

        active_pairs_ = std::move(newActivePairs);
    }


    void PhysicsWorld::ResolveCollisionPair(const ColliderPair& pair, const bool is_new_pair)
    {
#ifdef TRACY_ENABLE
        ZoneScoped;
#endif
        const ColliderHandle colA = pair.colliderA;
        const ColliderHandle colB = pair.colliderB;

        // Get associated bodies
        const BodyHandle bodyA = colliders_[colA.id].body_handle();
        const BodyHandle bodyB = colliders_[colB.id].body_handle();

        // Ensure both colliders are linked to valid bodies
        if (bodyA.id < 0 || bodyA.id >= static_cast<int>(bodies_.size()) ||
            bodyB.id < 0 || bodyB.id >= static_cast<int>(bodies_.size()))
        {
            return;
        }

        ContactSolver solver;
        solver.SetContactObjects(
            bodies_[bodyA.id],
            bodies_[bodyB.id],
            colliders_[colA.id],
            colliders_[colB.id]
        );


        solver.CalculateProperties();

        if (solver.penetration_ <= 0.0f)
        {
            return;
        }

        if (colliders_[colA.id].is_trigger() || colliders_[colB.id].is_trigger())
        {
            if (contact_listener_)
            {
                if (is_new_pair)
                {
                    contact_listener_->OnTriggerEnter(pair);
                }
                else
                {
                    contact_listener_->OnTriggerStay(pair);
                }
            }
        }
        else
        {
            if (is_new_pair && contact_listener_)
            {
                contact_listener_->OnCollisionEnter(pair);
            }
            else if (!is_new_pair && contact_listener_)
            {
                contact_listener_->OnCollisionStay(pair);
            }

            solver.ResolveContact();
        }
    }
} // crackitos_physics
