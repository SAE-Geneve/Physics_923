#include "collision_system.h"

#include <ranges>

#include "contact_solver.h"
#include "display.h"
#include "random.h"

namespace physics923
{
    void CollisionSystem::Initialize()
    {
        Clear();

        quadtree_ = new physics::Quadtree(math::AABB(math::Vec2f(0, 0),  math::Vec2f(kWindowWidth, kWindowHeight)));
        constexpr physics923::commons::fp margin = 20.0f;

        for (size_t i = 0; i < number_of_objects_ / 2 - 1; i++)
        {
            const math::Vec2f position(random::Range(margin, kWindowWidth - margin), random::Range(margin, kWindowHeight - margin));
            const physics923::commons::fp radius = random::Range(5.f, 10.f);

            math::Circle circle(position, radius);
            CreateObject(i, circle);
        }
        for (size_t i = number_of_objects_ / 2 - 1; i < number_of_objects_; i++)
        {
            const math::Vec2f position(random::Range(margin, kWindowWidth - margin), random::Range(margin, kWindowHeight - margin));
            math::Vec2f half_size_vec = math::Vec2f(random::Range(5.f, 10.f),random::Range(5.f, 10.f));
            auto half_size_length = half_size_vec.Magnitude();

            math::AABB aabb(position, half_size_vec, half_size_length);
            CreateObject(i, aabb);
        }
    }

    void CollisionSystem::Clear()
    {
        if (quadtree_)
        {
            quadtree_->Clear();
            delete quadtree_;
            quadtree_ = nullptr;
        }

        for (size_t i = 0; i < number_of_objects_; ++i)
        {
            DeleteObject(i);
        }

        objects_.fill({});
        potential_pairs_.clear();
        active_pairs_.clear();
        collider_to_object_map_.clear();
    }


    void CollisionSystem::CreateObject(size_t index, math::Circle& circle)
    {
        math::Vec2f velocity(random::Range(-50.0f, 50.0f), random::Range(-50.0f, 50.0f));
        physics::Body body(physics::BodyType::Dynamic,circle.centre(), velocity, random::Range(1.0f, 50.0f));
        physics::Collider collider(circle, random::Range(1.0f, 1.0f), 0, false);
        GameObject object(body, collider, circle.radius());

        objects_[index] = object;
        RegisterObject(objects_[index]);
    }

    void CollisionSystem::CreateObject(size_t index, math::AABB& aabb)
    {
        math::Vec2f velocity(random::Range(-50.0f, 50.0f), random::Range(-50.0f, 50.0f));
        physics::Body body(physics::BodyType::Dynamic, aabb.GetCentre(), velocity, random::Range(1.0f, 50.0f));
        physics::Collider collider(aabb, random::Range(1.0f, 1.0f), 0, false);
        GameObject object(body, collider, aabb.half_size_length());

        objects_[index] = object;
        RegisterObject(objects_[index]);
    }

    void CollisionSystem::DeleteObject(size_t index)
    {
        if (index >= objects_.size()) return;

        GameObject& object = objects_[index];
        UnregisterObject(object);
        objects_[index] = {};
    }

    void CollisionSystem::RegisterObject(GameObject& object)
    {
        collider_to_object_map_[&object.collider()] = &object;
    }

    void CollisionSystem::UnregisterObject(GameObject& object)
    {
        collider_to_object_map_.erase(&object.collider());
    }

    void CollisionSystem::Update(physics923::commons::fp delta_time)
    {
        UpdateShapes(delta_time);
        BroadPhase();
        NarrowPhase();
    }

    void CollisionSystem::UpdateShapes(physics923::commons::fp delta_time)
    {
        for (auto& object : objects_)
        {
            auto& body = object.body();
            auto& collider = object.collider();

            body.Update(delta_time);

            auto position = body.position();

            physics923::commons::fp radius = object.radius();

            //Check for collision with window borders
            if (position.x - radius < 0)
            {
                position.x = radius;
                body.set_velocity(math::Vec2f(-body.velocity().x, body.velocity().y));
            }
            if(position.x + radius > 1200)
            {
                position.x = 1200 - radius;
                body.set_velocity(math::Vec2f(-body.velocity().x, body.velocity().y));
            }
            if (position.y - radius < 0)
            {
                position.y = radius;
                body.set_velocity(math::Vec2f(body.velocity().x, -body.velocity().y));
            }
            if(position.y + radius > 800)
            {
                position.y = 800 - radius;
                body.set_velocity(math::Vec2f(body.velocity().x, -body.velocity().y));
            }

            //Update the collider's position
            collider.UpdatePosition(position);
        }
    }

    void CollisionSystem::SimplisticBroadPhase()
    {
        std::unordered_map<GameObjectPair, bool> new_potential_pairs;

        // Loop through all objects
        for (size_t i = 0; i < objects_.size(); ++i)
        {
            auto& objectA = objects_[i];
            auto& colliderA = objectA.collider();

            // Get the AABB of the first collider
            auto rangeA = colliderA.GetBoundingBox();

            // Compare with all other objects
            for (size_t j = i + 1; j < objects_.size(); ++j)
            {
                auto& objectB = objects_[j];
                auto& colliderB = objectB.collider();

                // Get the AABB of the second collider
                auto rangeB = colliderB.GetBoundingBox();

                // Check for AABB overlap
                if (math::Intersect(rangeA, rangeB))
                {
                    GameObjectPair pair{&objectA, &objectB};
                    new_potential_pairs[pair] = true;
                }
            }
        }

        // Update the potential pairs for narrow phase to process
        potential_pairs_ = std::move(new_potential_pairs);
    }



    void CollisionSystem::BroadPhase()
    {
        std::unordered_map<GameObjectPair, bool> new_potential_pairs;

        quadtree_->Clear();
        for (auto& object : objects_)
        {
            quadtree_->Insert(&object.collider());
        }

        // Use AABB tests for broad phase
        for (auto& object : objects_)
        {
            auto& collider = object.collider();
            // Get the AABB of the collider for broad phase test
            auto range = collider.GetBoundingBox();
            auto potentialColliders = quadtree_->Query(range);

            for (auto& otherCollider : potentialColliders)
            {
                if (&collider != otherCollider)
                {
                    // Avoid self-collision
                    // Only test AABB overlap in broad phase
                    if (math::Intersect(range, otherCollider->GetBoundingBox()))
                    {
                        GameObject* objectA = collider_to_object_map_[&collider];
                        GameObject* objectB = collider_to_object_map_[otherCollider];
                        if (objectA && objectB)
                        {
                            GameObjectPair pair{objectA, objectB};
                            new_potential_pairs[pair] = true;
                        }
                    }
                }
            }
        }

        // Update the potential pairs for narrow phase to process
        potential_pairs_ = std::move(new_potential_pairs);
    }

    void CollisionSystem::NarrowPhase()
    {
        std::unordered_set<GameObjectPair> newActivePairs;

        for (const auto& pair : potential_pairs_ | std::views::keys)
        {
            if (!pair.gameObjectA_ || !pair.gameObjectB_)
            {
                continue;
            }

            bool intersect = std::visit([](auto&& shape_a, auto&& shape_b)
                                        {
                                            return math::Intersect(shape_a, shape_b);
                                        }, pair.gameObjectA_->collider().shape(),
                                        pair.gameObjectB_->collider().shape());

            if (intersect)
            {
                newActivePairs.insert(pair);

                // If this is a new collision Keep for triggers?
                if (active_pairs_.find(pair) == active_pairs_.end())
                {
                    OnPairCollideStart(pair);
                }
                else
                {
                    OnPairCollideStay(pair);
                }
            }
        }

        for (const auto& pair : active_pairs_)
        {
            if (newActivePairs.find(pair) == newActivePairs.end())
            {
                OnPairCollideEnd(pair);
            }
        }
        active_pairs_ = std::move(newActivePairs);
    }

    //Called on the first collision frame
    void CollisionSystem::OnPairCollideStart(const GameObjectPair& pair)
    {
        if(!pair.gameObjectA_ || !pair.gameObjectB_){return;}

        pair.gameObjectA_->AddCollision();
        pair.gameObjectB_->AddCollision();

        if (pair.gameObjectA_->collider().is_trigger() || pair.gameObjectB_->collider().is_trigger())
        {
            pair.gameObjectA_->OnTriggerEnter();
            pair.gameObjectB_->OnTriggerEnter();
        }
        else
        {
            physics::ContactSolver ContactSolver;
            ContactSolver.SetContactObjects(pair);
            ContactSolver.ResolveContact();
            pair.gameObjectA_->OnCollisionEnter();
            pair.gameObjectB_->OnCollisionEnter();
        }
    }
    void CollisionSystem::OnPairCollideStay(const GameObjectPair& pair)
    {
        if(!pair.gameObjectA_ || !pair.gameObjectB_){return;}

        if (pair.gameObjectA_->collider().is_trigger() || pair.gameObjectB_->collider().is_trigger())
        {
            pair.gameObjectA_->OnTriggerStay();
            pair.gameObjectB_->OnTriggerStay();
        }
        else
        {
            //pair.gameObjectA_->OnCollisionStay();
            //pair.gameObjectB_->OnCollisionStay();
        }
    }

    void CollisionSystem::OnPairCollideEnd(const GameObjectPair& pair)
    {
        if (!pair.gameObjectA_ || !pair.gameObjectB_) return;

        pair.gameObjectA_->SubCollision();
        pair.gameObjectB_->SubCollision();

        if (pair.gameObjectA_->collider().is_trigger() || pair.gameObjectB_->collider().is_trigger())
        {
            if (pair.gameObjectA_->collisions_count() <= 0)
            {
                pair.gameObjectA_->OnTriggerExit();
            }
            if (pair.gameObjectB_->collisions_count() <= 0)
            {
                pair.gameObjectB_->OnTriggerExit();
            }
        }
        else
        {

            if (pair.gameObjectA_->collisions_count() <= 0)
            {
                pair.gameObjectA_->OnCollisionExit();
            }
            if (pair.gameObjectB_->collisions_count() <= 0)
            {
                pair.gameObjectB_->OnCollisionExit();
            }
        }
    }
}