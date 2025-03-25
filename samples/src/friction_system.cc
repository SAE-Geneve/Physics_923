// #include "friction_system.h"
//
// #include <iostream>
// #include <ranges>
//
// #include "contact_solver.h"
// #include "random.h"
//
// namespace crackitos_physics::samples
// {
//     FrictionSystem::FrictionSystem(): quadtree_(math::AABB(math::Vec2f(0, 0), math::Vec2f(kWindowWidth, kWindowHeight)))
//     {
//     }
//
//     FrictionSystem::~FrictionSystem()
//     {
//         Clear();
//     }
//
//     void FrictionSystem::Initialize()
//     {
//         Clear();
//         objects_.reserve(1000);
//         CreateGround();
//     }
//
//     void FrictionSystem::Clear()
//     {
//         objects_.clear();
//         potential_pairs_.clear();
//         active_pairs_.clear();
//         collider_to_object_map_.clear();
//     }
//
//
//     void FrictionSystem::SpawnShape(const math::Vec2f pos, const math::ShapeType type)
//     {
//         const size_t i = objects_.size();
//         math::Vec2f new_position = pos;
//         const crackitos_physics::commons::fp radius = random::Range(5.f, 20.f);
//
//         switch (type)
//         {
//         case math::ShapeType::kAABB:
//             {
//                 const crackitos_physics::commons::fp half_size_x = random::Range(5.f, 20.f);
//                 const crackitos_physics::commons::fp half_size_y = radius;
//                 const auto half_size_vec = math::Vec2f(half_size_x, half_size_y);
//                 const auto half_size_length = half_size_vec.Magnitude();
//                 math::AABB aabb(new_position, half_size_vec, half_size_length);
//                 CreateObject(i, aabb);
//             }
//             break;
//         case math::ShapeType::kCircle:
//             {
//                 math::Circle circle(new_position, radius);
//                 CreateObject(i, circle);
//             }
//             break;
//         case math::ShapeType::kPolygon:
//         case math::ShapeType::kNone:
//         default:
//             break;
//         }
//     }
//
//     void FrictionSystem::CreateObject(size_t index, math::Circle& circle)
//     {
//         math::Vec2f velocity(0.0f, 0.0f);
//         physics::Body body(physics::BodyType::Dynamic,
//                            circle.centre(),
//                            velocity,
//                            gravity,
//                            true,
//                            random::Range(50.f, 100.f)
//         );
//         physics::Collider collider(circle, random::Range(0.5f, 0.9f), 0.1f, false);
//         GameObject object(body, collider, circle.radius());
//
//         objects_.push_back(object);
//         RegisterObject(objects_[index]);
//     }
//
//     void FrictionSystem::CreateObject(size_t index, math::AABB& aabb)
//     {
//         math::Vec2f velocity(0.0f, 0.0f);
//         physics::Body body(physics::BodyType::Dynamic,
//                            aabb.GetCentre(),
//                            velocity,
//                            gravity,
//                            true,
//                            random::Range(50.f, 100.f)
//         );
//         physics::Collider collider(aabb,
//                                    random::Range(0.0f, 0.0f),
//                                    0.5f,
//                                    false);
//         GameObject object(body, collider, aabb.half_size_length());
//
//         objects_.push_back(object);
//         RegisterObject(objects_[index]);
//     }
//
//     void FrictionSystem::CreateGround()
//     {
//         size_t i = objects_.size();
//
//         math::AABB ground(math::Vec2f(360.0f, 650.0f), math::Vec2f(840.0f, 750.0f));
//
//         math::Vec2f velocity(0.0f, 0.0f);
//         physics::Body body(physics::BodyType::Static,
//                            ground.GetCentre(),
//                            velocity,
//                            gravity,
//                            false,
//                            0.0f);
//         physics::Collider collider(ground, 0.0f, 0.0f, false);
//         GameObject object(body, collider, ground.half_size_length());
//
//
//         objects_.push_back(object);
//         RegisterObject(objects_[i]);
//     }
//
//     void FrictionSystem::DeleteObject(const size_t index)
//     {
//         if (index >= objects_.size()) return;
//
//         GameObject& object = objects_[index];
//         UnregisterObject(object);
//         objects_.erase(objects_.begin() + index);
//     }
//
//     void FrictionSystem::RemoveOutOfBoundsObjects()
//     {
//         for (size_t i = objects_.size(); i-- > 0;) // Reverse iteration
//         {
//             const auto& bounding_box = objects_[i].collider().GetBoundingBox();
//
//             // Check if the object is out of bounds (left, right, or bottom)
//             if (bounding_box.min_bound().x < frame_bounds_.min_bound().x ||
//                 bounding_box.max_bound().x > frame_bounds_.max_bound().x ||
//                 bounding_box.max_bound().y > frame_bounds_.max_bound().y)
//             {
//                 DeleteObject(i);
//             }
//         }
//     }
//
//
//     void FrictionSystem::RegisterObject(GameObject& object)
//     {
//         collider_to_object_map_[&object.collider()] = &object;
//     }
//
//     void FrictionSystem::UnregisterObject(GameObject& object)
//     {
//         const auto collider = &object.collider();
//
//         // Remove collider from collider_to_object_map_
//         collider_to_object_map_.erase(collider);
//
//         // Remove related pairs in active_pairs_ and potential_pairs_
//         for (auto it = active_pairs_.begin(); it != active_pairs_.end();)
//         {
//             if (it->gameObjectA_ == &object || it->gameObjectB_ == &object)
//                 it = active_pairs_.erase(it);
//             else
//                 ++it;
//         }
//
//         for (auto it = potential_pairs_.begin(); it != potential_pairs_.end();)
//         {
//             if (it->first.gameObjectA_ == &object || it->first.gameObjectB_ == &object)
//                 it = potential_pairs_.erase(it);
//             else
//                 ++it;
//         }
//     }
//
//
//     void FrictionSystem::Update(const crackitos_physics::commons::fp delta_time)
//     {
//         UpdateShapes(delta_time);
//         RemoveOutOfBoundsObjects();
//         BroadPhase();
//         NarrowPhase();
//     }
//
//     void FrictionSystem::UpdateShapes(const crackitos_physics::commons::fp delta_time)
//     {
//         for (auto& object : objects_)
//         {
//             auto& body = object.body();
//             auto& collider = object.collider();
//
//             body.Update(delta_time);
//
//             const auto position = body.position();
//
//             //Update the collider's position
//             collider.UpdatePosition(position);
//         }
//     }
//
//     void FrictionSystem::SimplisticBroadPhase()
//     {
//         std::unordered_map<GameObjectPair, bool> new_potential_pairs;
//
//         // Loop through all objects
//         for (size_t i = 0; i < objects_.size(); ++i)
//         {
//             auto& objectA = objects_[i];
//             auto& colliderA = objectA.collider();
//
//             // Get the AABB of the first collider
//             auto rangeA = colliderA.GetBoundingBox();
//
//             // Compare with all other objects
//             for (size_t j = i + 1; j < objects_.size(); ++j)
//             {
//                 auto& objectB = objects_[j];
//                 auto& colliderB = objectB.collider();
//
//                 // Get the AABB of the second collider
//                 auto rangeB = colliderB.GetBoundingBox();
//
//                 // Check for AABB overlap
//                 if (math::Intersect(rangeA, rangeB))
//                 {
//                     GameObjectPair pair{&objectA, &objectB};
//                     new_potential_pairs[pair] = true;
//                 }
//             }
//         }
//
//         // Update the potential pairs for narrow phase to process
//         potential_pairs_ = std::move(new_potential_pairs);
//     }
//
//     void FrictionSystem::BroadPhase()
//     {
//         std::unordered_map<GameObjectPair, bool> new_potential_pairs;
//
//         quadtree_.Clear();
//
//         //Insert static objects first
//         for (auto& object : objects_)
//         {
//             if (object.body().type() == physics::BodyType::Static)
//             {
//                 quadtree_.Insert(&object.collider());
//             }
//         }
//
//         //Then insert all other objects
//         for (auto& object : objects_)
//         {
//             if (object.body().type() != physics::BodyType::Static)
//             {
//                 quadtree_.Insert(&object.collider());
//             }
//         }
//
//         // Use AABB tests for broad phase
//         for (auto& object : objects_)
//         {
//             auto& collider = object.collider();
//             // Get the AABB of the collider for broadphase test
//             auto range = collider.GetBoundingBox();
//             auto potentialColliders = quadtree_.Query(range);
//
//             for (auto& otherCollider : potentialColliders)
//             {
//                 // Avoid self-collision
//                 if (&collider != otherCollider)
//                 {
//                     GameObject* objectA = collider_to_object_map_[&collider];
//                     GameObject* objectB = collider_to_object_map_[otherCollider];
//                     if (objectA && objectB)
//                     {
//                         GameObjectPair pair{objectA, objectB};
//                         new_potential_pairs[pair] = true;
//                     }
//                 }
//             }
//         }
//
//         // Update the potential pairs for narrow phase to process
//         potential_pairs_ = std::move(new_potential_pairs);
//     }
//
//     void FrictionSystem::NarrowPhase()
//     {
//         std::unordered_set<GameObjectPair> newActivePairs;
//
//         for (const auto& pair : potential_pairs_ | std::views::keys)
//         {
//             if (!pair.gameObjectA_ || !pair.gameObjectB_) continue;
//
//             bool is_new_pair = active_pairs_.find(pair) == active_pairs_.end();
//             ResolveCollisionPair(pair, is_new_pair);
//
//             newActivePairs.insert(pair);
//         }
//
//         for (const auto& pair : active_pairs_)
//         {
//             if (newActivePairs.find(pair) == newActivePairs.end())
//             {
//                 OnPairCollideEnd(pair);
//             }
//         }
//
//         active_pairs_ = std::move(newActivePairs);
//     }
//
//
//     void FrictionSystem::ResolveCollisionPair(const GameObjectPair& pair, bool is_new_pair)
//     {
//         if (!pair.gameObjectA_ || !pair.gameObjectB_) return;
//
//         physics::ContactSolver solver;
//         solver.SetContactObjects(
//             {&pair.gameObjectA_->body(), &pair.gameObjectB_->body()},
//             {&pair.gameObjectA_->collider(), &pair.gameObjectB_->collider()}
//         );
//         solver.CalculateProperties();
//
//         if (solver.penetration_ <= 0.0f) return;
//
//         // Trigger behavior
//         if (pair.gameObjectA_->collider().is_trigger() || pair.gameObjectB_->collider().is_trigger())
//         {
//             if (is_new_pair)
//             {
//                 pair.gameObjectA_->OnTriggerEnter();
//                 pair.gameObjectB_->OnTriggerEnter();
//             }
//             else
//             {
//                 pair.gameObjectA_->OnTriggerStay();
//                 pair.gameObjectB_->OnTriggerStay();
//             }
//         }
//         else
//         {
//             if (is_new_pair)
//             {
//                 pair.gameObjectA_->AddCollision();
//                 pair.gameObjectB_->AddCollision();
//                 solver.ResolveContact();
//                 pair.gameObjectA_->OnCollisionEnter();
//                 pair.gameObjectB_->OnCollisionEnter();
//             }
//             else
//             {
//                 solver.ResolveContact(); // Optional: resolve every frame
//                 // Optionally add OnCollisionStay()
//                 // pair.gameObjectA_->OnCollisionStay();
//                 // pair.gameObjectB_->OnCollisionStay();
//             }
//         }
//
//         active_pairs_.insert(pair);
//     }
//
//
//     void FrictionSystem::OnPairCollideEnd(const GameObjectPair& pair)
//     {
//         if (!pair.gameObjectA_ || !pair.gameObjectB_) return;
//
//         pair.gameObjectA_->SubCollision();
//         pair.gameObjectB_->SubCollision();
//
//         if (pair.gameObjectA_->collider().is_trigger() || pair.gameObjectB_->collider().is_trigger())
//         {
//             if (pair.gameObjectA_->collisions_count() <= 0)
//             {
//                 pair.gameObjectA_->OnTriggerExit();
//             }
//             if (pair.gameObjectB_->collisions_count() <= 0)
//             {
//                 pair.gameObjectB_->OnTriggerExit();
//             }
//         }
//         else
//         {
//             if (pair.gameObjectA_->collisions_count() <= 0)
//             {
//                 pair.gameObjectA_->OnCollisionExit();
//             }
//             if (pair.gameObjectB_->collisions_count() <= 0)
//             {
//                 pair.gameObjectB_->OnCollisionExit();
//             }
//         }
//     }
// } // namespace samples
