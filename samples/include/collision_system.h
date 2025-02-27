#ifndef PHYSICS_923_API_COLLISION_SYSTEM_H_
#define PHYSICS_923_API_COLLISION_SYSTEM_H_

#include <unordered_map>

#include "game_object.h"
#include "conversions.h"
#include "quadtree.h"
#include "trigger_system.h"

namespace physics923
{

class CollisionSystem
{
private:
    int number_of_objects_ = 200;
    std::array<GameObject, 200> objects_ = {};
    physics::Quadtree quadtree_;

    std::unordered_map<GameObjectPair, bool> potential_pairs_;
    std::unordered_set<GameObjectPair> active_pairs_;
    //Mapping from Collider to GameObject
    std::unordered_map<physics::Collider*, GameObject*> collider_to_object_map_;

public:
    CollisionSystem();
    ~CollisionSystem() = default;

    void Initialize();
    void Clear();

    std::array<GameObject, 200> objects() { return objects_; }
    [[nodiscard]] physics::Quadtree& quadtree() { return quadtree_; }

    void CreateObject(size_t index, math::Circle& circle);
    void CreateObject(size_t index, math::AABB& aabb);
    //void CreateObject(size_t index, math::Polygon& polygon);
    void DeleteObject(size_t index);

    void RegisterObject(GameObject& object);
    void UnregisterObject(GameObject& object);

    void Update(physics923::commons::fp delta_time);
    void UpdateShapes(physics923::commons::fp delta_time);

    void SimplisticBroadPhase();
    void BroadPhase();
    void NarrowPhase();

    static void OnPairCollideStart(const GameObjectPair& pair);
    static void OnPairCollideStay(const GameObjectPair& pair);
    static void OnPairCollideEnd(const GameObjectPair& pair);
};
    }
#endif // PHYSICS_923_API_COLLISION_SYSTEM_H_
