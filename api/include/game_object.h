#ifndef PHYSICS_923_API_GAME_OBJECT_H_
#define PHYSICS_923_API_GAME_OBJECT_H_

#include <SDL_pixels.h>

#include <utility>

#include "body.h"
#include "collider.h"
#include "random.h"

namespace physics923
{
    class GameObject
    {
    private:
        physics::Body body_;
        physics::Collider collider_ = {};
        float radius_ = 0.f;
        SDL_Color color_ = SDL_Color{255, 13, 132, 255};
        int collisions_count_ = 0;

    public:
        GameObject() = default;

        GameObject(const physics::Body& body, physics::Collider collider, const float radius) : body_(body),
            collider_(std::move(collider)), radius_(radius)
        {
        }

        GameObject(const physics::Body& body, const float radius, const SDL_Color& color) : body_(body),
            radius_(radius), color_(color)
        {
        }

        ~GameObject() = default;

        [[nodiscard]] physics::Body& body() { return body_; }
        [[nodiscard]] physics::Collider& collider() { return collider_; }
        [[nodiscard]] float radius() const { return radius_; }
        [[nodiscard]] SDL_Color color() const { return color_; }
        [[nodiscard]] math::Vec2f position() const { return body_.position(); }
        [[nodiscard]] int collisions_count() const { return collisions_count_; }

        void set_body(const physics::Body& body) { body_ = body; }
        void set_collider(const physics::Collider& collider) { collider_ = collider; }
        void set_radius(const float radius) { radius_ = radius; }
        void set_color(const SDL_Color& color) { color_ = color; }
        void AddCollision() { collisions_count_++; }
        void SubCollision() { collisions_count_--; }

        void OnTriggerEnter()
        {
            color_ = SDL_Color{0, 255, 0, 255};
        }

        void OnTriggerStay()
        {
            color_ = SDL_Color{0, 255, 0, 255};
        }

        void OnTriggerExit()
        {
            color_ = SDL_Color{255, 13, 132, 255};
        }

        void OnCollisionEnter()
        {
            Uint8 r = random::Range(128, 255);
            Uint8 g = random::Range(128, 255);
            Uint8 b = random::Range(128, 255);
            color_ = SDL_Color{r, g, b, 255};
        }

        void OnCollisionExit()
        {
            //color_ = SDL_Color{ 255, 13, 132, 255 };
        }
    };

    struct GameObjectPair
    {
        GameObject* gameObjectA_;
        GameObject* gameObjectB_;

        bool operator==(const GameObjectPair& other) const
        {
            return (gameObjectA_ == other.gameObjectA_ && gameObjectB_ == other.gameObjectB_) ||
                (gameObjectA_ == other.gameObjectB_ && gameObjectB_ == other.gameObjectA_);
        }
    };
}

    namespace std
    {
        template <>
        struct std::hash<physics923::GameObjectPair>
        {
            std::size_t operator()(const physics923::GameObjectPair& pair) const noexcept
            {
                // Ensure consistent ordering by hashing the pointers in a sorted manner
                const physics923::GameObject* first = pair.gameObjectA_ < pair.gameObjectB_ ? pair.gameObjectA_ : pair.gameObjectB_;
                const physics923::GameObject* second = pair.gameObjectA_ < pair.gameObjectB_
                                                           ? pair.gameObjectB_
                                                           : pair.gameObjectA_;

                std::size_t h1 = std::hash<const physics923::GameObject*>{}(first);
                std::size_t h2 = std::hash<const physics923::GameObject*>{}(second);
                // Combine hashes
                return h1 ^ (h2 << 1);
            }
        };
    };
#endif // PHYSICS_923_API_GAME_OBJECT_H_
