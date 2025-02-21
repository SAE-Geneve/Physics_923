#ifndef PHYSICS_923_API_PLANET_SYSTEM_H_
#define PHYSICS_923_API_PLANET_SYSTEM_H_

#include <vector>

#include "body.h"
#include "game_object.h"
#include "vec2.h"

namespace physics923
{
    class PlanetSystem
    {
    private:
        static constexpr float kGravitationConstant_ = 0.0667f;
        static constexpr std::size_t kStartingPlanetsCount_ = 20;

        float star_mass_ = 10.f;
        float planet_mass_ = 3.f;

        physics::Body star_;
        std::vector<GameObject> planets_{};

        bool is_spawner_active_ = false;

    public:
        PlanetSystem() = default;
        ~PlanetSystem() = default;

        void Initialize();
        void Update(float delta_time, SDL_Color colour);
        void Clear();

        void CreatePlanet(math::Vec2f position, float radius, SDL_Color color);
        void UpdatePlanets(float delta_time);
        void UpdatePlanetsSIMD(float delta_time);
        void SpawnPlanets(SDL_Color colour);

        std::vector<GameObject> planets() { return planets_; }
        physics::Body* star() { return &star_; }

        void ToggleSpawner() { is_spawner_active_ = !is_spawner_active_; };
    };
}
#endif // PHYSICS_923_API_PLANET_SYSTEM_H_
