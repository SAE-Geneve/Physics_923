#include "planet_system.h"

#include <SDL_mouse.h>

#include "display.h"
#include "four_vec2.h"
#include "random.h"
#include "distance.h"

namespace crackitos_physics::samples {
void PlanetSystem::Initialize() {
  Clear();
  star_ = physics::Body(physics::BodyType::Static,
                        crackitos_core::math::Vec2f(kWindowWidth / 2.f, kWindowHeight / 2.f),
                        crackitos_core::math::Vec2f::Zero(),
                        false,
                        star_mass_
  );

  for (std::size_t i = 0; i < kStartingPlanetsCount_; i++) {
    constexpr crackitos_core::commons::fp margin = 20.0f;
    const crackitos_core::math::Vec2f position(crackitos_core::random::Range(margin, kWindowWidth - margin),
                               crackitos_core::random::Range(margin, kWindowHeight - margin));
    const crackitos_core::commons::fp radius = crackitos_core::random::Range(5.f, 20.f);
    const uint8_t alpha = crackitos_core::random::Range(10, 255);

    CreatePlanet(position, radius, SDL_Color{255, 13, 132, alpha});
  }

  is_spawner_active_ = false;
}

void PlanetSystem::Update(crackitos_core::commons::fp delta_time, SDL_Color colour) {
  if (is_spawner_active_) {
    SpawnPlanets(colour);
  }
  UpdatePlanetsSIMD(delta_time);
}

void PlanetSystem::Clear() {
  // Clear the vector of planets
  planets_.clear();

  // Reset the star
  star_ = physics::Body();

  // Deactivate the spawner
  is_spawner_active_ = false;
}

void PlanetSystem::CreatePlanet(const crackitos_core::math::Vec2f position, const crackitos_core::commons::fp radius,
                                const SDL_Color color) {
  crackitos_core::math::Vec2f u = star_.position() - position;
  crackitos_core::commons::fp r = u.Magnitude();

  if (r > 0) {
    // Calculate angular velocity magnitude based on gravitational force
    crackitos_core::math::Vec2f tangential_direction = crackitos_core::math::Vec2f(-u.y, u.x).Normalized();
    crackitos_core::commons::fp orbital_velocity = std::sqrt(kGravitationConstant_ * (star_mass_ / r));

    // Calculate angular velocity
    crackitos_core::math::Vec2f angular_velocity = tangential_direction * orbital_velocity;
    physics::Body body(physics::BodyType::Dynamic, position,
                       angular_velocity,
                       false,
                       planet_mass_
    );
    //crackitos_core::random mass: crackitos_core::random::Range(1.0f, 50.0f)

    auto planet = GameObject(body, radius, color);
    planets_.push_back(planet);
  }
}

void PlanetSystem::UpdatePlanets(crackitos_core::commons::fp delta_time) {
  for (auto &planet : planets_) {
    crackitos_core::math::Vec2f u = star_.position() - planet.position();
    const crackitos_core::commons::fp r = u.Magnitude();

    if (r > 0) {
      const crackitos_core::commons::fp force_magnitude = kGravitationConstant_ * (planet_mass_ *
          star_mass_ / (r * r));
      const crackitos_core::math::Vec2f force = force_magnitude * u.Normalized();
      planet.body().ApplyForce(force);
    }

    planet.body().Update(delta_time);
  }
}

void PlanetSystem::UpdatePlanetsSIMD(crackitos_core::commons::fp delta_time) {
  const std::size_t simdSize = planets_.size() / 4 * 4;

  for (std::size_t i = 0; i < simdSize; i += 4) {
    //Load four planet positions and calculate vector u to the star for each
    crackitos_core::math::FourVec2f planetPositions = {
        planets_[i].position(),
        planets_[i + 1].position(),
        planets_[i + 2].position(),
        planets_[i + 3].position()
    };

    crackitos_core::math::FourVec2f u = crackitos_core::math::FourVec2f({
                                            star_.position(), star_.position(), star_.position(), star_.position()
                                        }) - planetPositions;

    //Calculate crackitos_core::distance to the star and force magnitudes
    std::array<crackitos_core::commons::fp, 4> distances = u.Magnitude();
    crackitos_core::math::FourVec2f normalizedU = u.Normalize();

    std::array<crackitos_core::commons::fp, 4> forceMagnitudes = {};
    for (int j = 0; j < 4; ++j) {
      if (distances[j] > 0) {
        forceMagnitudes[j] = kGravitationConstant_ * (planets_[i + j].body().mass() * star_mass_ / (
            distances[j] * distances[j]));
      } else {
        forceMagnitudes[j] = 0.0f; // Handle edge case for zero crackitos_core::distance
      }
    }

    // Apply forces and update position/velocity for each planet
    crackitos_core::math::FourVec2f forces = normalizedU * forceMagnitudes;
    for (int j = 0; j < 4; ++j) {
      planets_[i + j].body().ApplyForce(crackitos_core::math::Vec2f(forces.x[j], forces.y[j]));
      planets_[i + j].body().Update(delta_time);
    }
  }

  // Handle any remaining planets that weren't in a set of four
  for (std::size_t i = simdSize; i < planets_.size(); ++i) {
    crackitos_core::math::Vec2f u = star_.position() - planets_[i].position();
    const crackitos_core::commons::fp r = u.Magnitude();
    if (r > 0) {
      const crackitos_core::commons::fp force_magnitude = kGravitationConstant_ * (planets_[i].body().
          mass() * star_mass_ / (r * r));
      const crackitos_core::math::Vec2f force = force_magnitude * u.Normalized();
      planets_[i].body().ApplyForce(force);
    }
    planets_[i].body().Update(delta_time);
  }
}

void PlanetSystem::SpawnPlanets(SDL_Color colour) {
  crackitos_core::math::Vec2i mouse_pos;
  SDL_GetMouseState(&mouse_pos.x, &mouse_pos.y);
  const auto mouse_pos_f = crackitos_core::math::Vec2f(static_cast<crackitos_core::commons::fp>(mouse_pos.x),
                                       static_cast<crackitos_core::commons::fp>(mouse_pos.y));

  auto distance = crackitos_core::distance::Pixel((mouse_pos_f - star_.position()).Magnitude());
  auto minimum_distance = crackitos_core::distance::Pixel(30);

  if (constexpr crackitos_core::commons::fp minimum_range = 30;
      crackitos_core::distance::Convert<crackitos_core::distance::Pixel, crackitos_core::distance::Meter>(distance).value
          > crackitos_core::distance::Convert<crackitos_core::distance::Pixel, crackitos_core::distance::Meter>(minimum_distance).value) {
    const crackitos_core::math::Vec2f random_pos(crackitos_core::random::Range(mouse_pos_f.x - minimum_range, mouse_pos_f.x + minimum_range),
                                 crackitos_core::random::Range(mouse_pos_f.y - minimum_range, mouse_pos_f.y + minimum_range));

    const crackitos_core::commons::fp radius = crackitos_core::random::Range(5.f, 20.f);
    const uint8_t alpha = crackitos_core::random::Range(10, 255);
    colour.a = alpha;

    CreatePlanet(random_pos, radius, colour);
  }
}
} // namespace samples