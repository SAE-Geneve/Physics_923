#ifndef PHYSICS_SAMPLES_IMGUI_H_
#define PHYSICS_SAMPLES_IMGUI_H_

#include <imgui_impl_sdl2.h>

#include "commons.h"
#include "display.h"

namespace crackitos_physics::samples
{
    class GameEngine;

    class ImGuiInterface
    {
    private:
        GameEngine* game_engine_ = nullptr;
        bool show_quadtree_ = true;
        crackitos_physics::commons::fp speed_multiplier_ = 1.0f;
        int current_scene_ = 0;

        SDL_Color planets_colour_ = {255, 13, 132};

    public:
        ImGuiInterface() = default;
        ~ImGuiInterface();

        void Initialize(Display* display, GameEngine* engine);
        void Update(bool& show_imgui);
        void Render(SDL_Renderer* renderer);
        void PassEvents(SDL_Event& event);

        [[nodiscard]] bool show_quadtree() const { return show_quadtree_; }
        [[nodiscard]] crackitos_physics::commons::fp speed_multiplier() const { return speed_multiplier_; }
        [[nodiscard]] SDL_Color planets_colour() const { return planets_colour_; }
    };
} // namespace samples
#endif // PHYSICS_SAMPLES_IMGUI_H_
