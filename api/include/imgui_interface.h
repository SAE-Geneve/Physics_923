#ifndef PHYSICS_923_API_IMGUI_H_
#define PHYSICS_923_API_IMGUI_H_

#include <imgui_impl_sdl2.h>

#include "display.h"

namespace physics923
{
    class GameEngine;

    class ImGuiInterface
    {
    private:
        GameEngine* game_engine_ = nullptr;
        bool show_quadtree_ = true;
        float speed_multiplier_ = 1.0f;
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
        [[nodiscard]] float speed_multiplier() const { return speed_multiplier_; }
        [[nodiscard]] SDL_Color planets_colour() const { return planets_colour_; }
    };
}
#endif // PHYSICS_923_API_IMGUI_H_
