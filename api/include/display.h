#ifndef PHYSICS_923_API_DISPLAY_H_
#define PHYSICS_923_API_DISPLAY_H_

#include <SDL_render.h>
#include <SDL_video.h>
#include "commons.h"

namespace physics923
{
    static constexpr int kWindowWidth = 1200;
    static constexpr int kWindowHeight = 800;


    class Display
    {
    private:
        SDL_Window* window_;
        SDL_Renderer* renderer_;

        int window_width_ = kWindowWidth;
        int window_height_ = kWindowHeight;

    public:
        Display();
        ~Display();

        [[nodiscard]] SDL_Window* window() const { return window_; }
        [[nodiscard]] SDL_Renderer* renderer() const { return renderer_; }

        void Clear() const;
    };
}
#endif // PHYSICS_923_API_DISPLAY_H_
