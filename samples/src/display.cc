#include "display.h"

#include <SDL.h>

namespace physics923
{
    Display::Display()
    {

        SDL_Init(SDL_INIT_VIDEO);
        window_ = SDL_CreateWindow("SDL2 Window",
            SDL_WINDOWPOS_CENTERED,
            SDL_WINDOWPOS_CENTERED,
            window_width_,
            window_height_,
            SDL_WINDOW_SHOWN);

        if (window_ == nullptr)
        {
            printf("Failed to create window: %s\n", SDL_GetError());
            exit(1);
        }

        renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED);

        if (renderer_ == nullptr)
        {
            printf("Failed to create renderer: %s\n", SDL_GetError());
            exit(1);
        }

        SDL_SetRenderDrawBlendMode(renderer_, SDL_BLENDMODE_BLEND);
    }

    Display::~Display()
    {
        SDL_DestroyRenderer(renderer_);
        SDL_DestroyWindow(window_);
        SDL_Quit();
    }

    void Display::Clear() const
    {
        SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 255);
        SDL_RenderClear(renderer_);
    }
}