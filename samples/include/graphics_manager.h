#ifndef PHYSICS_SAMPLES_SHAPE_MANAGER_H_
#define PHYSICS_SAMPLES_SHAPE_MANAGER_H_

#include <SDL_render.h>
#include <vector>

#include "vec2.h"

namespace crackitos_physics::samples
{
    static constexpr size_t kCircleVertexCount = 20;

    class GraphicsManager
    {
    private:
        std::vector<SDL_Vertex> vertices_;
        std::vector<int> indices_;

    public:
        GraphicsManager() = default;
        ~GraphicsManager() = default;

        std::vector<SDL_Vertex>& vertices() { return vertices_; }
        std::vector<int>& indices() { return indices_; }

        void AddVertex(crackitos_core::math::Vec2f position, SDL_Color color);
        void Clear();
        void CreateCircle(crackitos_core::math::Vec2f centre, crackitos_core::commons::fp radius, SDL_Color color, bool rotation);
        void CreateAABB(crackitos_core::math::Vec2f min, crackitos_core::math::Vec2f max, SDL_Color color, bool fill_status);
        void CreateAABB(crackitos_core::math::Vec2f centre, crackitos_core::commons::fp half_size, SDL_Color color,
                        bool fill_status);
        void CreatePolygon(const std::vector<crackitos_core::math::Vec2f>& points, crackitos_core::math::Vec2f center, SDL_Color color,
                           bool fill_status);
    };
} // namespace samples
#endif // PHYSICS_SAMPLES_SHAPE_MANAGER_H_
