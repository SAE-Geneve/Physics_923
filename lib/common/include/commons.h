#ifndef PHYSICS_923_LIB_COMMON_COMMONS_H_
#define PHYSICS_923_LIB_COMMON_COMMONS_H_

namespace physics923::commons
{
    using fp = float;
    constexpr static fp Pi = 3.14159265358979323846f;
    constexpr static fp Epsilon = 0.000001f;


    template <typename T>
    [[nodiscard]] constexpr bool Approx(T value, T target)
    {
        return abs(value - target) <= Epsilon;
    }

} // namespace common
#endif // PHYSICS_923_LIB_COMMON_COMMONS_H_
