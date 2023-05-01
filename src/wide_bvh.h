#pragma once

#include "hw2_types.h"


struct WideAABB {
    Vector3 a, b;
    std::vector<Shape*> shapes;
    std::shared_ptr<WideAABB> c1, c2, c3, c4;
    Real surface_area;
};



inline bool wide_aabb_x_compare (const std::shared_ptr<WideAABB> &a, const std::shared_ptr<WideAABB> &b) {
    return a->a.x < b->a.x;
}
inline bool wide_aabb_y_compare (const std::shared_ptr<WideAABB> &a, const std::shared_ptr<WideAABB> &b) {
    return a->a.y < b->a.y;
}
inline bool wide_aabb_z_compare (const std::shared_ptr<WideAABB> &a, const std::shared_ptr<WideAABB> &b) {
    return a->a.z < b->a.z;
}

inline bool wide_aabb_x_compare_max (const std::shared_ptr<WideAABB> &a, const std::shared_ptr<WideAABB> &b) {
    return a->b.x < b->b.x;
}
inline bool wide_aabb_y_compare_max (const std::shared_ptr<WideAABB> &a, const std::shared_ptr<WideAABB> &b) {
    return a->b.y < b->b.y;
}
inline bool wide_aabb_z_compare_max (const std::shared_ptr<WideAABB> &a, const std::shared_ptr<WideAABB> &b) {
    return a->b.z < b->b.z;
}