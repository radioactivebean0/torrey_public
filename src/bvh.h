#pragma once
#include "hw2_types.h"

inline bool aabb_x_compare (const std::shared_ptr<AABB> &a, const std::shared_ptr<AABB> &b) {
    return a->a.x < b->a.x;
}
inline bool aabb_y_compare (const std::shared_ptr<AABB> &a, const std::shared_ptr<AABB> &b) {
    return a->a.y < b->a.y;
}
inline bool aabb_z_compare (const std::shared_ptr<AABB> &a, const std::shared_ptr<AABB> &b) {
    return a->a.z < b->a.z;
}

inline bool aabb_x_compare_max (const std::shared_ptr<AABB> &a, const std::shared_ptr<AABB> &b) {
    return a->b.x < b->b.x;
}
inline bool aabb_y_compare_max (const std::shared_ptr<AABB> &a, const std::shared_ptr<AABB> &b) {
    return a->b.y < b->b.y;
}
inline bool aabb_z_compare_max (const std::shared_ptr<AABB> &a, const std::shared_ptr<AABB> &b) {
    return a->b.z < b->b.z;
}
AABB build_bvh(std::vector<Shape> &shapes, int split_method);

bool hit_box(const AABB &box, const Vector3 &ray, const Vector3 &lookfrom, Real t_min, Real t_max);

bool hit_bvh(const AABB &bvh, const Vector3 &ray, const Vector3 &ray_origin, const Real eps, Shape **hit_shape, Real &t_val, Vector3 &b_coord, bool with_simd);

