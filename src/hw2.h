#pragma once

#include "image.h"
#include "parse_scene.h"
#include "hw2_types.h"
#include "bvh.h"


double hit_sphere(const Sphere &sphere, const Vector3 &ray, const Vector3 &origin);

bool get_tri_intersect(const Vector3 &p0, const Vector3 &p1, const Vector3 &p2, const Vector3 &ray, const Vector3 &lookfrom, const Real eps, Vector3 &coordinate, Real &t_val);

Vector3 get_color(const Scene &scene, const Vector3 &ray_in, const Vector3 &pt, const Real eps, const int obj_id, const bool with_bvh, const bool with_simd);
Vector3 get_color(const Scene &scene, const Vector3 &ray_in, const Vector3 &pt, const Real eps, Shape *hit_shape, const bool with_bvh, const bool with_simd);

Image3 hw_2_1(const std::vector<std::string> &params);
Image3 hw_2_2(const std::vector<std::string> &params);
Image3 hw_2_3(const std::vector<std::string> &params);
Image3 hw_2_4(const std::vector<std::string> &params);
Image3 hw_2_5(const std::vector<std::string> &params);
Image3 hw_2_7(const std::vector<std::string> &params);
