#pragma once
#include "vector.h"
#include "parse_scene.h"
#include "camera.h"
#include "shapes.h"
#include "lights.h"
#include "materials.h"

struct Scene {
    Scene(const ParsedScene &scene);
    Camera camera;
    int width, height;
    std::vector<Shape> shapes;
    std::vector<HW2Material> materials;
    std::vector<Light> lights;
    Vector3 background_color;
    int samples_per_pixel;
    AABB bvh;
    //WideAABB widebvh;
    // For the Triangle in the shapes to reference to.
    std::vector<HW2TriangleMesh> meshes;
};




