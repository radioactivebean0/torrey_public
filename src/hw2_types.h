#pragma once

#include "parse_scene.h"

enum material_e {
    DiffuseType,
    MirrorType
};

struct TriangleMesh {
    int material_id = -1;
    std::vector<Vector3> positions;
    std::vector<Vector3i> indices;
};

struct Camera {
    Vector3 lookfrom;
    Vector3 lookat;
    Vector3 up;
    Real vfov;
    int width, height;
};

struct Sphere {
    Vector3 center;
    Real radius;
    int material_id;
};

struct Triangle{
    int face_index;
    const TriangleMesh *mesh;
};

using Shape = std::variant<Sphere, Triangle>;

struct AABB {
    Vector3 a, b;
    int axis;
    std::vector<Shape*> shapes;
    std::shared_ptr<AABB> left, right;
};


struct Material {
    Vector3 reflectance;
    material_e material_type;
};

struct Light {
    Vector3 position;
    Vector3 intensity;
};

struct Scene {
    Scene(const ParsedScene &scene);
    Camera camera;
    int width, height;
    std::vector<Shape> shapes;
    std::vector<Material> materials;
    std::vector<Light> lights;
    Vector3 background_color;
    int samples_per_pixel;
    AABB bvh;
    //WideAABB widebvh;
    // For the Triangle in the shapes to reference to.
    std::vector<TriangleMesh> meshes;
};