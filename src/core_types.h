#pragma once

#include "vector.h"
#include <variant>
#include <vector>

enum material_e {
    DiffuseType,
    MirrorType
};

struct TriangleMesh {
    int material_id = -1;
    std::vector<Vector3> positions;
    std::vector<Vector3i> indices;
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


struct Material {
    Vector3 reflectance;
    material_e material_type;
};

struct Light {
    Vector3 position;
    Vector3 intensity;
};

