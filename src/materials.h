#pragma once
#include "vector.h"
#include <variant>
#include <filesystem>

enum material_e {
    DiffuseType,
    MirrorType
};

struct SolidTexture {
    Vector3 reflectance;
};
struct ImgTexture {
    ImgTexture(const fs::path &path, const Real us, const Real vs, const Real uo, const Real vo);
    int width, height;
    int channels;
    Real uoffset, voffset;
    Real uscale, vscale;
    unsigned char* data;
};

using Texture = std::variant<SolidTexture, ImgTexture>;

struct Material {
    Texture reflectance;
    material_e material_type;
};
struct HW2Material {
    Vector3 reflectance;
    material_e material_type;
};

Vector3 get_texture_kd(const Texture &texture, const Vector2 uv);