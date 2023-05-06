#include "materials.h"
#include "3rdparty/stb_image.h"

ImgTexture::ImgTexture(const fs::path &path, const Real us, const Real vs, const Real uo, const Real vo): 
    uoffset(uo),
    voffset(vo),
    uscale(us),
    vscale(vs)
{
    stbi_load(path.c_str(), &width, &height, &channels, 0);
}

Vector3 get_texture_kd(const Texture &texture, const Vector2 uv){
    if (auto *solid = std::get_if<SolidTexture>(&texture)){
        return solid->reflectance;
    } else if (auto *img = std::get_if<ImgTexture>(&texture)){
        // bilinear interpolation
        Real x = Real(img->width) * modulo(img->uscale * uv.x + img->uoffset, 1.0);
        Real y = Real(img->height) * modulo(img->vscale * uv.y + img->voffset, 1.0);
        int x1 = std::floor(x); int x2 = std::ceil(x);
        int y1 = std::floor(y); int y2 = std::ceil(y);
        Vector3 q11 = Vector3{
             img->data[y1*img->channels*img->width+x1*img->channels],
             img->data[y1*img->channels*img->width+x1*img->channels+1],
             img->data[y1*img->channels*img->width+x1*img->channels+2]
        };
        Vector3 q21 = Vector3{
            img->data[y1*img->channels*img->width+x2*img->channels],
            img->data[y1*img->channels*img->width+x2*img->channels+1],
            img->data[y1*img->channels*img->width+x2*img->channels+2]
        };
        Vector3 q12 = Vector3{
            img->data[y2*img->channels*img->width+x1*img->channels],
            img->data[y2*img->channels*img->width+x1*img->channels+1],
            img->data[y2*img->channels*img->width+x1*img->channels+2]
        };
        Vector3 q22 = Vector3{
            img->data[y2*img->channels*img->width+x2*img->channels],
            img->data[y2*img->channels*img->width+x2*img->channels+1],
            img->data[y2*img->channels*img->width+x2*img->channels+2]
        };
        Vector3 fxy1 = ((Real(x2) - x)/1.0 *q11) +( (x - Real(x1))/1.0 * q21);
        Vector3 fxy2 = ((Real(x2) - x)/1.0 *q12) +( (x - Real(x1))/1.0 * q22);
        return ((Real(y2)-y)/1.0 * fxy1 )+ ((y - Real(y1))/1.0 * fxy2);
    } else {
        assert(false);
    }
}