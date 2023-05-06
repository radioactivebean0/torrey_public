#include "hw3_scene.h"
#include "bvh2.h"
Scene::Scene(const ParsedScene &scene) :
        camera(from_parsed_camera(scene.camera)),
        width(scene.camera.width),
        height(scene.camera.height),
        background_color(scene.background_color),
        samples_per_pixel(scene.samples_per_pixel){
    // Extract triangle meshes from the parsed scene.
    int tri_mesh_count = 0;
    for (const ParsedShape &parsed_shape : scene.shapes) {
        if (std::get_if<ParsedTriangleMesh>(&parsed_shape)) {
            tri_mesh_count++;
        }
    }
    meshes.resize(tri_mesh_count);
    // Extract the shapes
    tri_mesh_count = 0;
    for (int i = 0; i < (int)scene.shapes.size(); i++) {
        const ParsedShape &parsed_shape = scene.shapes[i];
        if (auto *sph = std::get_if<ParsedSphere>(&parsed_shape)) {
            shapes.push_back(Sphere{
                .center = sph->position,
                .material_id = sph->material_id,
                .radius = sph->radius
            });
        } else if (auto *parsed_mesh = std::get_if<ParsedTriangleMesh>(&parsed_shape)) {
            meshes[tri_mesh_count] = TriangleMesh{.material_id= parsed_mesh->material_id,
                                                  .positions= parsed_mesh->positions,
                                                  .indices= parsed_mesh->indices, 
                                                  .normals = parsed_mesh->normals,
                                                  .uvs = parsed_mesh->uvs};
            // Extract all the individual triangles
            for (int face_index = 0; face_index < (int)parsed_mesh->indices.size(); face_index++) {
                shapes.push_back(Triangle{face_index, &meshes[tri_mesh_count]});
            }
            tri_mesh_count++;
        } else {
            assert(false);
        }
    }
    // Copy the materials
    for (const ParsedMaterial &parsed_mat : scene.materials) {
        if (auto *diffuse = std::get_if<ParsedDiffuse>(&parsed_mat)) {
            if (auto *tex = std::get_if<Vector3>(&diffuse->reflectance)){
                materials.push_back(Material{.material_type = material_e::DiffuseType, .reflectance = SolidTexture{.reflectance = *tex}});
            } else if (auto *tex = std::get_if<ParsedImageTexture>(&diffuse->reflectance)){
                materials.push_back(Material{.material_type = material_e::DiffuseType, .reflectance = ImgTexture(tex->filename, tex->uscale, tex->vscale, tex->uoffset, tex->voffset)});
            }
        } else if (auto *mirror = std::get_if<ParsedMirror>(&parsed_mat)) {
            if (auto *tex = std::get_if<Vector3>(&mirror->reflectance)){
                materials.push_back(Material{.material_type = material_e::MirrorType, .reflectance = SolidTexture{.reflectance = *tex}});
            } else if (auto *tex = std::get_if<ParsedImageTexture>(&mirror->reflectance)){
                materials.push_back(Material{.material_type = material_e::MirrorType, .reflectance = ImgTexture(tex->filename, tex->uscale, tex->vscale, tex->uoffset, tex->voffset)});
            }
        } else {
            assert(false);
        }
    }
    // Copy the lights
    for (const ParsedLight &parsed_light : scene.lights) {
        // We assume all lights are point lights for now.
        ParsedPointLight point_light = std::get<ParsedPointLight>(parsed_light);
        lights.push_back(Light{.position= point_light.position , .intensity= point_light.intensity});
    }
}

Vector3 get_diffuse_color(const Scene &scene, const Vector3 &pt, const Real eps, const Vector3 &kd, const Vector3 &n, const bool with_simd){
    Vector3 light_pos, light_ray, temp_b;
    Real t_shadow, hit_dist;
    Shape *temp_shape;
    Vector3 l, I;
    Real d_squared, nl;
    Vector3 color{0.0,0.0,0.0};
    Vector2 uv;
    for (int lit = 0; lit < scene.lights.size(); lit++){ // iterate all lights
        light_pos = scene.lights.at(lit).position;
        light_ray = normalize(pt-light_pos);
        t_shadow = -1.0;
        if (hit_bvh(scene.bvh, light_ray, light_pos, eps, eps, infinity<Real>(), &temp_shape, t_shadow, uv, with_simd)){
            if (t_shadow > 0.0){
                hit_dist = distance(light_pos, light_pos + light_ray * t_shadow);
                if (hit_dist > eps &&
                        hit_dist < (1-eps)*distance(light_pos, pt)){
                    continue;
                } else {
                    l = Real(-1.0) * light_ray;
                    I = scene.lights.at(lit).intensity;
                    d_squared = distance_squared(pt, light_pos);
                    nl = dot(n, l);
                    if (nl < 0.0){
                        nl = dot(Real(-1.0)*n,l);
                    }
                    color += ((kd * nl)/c_PI)*(I/d_squared);
                }
            }
        }
    }
    return color;
}

Vector3 get_mirror_color(const Scene &scene, const Vector3 &ray_in, const Vector3 &pt, const Real eps, const Vector3 &kd, const Vector3 &n, const bool with_simd){
    // get reflected ray intersecting object and then get color
    Real t_reflect = -1.0;
    Vector3 ray_reflect =  ray_in - 2.0*dot(ray_in,n)*n;
    Shape *temp_shape;
    Vector2 uv;
    if (hit_bvh(scene.bvh, ray_reflect, pt, eps, eps, infinity<Real>(), &temp_shape, t_reflect, uv, with_simd)){
        Vector3 mirror_pt = pt + t_reflect * ray_reflect;
        return kd*get_color(scene, ray_reflect, mirror_pt, eps, temp_shape, uv, with_simd);
    } 
    return kd*scene.background_color;
}

Vector3 get_color(const Scene &scene, const Vector3 &ray_in, const Vector3 &pt, const Real eps, Shape *shape, const Vector2 &uv, const bool with_simd){
    Vector3 kd, n;
    Vector2 uvt;
    if (auto *sph = std::get_if<Sphere>(shape)){ // get sphere color
        // calc uv
        uvt = sphere_uv(sph, pt);
        kd = get_texture_kd(scene.materials.at(sph->material_id).reflectance, uvt);
        n = normalize(pt - sph->center);
        if (scene.materials.at(sph->material_id).material_type == material_e::MirrorType) {
            return get_mirror_color(scene, ray_in, pt, eps, kd, n, with_simd);
        } else {
            return get_diffuse_color(scene, pt, eps, kd, n, with_simd);
        }
    } else if (auto *tri = std::get_if<Triangle>(shape)) { // triangle intersection test
        Vector3 p0 = tri->mesh->positions.at(tri->mesh->indices.at(tri->face_index).x);
        Vector3 p1 = tri->mesh->positions.at(tri->mesh->indices.at(tri->face_index).y);
        Vector3 p2 = tri->mesh->positions.at(tri->mesh->indices.at(tri->face_index).z);
        n = normalize(cross(p1 - p0, p2 - p1));
        // calc uv
        uvt = triangle_uv(tri, uv);
        kd = get_texture_kd(scene.materials.at(tri->mesh->material_id).reflectance, uvt);
        if (scene.materials.at(tri->mesh->material_id).material_type == material_e::MirrorType) {
            return get_mirror_color(scene, ray_in, pt, eps, kd, n, with_simd);
        } else {
            return get_diffuse_color(scene, pt, eps, kd, n, with_simd);
        }
    } else {
        assert(false);
    }
}
Vector3 get_color(const Scene &scene, const Vector3 &ray_in, const Vector3 &pt, const Real eps, Shape *shape, const Vector2 &uv){
    return get_color(scene, ray_in, pt, eps, shape, uv, true);
}

bool closest_hit(const std::vector<Shape> &shapes, const Vector3 &ray, const Vector3 &ray_origin, const Real eps, int &obj_id, Real &t_val){
    t_val = -1.0;
    obj_id = -1;
    int shape;
    Real t_temp;
    Vector3 intersection, p0, p1, p2;
    Vector2 uv;
    for (shape = 0; shape < shapes.size(); ++shape){
        if (auto *sph = std::get_if<Sphere>(&shapes.at(shape))){ // sphere intersection test
            t_temp = hit_sphere(*sph, ray, ray_origin);
            if (t_temp >= 0.0 && (t_val < 0 || t_val > t_temp) && distance(ray_origin, ray_origin + ray * t_temp)> eps){ // new closest obj
                t_val = t_temp;
                obj_id = shape;
            }
        } else if (auto *tri = std::get_if<Triangle>(&shapes.at(shape))) { // triangle intersection test
            p0 = tri->mesh->positions.at(tri->mesh->indices.at(tri->face_index).x);
            p1 = tri->mesh->positions.at(tri->mesh->indices.at(tri->face_index).y);
            p2 = tri->mesh->positions.at(tri->mesh->indices.at(tri->face_index).z);
            t_temp = get_tri_intersect(p0, p1, p2, ray, ray_origin, eps, uv);

            if (t_temp >= 0.0 && (t_val < 0 || t_val > t_temp) && distance(ray_origin, ray_origin + ray * t_temp)> eps){ // new closest obj
                t_val = t_temp;
                obj_id = shape;
            }
        }
    }
    if (t_val < 0.0){
        return false;
    } else {
        return true;
    }
}

Vector3 get_diffuse_color_nobvh(const Scene &scene, const Vector3 &pt, const Real eps, const Vector3 &kd, const Vector3 &n){
    Vector3 light_pos, light_ray, temp_b;
    Real t_shadow, hit_dist;
    int temp_obj;
    Vector3 l, I;
    Real d_squared, nl;
    Vector3 color{0.0,0.0,0.0};
    for (int lit = 0; lit < scene.lights.size(); lit++){ // iterate all lights
        light_pos = scene.lights.at(lit).position;
        light_ray = normalize(pt-light_pos);
        t_shadow = -1.0;
        if (closest_hit(scene.shapes, light_ray, light_pos, eps, temp_obj, t_shadow)){
            hit_dist = distance(light_pos, light_pos + light_ray * t_shadow);
            if (hit_dist > eps &&
                    hit_dist < (1-eps)*distance(light_pos, pt)){
                continue;
            } else {
                l = Real(-1.0) * light_ray;
                I = scene.lights.at(lit).intensity;
                d_squared = distance_squared(pt, light_pos);
                nl = dot(n, l);
                if (nl < 0.0){
                    nl = dot(Real(-1.0)*n,l);
                }
                color += ((kd * nl)/c_PI)*(I/d_squared);
            }
        } else {
            continue;
        }
    }
    return color;
}

Vector3 get_mirror_color_nobvh(const Scene &scene, const Vector3 &ray_in, const Vector3 &pt, const Real eps, const Vector3 &kd, const Vector3 &n){
    // get reflected ray intersecting object and then get color
    Real t_reflect = -1.0;
    Vector3 ray_reflect =  ray_in - 2.0*dot(ray_in,n)*n;
    int mirror_obj = -1;
    Vector3 b_coord;
        if (closest_hit(scene.shapes, ray_reflect, pt, eps, mirror_obj, t_reflect)){
            Vector3 mirror_pt = pt + t_reflect * ray_reflect;
            return kd*get_color_nobvh(scene, ray_reflect, mirror_pt, eps, mirror_obj);
        } else {
            return kd*scene.background_color;
        }
}

Vector3 get_color_nobvh(const Scene &scene, const Vector3 &ray_in, const Vector3 &pt, const Real eps, const int obj_id){
    Vector3 kd, n;
    if (auto *sph = std::get_if<Sphere>(&scene.shapes.at(obj_id))){ // get sphere color
        kd = std::get<SolidTexture>(scene.materials.at(sph->material_id).reflectance).reflectance;
        n = normalize(pt - sph->center);
        if (scene.materials.at(sph->material_id).material_type == material_e::MirrorType) {
            return get_mirror_color_nobvh(scene, ray_in, pt, eps, kd, n);

        } else {
            return get_diffuse_color_nobvh(scene, pt, eps, kd, n);
        }
    } else if (auto *tri = std::get_if<Triangle>(&scene.shapes.at(obj_id))) { // triangle intersection test
        Vector3 p0 = tri->mesh->positions.at(tri->mesh->indices.at(tri->face_index).x);
        Vector3 p1 = tri->mesh->positions.at(tri->mesh->indices.at(tri->face_index).y);
        Vector3 p2 = tri->mesh->positions.at(tri->mesh->indices.at(tri->face_index).z);
        n = normalize(cross(p1 - p0, p2 - p1));
        kd = std::get<SolidTexture>(scene.materials.at(tri->mesh->material_id).reflectance).reflectance;
        if (scene.materials.at(tri->mesh->material_id).material_type == material_e::MirrorType) {
            return get_mirror_color_nobvh(scene, ray_in, pt, eps, kd, n);
        } else {
            return get_diffuse_color_nobvh(scene, pt, eps, kd, n);
        }
    } else {
        assert(false);
    }
}