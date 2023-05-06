#include "hw2.h"
#include "parse_scene.h"
#include "scene.h"
#include "print_scene.h"
#include "timer.h"
#include "3rdparty/pcg.h"
#include "parallel.h"
#include "progressreporter.h"
#include "bvh.h"



// Real get_surface_area(const Shape* shape){
//     if (auto *sph = std::get_if<Sphere>(shape)){
//         return 4.0 * c_PI * sph->radius * sph->radius;
//     } else if (auto *tri = std::get_if<Triangle>(shape)) {
//         Vector3 p0 = tri->mesh->positions.at(tri->mesh->indices.at(tri->face_index).x);
//         Vector3 p1 = tri->mesh->positions.at(tri->mesh->indices.at(tri->face_index).y);
//         Vector3 p2 = tri->mesh->positions.at(tri->mesh->indices.at(tri->face_index).z);
//         return 0.5*length(cross(p0, p1) + cross(p1, p2) + cross(p2, p0));
//     } else {
//         assert(false);
//     }
// }

// double hit_sphere_old(const Sphere &sphere, const Vector3 &ray, const Vector3 &origin){
//     Real a = dot(ray, ray);
//     Real b = Real(2.0) * dot(origin-sphere.center, ray);
//     Real c = dot(origin-sphere.center, origin-sphere.center) - sphere.radius*sphere.radius;
//     Real disc = b*b - 4*a*c;
//     if (disc < 0){
//         return -1.0;
//     } else {
//         Real min_disc = (-b - sqrt(disc)) / (Real(2.0)*a);
//         if (min_disc < 0.0){
//             return (-b + sqrt(disc)) / (Real(2.0)*a);
//         } else {
//             return min_disc;
//         }
//     }
// }

// bool get_tri_intersect_old(const Vector3 &p0, const Vector3 &p1, const Vector3 &p2, const Vector3 &ray, const Vector3 &lookfrom, const Real eps, Vector3 &coordinate, Real &t_val) {
//         Vector3 vec1 = p1 - p0;
//         Vector3 vec2 = p2 - p0;
//         Vector3 pvec = cross(ray, vec2);
//         Real det = dot(vec1, pvec);

//         if (abs(det) < eps) {// ray is parallel
//             return false;
//         }
//         Vector3 tvec = lookfrom-p0;
//         Real m = dot(tvec, pvec) * (1.0/det);
//         if (m < 0.0 || m > 1.0){
//             return false;
//         }
//         Vector3 qvec = cross(tvec, vec1);
//         Real n = (1.0/det) * dot(ray, qvec);

//         if (n < 0.0 || m+n > 1.0){
//             return false;
//         }

//         Real t = (1.0/det) * dot(vec2, qvec);
//         if (t > eps){
//             coordinate.x = 1-m-n;
//             coordinate.y = m;
//             coordinate.z = n; 
//             t_val = t;
//             return true;
//         } else {
//             return false;
//         }
// }





// Vector3 get_diffuse_color(const Scene &scene, const Vector3 &pt, const Real eps, const Vector3 &kd, const Vector3 &n, const bool with_bvh, const bool with_simd){
//     Vector3 light_pos, light_ray, temp_b;
//     Real t_shadow, hit_dist;
//     int temp_obj;
//     Shape *temp_shape;
//     Vector3 l, I;
//     Real d_squared, nl;
//     Vector3 color{0.0,0.0,0.0};
//     for (int lit = 0; lit < scene.lights.size(); lit++){ // iterate all lights
//         light_pos = scene.lights.at(lit).position;
//         light_ray = normalize(pt-light_pos);
//         t_shadow = -1.0;
//         if (with_bvh){
//             if (hit_bvh(scene.bvh, light_ray, light_pos, eps,  &temp_shape, t_shadow, temp_b, with_simd)){
//                 hit_dist = distance(light_pos, light_pos + light_ray * t_shadow);
//                 if (hit_dist > eps &&
//                         hit_dist < (1-eps)*distance(light_pos, pt)){
//                     continue;
//                 } else {
//                     l = Real(-1.0) * light_ray;
//                     I = scene.lights.at(lit).intensity;
//                     d_squared = distance_squared(pt, light_pos);
//                     nl = dot(n, l);
//                     if (nl < 0.0){
//                         nl = dot(Real(-1.0)*n,l);
//                     }
//                     color += ((kd * nl)/c_PI)*(I/d_squared);
//                 }
//             } else {
//                 continue;
//             }
//         } else {
//             if (closest_hit(scene.shapes, light_ray, light_pos, eps, temp_obj, t_shadow, temp_b)){
//                 hit_dist = distance(light_pos, light_pos + light_ray * t_shadow);
//                 if (hit_dist > eps &&
//                         hit_dist < (1-eps)*distance(light_pos, pt)){
//                     continue;
//                 } else {
//                     l = Real(-1.0) * light_ray;
//                     I = scene.lights.at(lit).intensity;
//                     d_squared = distance_squared(pt, light_pos);
//                     nl = dot(n, l);
//                     if (nl < 0.0){
//                         nl = dot(Real(-1.0)*n,l);
//                     }
//                     color += ((kd * nl)/c_PI)*(I/d_squared);
//                 }
//             } else {
//                 continue;
//             }
//         }
//     }
//     return color;
// }

// Vector3 get_mirror_color(const Scene &scene, const Vector3 &ray_in, const Vector3 &pt, const Real eps, const Vector3 &kd, const Vector3 &n, const bool with_bvh, const bool with_simd){
//     // get reflected ray intersecting object and then get color
//     Real t_reflect = -1.0;
//     Vector3 ray_reflect =  ray_in - 2.0*dot(ray_in,n)*n;
//     int mirror_obj = -1;
//     Shape *temp_shape;
//     Vector3 b_coord;
//     if (with_bvh){
//         if (hit_bvh(scene.bvh, ray_reflect, pt, eps,  &temp_shape, t_reflect, b_coord, with_simd)){
//             Vector3 mirror_pt = pt + t_reflect * ray_reflect;
//             return kd*get_color(scene, ray_reflect, mirror_pt, eps, temp_shape, with_bvh, with_simd);
//         } else {
//             return kd*scene.background_color;
//         }
//     } else {
//         if (closest_hit(scene.shapes, ray_reflect, pt, eps, mirror_obj, t_reflect, b_coord)){
//             Vector3 mirror_pt = pt + t_reflect * ray_reflect;
//             return kd*get_color(scene, ray_reflect, mirror_pt, eps, mirror_obj, with_bvh, with_simd);
//         } else {
//             return kd*scene.background_color;
//         }
//     }

// }

// Vector3 get_color(const Scene &scene, const Vector3 &ray_in, const Vector3 &pt, const Real eps, const int obj_id, const bool with_bvh, const bool with_simd){
//     Vector3 kd, n;
//     if (auto *sph = std::get_if<Sphere>(&scene.shapes.at(obj_id))){ // get sphere color
//         kd = scene.materials.at(sph->material_id).reflectance;
//         n = normalize(pt - sph->center);
//         if (scene.materials.at(sph->material_id).material_type == material_e::MirrorType) {
//             return get_mirror_color(scene, ray_in, pt, eps, kd, n, with_bvh, with_simd);

//         } else {
//             return get_diffuse_color(scene, pt, eps, kd, n, with_bvh, with_simd);
//         }
//     } else if (auto *tri = std::get_if<Triangle>(&scene.shapes.at(obj_id))) { // triangle intersection test
//         Vector3 p0 = tri->mesh->positions.at(tri->mesh->indices.at(tri->face_index).x);
//         Vector3 p1 = tri->mesh->positions.at(tri->mesh->indices.at(tri->face_index).y);
//         Vector3 p2 = tri->mesh->positions.at(tri->mesh->indices.at(tri->face_index).z);
//         n = normalize(cross(p1 - p0, p2 - p1));
//         kd = scene.materials.at(tri->mesh->material_id).reflectance;
//         if (scene.materials.at(tri->mesh->material_id).material_type == material_e::MirrorType) {
//             return get_mirror_color(scene, ray_in, pt, eps, kd, n, with_bvh, with_simd);
//         } else {
//             return get_diffuse_color(scene, pt, eps, kd, n, with_bvh, with_simd);
//         }
//     } else {
//         assert(false);
//     }
// }

// Vector3 get_color(const Scene &scene, const Vector3 &ray_in, const Vector3 &pt, const Real eps, Shape *shape, const bool with_bvh, const bool with_simd){
//     Vector3 kd, n;
//     if (auto *sph = std::get_if<Sphere>(shape)){ // get sphere color
//         kd = scene.materials.at(sph->material_id).reflectance;
//         n = normalize(pt - sph->center);
//         if (scene.materials.at(sph->material_id).material_type == material_e::MirrorType) {
//             return get_mirror_color(scene, ray_in, pt, eps, kd, n, with_bvh, with_simd);

//         } else {
//             return get_diffuse_color(scene, pt, eps, kd, n, with_bvh, with_simd);
//         }
//     } else if (auto *tri = std::get_if<Triangle>(shape)) { // triangle intersection test
//         Vector3 p0 = tri->mesh->positions.at(tri->mesh->indices.at(tri->face_index).x);
//         Vector3 p1 = tri->mesh->positions.at(tri->mesh->indices.at(tri->face_index).y);
//         Vector3 p2 = tri->mesh->positions.at(tri->mesh->indices.at(tri->face_index).z);
//         n = normalize(cross(p1 - p0, p2 - p1));
//         kd = scene.materials.at(tri->mesh->material_id).reflectance;
//         if (scene.materials.at(tri->mesh->material_id).material_type == material_e::MirrorType) {
//             return get_mirror_color(scene, ray_in, pt, eps, kd, n, with_bvh, with_simd);
//         } else {
//             return get_diffuse_color(scene, pt, eps, kd, n, with_bvh, with_simd);
//         }
//     } else {
//         assert(false);
//     }
// }

void print_bvh(AABB &bvh){

    std::cout << "parent aabb -- size: " << bvh.shapes.size() << " a: " << bvh.a << " b: " << bvh.b << std::endl;
    for (auto shape: bvh.shapes){
        if (auto *sph = std::get_if<Sphere>(shape)){ // get sphere color
            std::cout << "Sphere-- center: " << sph->center << " rad: " << sph->radius << std::endl;
        } else if (auto *tri = std::get_if<Triangle>(shape)) { // triangle intersection test
            std::cout << "Triangle -- positions: " << tri->mesh->positions.at(tri->mesh->indices.at(tri->face_index).x)
             << ","<<tri->mesh->positions.at(tri->mesh->indices.at(tri->face_index).y) <<","<< tri->mesh->positions.at(tri->mesh->indices.at(tri->face_index).x) << std::endl;
        }
    }
    std::cout << "left child size: " << bvh.left->shapes.size() << " a: " << bvh.left->a << " b: " << bvh.left->b << std::endl;
        for (auto shape: bvh.left->shapes){
        if (auto *sph = std::get_if<Sphere>(shape)){ // get sphere color
            std::cout << "Sphere-- center: " << sph->center << " rad: " << sph->radius << std::endl;
        } else if (auto *tri = std::get_if<Triangle>(shape)) { // triangle intersection test
            std::cout << "Triangle -- positions: " << tri->mesh->positions.at(tri->mesh->indices.at(tri->face_index).x)
             << ","<<tri->mesh->positions.at(tri->mesh->indices.at(tri->face_index).y) <<","<< tri->mesh->positions.at(tri->mesh->indices.at(tri->face_index).x) << std::endl;
        }
        }
    
    std::cout << "right child size: " << bvh.right->shapes.size()<< " a: " << bvh.right->a << " b: " << bvh.right->b << std::endl;
            for (auto shape: bvh.right->shapes){
        if (auto *sph = std::get_if<Sphere>(shape)){ // get sphere color
            std::cout << "Sphere-- center: " << sph->center << " rad: " << sph->radius << std::endl;
        } else if (auto *tri = std::get_if<Triangle>(shape)) { // triangle intersection test
            std::cout << "Triangle -- positions: " << tri->mesh->positions.at(tri->mesh->indices.at(tri->face_index).x)
             << ","<<tri->mesh->positions.at(tri->mesh->indices.at(tri->face_index).y) <<","<< tri->mesh->positions.at(tri->mesh->indices.at(tri->face_index).x) << std::endl;
        }
        }
}

Image3 hw_2_1(const std::vector<std::string> &params) {
    // Homework 2.1: render a single triangle and outputs
    // its barycentric coordinates.
    // We will use the following camera parameter
    // lookfrom = (0, 0,  0)
    // lookat   = (0, 0, -1)
    // up       = (0, 1,  0)
    // vfov     = 45
    // and we will parse the triangle vertices from params
    // The three vertices are stored in v0, v1, and v2 below.

    std::vector<float> tri_params;
    int spp = 16;
    for (int i = 0; i < (int)params.size(); i++) {
        if (params[i] == "-spp") {
            spp = std::stoi(params[++i]);
        } else {
            tri_params.push_back(std::stof(params[i]));
        }
    }

    if (tri_params.size() < 9) {
        // Not enough parameters to parse the triangle vertices.
        return Image3(0, 0);
    }

    Vector3 p0{tri_params[0], tri_params[1], tri_params[2]};
    Vector3 p1{tri_params[3], tri_params[4], tri_params[5]};
    Vector3 p2{tri_params[6], tri_params[7], tri_params[8]};

    Image3 img(640 /* width */, 480 /* height */);
    Camera camera{
        .lookfrom= Vector3{0, 0, 0},
        .lookat=  Vector3{0, 0, -1},
        .up= Vector3{0, 1, 0},
        .vfov = 45,
        .width = img.width,
        .height = img.height
    };
    Real eps = 0.00001;
    pcg32_state pcg_state = init_pcg32();


    for (int y = 0; y < img.height; y++) {
        for (int x = 0; x < img.width; x++) {
            Vector3 ray = get_ray(camera, x, y, pcg_state);
            Vector2 uv;
            Real t_val;
            t_val = get_tri_intersect(p0, p1, p2, ray, camera.lookfrom, eps, uv);
            if (t_val > 0.0){
                img(x,y) = Vector3{1-uv.x-uv.y, uv.x, uv.y};
            } else {
                img(x,y) = Vector3{0.5,0.5,0.5};
            }
        }
    }
    return img;
}

Image3 hw_2_2(const std::vector<std::string> &params) {
    // Homework 2.2: render a triangle mesh.
    // We will use the same camera parameter:
    // lookfrom = (0, 0,  0)
    // lookat   = (0, 0, -1)
    // up       = (0, 1,  0)
    // vfov     = 45
    // and we will use a fixed triangle mesh: a tetrahedron!
    int spp = 16;
    for (int i = 0; i < (int)params.size(); i++) {
        if (params[i] == "-spp") {
            spp = std::stoi(params[++i]);
        }
    }

    std::vector<Vector3> positions = {
        Vector3{ 0.0,  0.5, -2.0},
        Vector3{ 0.0, -0.3, -1.0},
        Vector3{ 1.0, -0.5, -3.0},
        Vector3{-1.0, -0.5, -3.0}
    };
    std::vector<Vector3i> indices = {
        Vector3i{0, 1, 2},
        Vector3i{0, 3, 1},
        Vector3i{0, 2, 3},
        Vector3i{1, 2, 3}
    };

    Image3 img(640 /* width */, 480 /* height */);
    Camera camera{
        .lookfrom= Vector3{0, 0, 0},
        .lookat=  Vector3{0, 0, -1},
        .up= Vector3{0, 1, 0},
        .vfov = 45,
        .width = img.width,
        .height = img.height
    };
    Real eps = 0.00001;
    pcg32_state pcg_state = init_pcg32();


    for (int y = 0; y < img.height; y++) {
        for (int x = 0; x < img.width; x++) {
            Vector3 ray = get_ray(camera, x, y, pcg_state);
            Vector2 close_uv;
            Real dist = infinity<Real>();
            bool hit = false;
            Real t_val;
            for (int triangle = 0; triangle < indices.size(); triangle++){
                Vector2 uv;
                t_val = get_tri_intersect(positions.at(indices.at(triangle).x),positions.at(indices.at(triangle).y), positions.at(indices.at(triangle).z), ray, camera.lookfrom, eps, uv);
                if (t_val > 0.0){
                    hit = true;
                    if (distance(camera.lookfrom + t_val * ray, camera.lookfrom) < dist){
                        close_uv = uv;
                        dist = distance(camera.lookfrom + t_val * ray, camera.lookfrom);
                    }
                }
            }
            if (hit){
                img(x,y) = Vector3{1-close_uv.x-close_uv.y, close_uv.x, close_uv.y};
            } else {
                img(x,y) = Vector3{0.5,0.5,0.5};
            }
        }
    }
    return img;
}

Image3 hw_2_3(const std::vector<std::string> &params) {
    // Homework 2.3: render a scene file provided by our parser.
    if (params.size() < 1) {
        return Image3(0, 0);
    }

    Timer timer;
    tick(timer);
    ParsedScene pscene = parse_scene(params[0]);
    std::cout << "Scene parsing done. Took " << tick(timer) << " seconds." << std::endl;
    //std::cout << scene << std::endl;
    tick(timer);
    Scene scene = Scene(pscene);
    std::cout << "Scene data struct construction done. Took " << tick(timer) << " seconds." << std::endl;
    tick(timer);
    Real eps = 0.0001;
    const int spp = scene.samples_per_pixel;
    const Real real_spp = Real(spp);
    Image3 img(scene.width, scene.height);
    const int w = img.width;
    const int h = img.height;
    constexpr int tile_size = 16;
    const int num_tiles_x = (w + tile_size - 1) / tile_size;
    const int num_tiles_y = (h + tile_size - 1) / tile_size;
    parallel_for([&](const Vector2i &tile) {
        pcg32_state pcg_state = init_pcg32(tile[1] * num_tiles_x + tile[0]);
        const int x0 = tile[0] * tile_size;
        const int x1 = min(x0 + tile_size, w);
        const int y0 = tile[1] * tile_size;
        const int y1 = min(y0 + tile_size, h);
        int x, y, sample, obj_id;
        Real t;
        Vector3 color_sum, ray, b_coord, color, pt;
        for (y = y0; y < y1; ++y){
            for (x = x0; x < x1; ++x){
                color_sum = Vector3{0.0,0.0,0.0};
                for (sample = 0; sample < spp; ++sample){
                    ray = get_ray(scene.camera, x, y, pcg_state);
                    obj_id = -1;
                    t = -1.0;
                    if (closest_hit(scene.shapes, ray, scene.camera.lookfrom, eps, obj_id, t)){ // was a hit
                        pt = scene.camera.lookfrom + t * ray;
                        color = get_color_nobvh(scene, ray, pt, eps, obj_id);
                    } else {
                        color = scene.background_color;
                    }
                    color_sum += color;
                }
                img(x,y) = color_sum/real_spp;
            }
        }
    }, Vector2i(num_tiles_x, num_tiles_y));
    std::cout << "Scene image render done. Took " << tick(timer) << " seconds." << std::endl;

    return img;
}

Image3 hw_2_4(const std::vector<std::string> &params) {
    // Homework 2.4: render the AABBs of the scene.
    if (params.size() < 1) {
        return Image3(0, 0);
    }

    Timer timer;
    tick(timer);
    ParsedScene pscene = parse_scene(params[0]);
    std::cout << "Scene parsing done. Took " << tick(timer) << " seconds." << std::endl;
    tick(timer);
    Scene scene = Scene(pscene);
    std::cout << "Scene data struct construction done. Took " << tick(timer) << " seconds." << std::endl;
    tick(timer);

    std::vector<AABB> boxes;
    Vector3 p0, p1, p2;

    for (int shape = 0; shape < scene.shapes.size(); ++shape){
        if (auto *sph = std::get_if<Sphere>(&scene.shapes.at(shape))){
            boxes.push_back(
                AABB{
                    .a = sph->center - sph->radius,
                    .b = sph->center + sph->radius,
                });
        } else if (auto *tri = std::get_if<Triangle>(&scene.shapes.at(shape))) {
            p0 = tri->mesh->positions.at(tri->mesh->indices.at(tri->face_index).x);
            p1 = tri->mesh->positions.at(tri->mesh->indices.at(tri->face_index).y);
            p2 = tri->mesh->positions.at(tri->mesh->indices.at(tri->face_index).z);
            boxes.push_back(
                AABB{
                    .a = Vector3{min(p2.x, min(p0.x, p1.x)), min(p2.y, min(p0.y, p1.y)), min(p2.z, min(p0.z, p1.z))},
                    .b = Vector3{max(p2.x, max(p0.x, p1.x)), max(p2.y, max(p0.y, p1.y)), max(p2.z, max(p0.z, p1.z))}
                });
        } else {
            assert(false);
        }
    }
    std::cout << "Bounded " << scene.shapes.size() << " objects. Took " << tick(timer) << " seconds." << std::endl;

    tick(timer);
    const int spp = scene.samples_per_pixel;
    const Real real_spp = Real(spp);
    Image3 img(scene.width, scene.height);
    const int w = img.width;
    const int h = img.height;
    constexpr int tile_size = 16;
    const int num_tiles_x = (w + tile_size - 1) / tile_size;
    const int num_tiles_y = (h + tile_size - 1) / tile_size;
    parallel_for([&](const Vector2i &tile) {
        pcg32_state pcg_state = init_pcg32(tile[1] * num_tiles_x + tile[0]);
        const int x0 = tile[0] * tile_size;
        const int x1 = min(x0 + tile_size, w);
        const int y0 = tile[1] * tile_size;
        const int y1 = min(y0 + tile_size, h);
        int x, y, sample;
        Vector3 color_sum, ray, color;
        for (y = y0; y < y1; ++y){
            for (x = x0; x < x1; ++x){
                color_sum = Vector3{0.0,0.0,0.0};
                for (sample = 0; sample < spp; ++sample){
                    ray = get_ray(scene.camera, x, y, pcg_state);
                    color = Vector3{0.5,0.5,0.5};
                    for (const AABB &box: boxes){
                        if (hit_box(box, ray, scene.camera.lookfrom, 0.0, infinity<Real>())){
                            color = Vector3{1.0,1.0,1.0};
                            break;
                        }
                    }
                    color_sum += color;
                }
                img(x,y) = color_sum/real_spp;
            }
        }
    }, Vector2i(num_tiles_x, num_tiles_y));
    std::cout << "Scene image render done. Took " << tick(timer) << " seconds." << std::endl;

    return img;
}

Image3 hw_2_5(const std::vector<std::string> &params) {
    // Homework 2.5: rendering with BVHs
    if (params.size() < 1) {
        return Image3(0, 0);
    }
    int split_type = 0;
    if (params.size() > 1){
        split_type = std::stoi(params[1]);
    }

    Timer timer;
    ParsedScene pscene = parse_scene(params[0]);
    std::cout << "Scene parsing done. Took " << tick(timer) << " seconds." << std::endl;
    tick(timer);
    Scene scene = Scene(pscene);
    std::cout << "Scene data construction done. Took " << tick(timer) << " seconds." << std::endl;
    tick(timer);
    scene.bvh = build_bvh(scene.shapes, split_type);
    std::cout << "BVH construction done. Took " << tick(timer) << " seconds." << std::endl;
    tick(timer);
    const int spp = scene.samples_per_pixel;
    const Real eps = 0.000001;
    const Real real_spp = Real(spp);
    Image3 img(scene.width, scene.height);
    const int w = img.width;
    const int h = img.height;
    constexpr int tile_size = 16;
    const int num_tiles_x = (w + tile_size - 1) / tile_size;
    const int num_tiles_y = (h + tile_size - 1) / tile_size;
    ProgressReporter reporter(num_tiles_x * num_tiles_y);
    parallel_for([&](const Vector2i &tile) {
        pcg32_state pcg_state = init_pcg32(tile[1] * num_tiles_x + tile[0]);
        const int x0 = tile[0] * tile_size;
        const int x1 = min(x0 + tile_size, w);
        const int y0 = tile[1] * tile_size;
        const int y1 = min(y0 + tile_size, h);
        int x, y, sample;
        Real t;
        Vector3 color_sum, ray, color, pt;
        Vector2 uv;
        Shape *hs;
        for (y = y0; y < y1; ++y){
            for (x = x0; x < x1; ++x){
                color_sum = Vector3{0.0,0.0,0.0};
                for (sample = 0; sample < spp; ++sample){
                    ray = get_ray(scene.camera, x, y, pcg_state);
                    t = -1.0;
                    if (hit_bvh(scene.bvh, ray, scene.camera.lookfrom,eps, eps, infinity<Real>(), &hs, t, uv, false)){ // was a hit
                            pt = scene.camera.lookfrom + t * ray;
                            color = get_color(scene, ray, pt, eps, hs, uv, false);
                    } else {
                        color = scene.background_color;
                    }
                    color_sum += color;
                }
                img(x,y) = color_sum/real_spp;
            }
        }
        reporter.update(1);
    }, Vector2i(num_tiles_x, num_tiles_y));
    reporter.done();
    std::cout << "Scene image render done. Took " << tick(timer) << " seconds." << std::endl;

    return img;
}

Image3 hw_2_7(const std::vector<std::string> &params) {
    // Homework 2.7: optimized rendering
    if (params.size() < 1) {
        return Image3(0, 0);
    }
    int split_type = 1;
    if (params.size() > 1){
        split_type = std::stoi(params[1]);
    }

    Timer timer;
    ParsedScene pscene = parse_scene(params[0]);
    std::cout << "Scene parsing done. Took " << tick(timer) << " seconds." << std::endl;
    tick(timer);
    Scene scene = Scene(pscene);
    std::cout << "Scene data construction done. Took " << tick(timer) << " seconds." << std::endl;
    tick(timer);
    scene.bvh = build_bvh(scene.shapes, split_type);
    std::cout << "BVH construction done. Took " << tick(timer) << " seconds." << std::endl;
    tick(timer);
    const int spp = scene.samples_per_pixel;
    const Real eps = 0.000001;
    const Real real_spp = Real(spp);
    Image3 img(scene.width, scene.height);
    const int w = img.width;
    const int h = img.height;
    constexpr int tile_size = 16;
    const int num_tiles_x = (w + tile_size - 1) / tile_size;
    const int num_tiles_y = (h + tile_size - 1) / tile_size;
    ProgressReporter reporter(num_tiles_x * num_tiles_y);
    parallel_for([&](const Vector2i &tile) {
        pcg32_state pcg_state = init_pcg32(tile[1] * num_tiles_x + tile[0]);
        const int x0 = tile[0] * tile_size;
        const int x1 = min(x0 + tile_size, w);
        const int y0 = tile[1] * tile_size;
        const int y1 = min(y0 + tile_size, h);
        int x, y, sample;
        Real t;
        Vector3 color_sum, ray, color, pt;
        Vector2 uv;
        Shape *hs;
        for (y = y0; y < y1; ++y){
            for (x = x0; x < x1; ++x){
                color_sum = Vector3{0.0,0.0,0.0};
                for (sample = 0; sample < spp; ++sample){
                    ray = get_ray(scene.camera, x, y, pcg_state);
                    t = -1.0;
                    if (hit_bvh(scene.bvh, ray, scene.camera.lookfrom, eps, eps, infinity<Real>(), &hs, t, uv, true)){ // was a hit
                        pt = scene.camera.lookfrom + t * ray;
                            color = get_color(scene, ray, pt, eps, hs, uv, true);
                    } else {
                        color = scene.background_color;
                    }
                    color_sum += color;
                }
                img(x,y) = color_sum/real_spp;
            }
        }
        reporter.update(1);
    }, Vector2i(num_tiles_x, num_tiles_y));
    reporter.done();
    std::cout << "Scene image render done. Took " << tick(timer) << " seconds." << std::endl;

    return img;
}
