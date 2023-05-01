#include "hw1.h"
#include "hw1_scenes.h"
#include "3rdparty/pcg.h"
#include "parallel.h"

using namespace hw1;

double hit_sphere(const Sphere &sphere, const Vector3 &ray, const Vector3 &origin){
    Real a = dot(ray, ray);
    Real b = Real(2.0) * dot(origin-sphere.center, ray);
    Real c = dot(origin-sphere.center, origin-sphere.center) - sphere.radius*sphere.radius;
    Real disc = b*b - 4*a*c;
    if (disc < 0){
        return -1.0;
    } else {
        Real min_disc = (-b - sqrt(disc)) / (Real(2.0)*a);
        if (min_disc < 0.0){
            return (-b + sqrt(disc)) / (Real(2.0)*a);
        } else {
            return min_disc;
        }
    }
}

Vector3 get_ray(Scene &scene, int x, int y, int width, int height, bool aa, pcg32_state &pcg_state){
    Vector3 lookfrom = scene.camera.lookfrom;
    Vector3 lookat = scene.camera.lookat;
    Vector3 up = scene.camera.up;
    Real vfov = scene.camera.vfov;

    Real theta = radians(vfov);
    Real h = tan(theta/2);
    Real ratio = Real(width)/height;

    Vector3 w = normalize(lookfrom - lookat);
    Vector3 u = normalize(cross(up, w));
    Vector3 v = cross(w, u);

    Vector3 vertical = 2.0 * h * v;
    Vector3 horizontal = ratio * 2.0 * h * u;
    Vector3 llc = lookfrom - horizontal/Real(2.0) + vertical/Real(2.0) - w;

    Real ux;
    Real vy;
    if (aa){
        ux = ((x + next_pcg32_real<double>(pcg_state)) / width);
        vy = ((y + next_pcg32_real<double>(pcg_state)) / height);
    } else {
        ux = ((x + Real(0.5)) / width);
        vy = ((y + Real(0.5)) / height);
    }
    return normalize(llc + ux*horizontal - vy*vertical - lookfrom);
}

Vector3 get_color(Scene &scene, Vector3 &ray_in, Vector3 &pt, Real eps, Vector3 &kd, Vector3 &obj_center, MaterialType material){
    Vector3 color = Vector3{0.0,0.0,0.0};
    std::vector<PointLight> &lights = scene.lights;
    std::vector<Sphere> &shapes = scene.shapes;
    Vector3 n = normalize(pt - obj_center);
    if (material==MaterialType::Mirror){
        // get reflected ray intersecting object and then get color
        Real t_reflect = -1.0;
        Vector3 ray_reflect =  ray_in - 2.0*dot(ray_in,n)*n;
        int mirror_obj = -1;
        for (int obj = 0; obj < shapes.size(); obj++){
            Real t_temp = hit_sphere(shapes.at(obj), ray_reflect, pt);
            if (t_temp >= 0.0 && (t_reflect < 0 || t_reflect > t_temp) && (distance(pt, pt + ray_reflect * t_temp) > eps)){
                t_reflect = t_temp;
                mirror_obj = obj;
            }
        }
        if (t_reflect < 0.0){
            color = kd*Vector3{0.5,0.5,0.5};
        } else {
            Vector3 mirror_pt = pt + ray_reflect * t_reflect;
            Vector3 mirror_kd = scene.materials.at(scene.shapes.at(mirror_obj).material_id).color;
            Vector3 mirror_center = scene.shapes.at(mirror_obj).center;
            color = kd*get_color(scene, ray_reflect, mirror_pt, eps, mirror_kd, mirror_center,  scene.materials.at(scene.shapes.at(mirror_obj).material_id).type);
        }
    } else if (material==MaterialType::Dielectric){
        Real ir = 1.5;
        Real refraction_ratio;
        if (dot(ray_in, n) < 0.0){ // ray points into sphere
            refraction_ratio = Real(1.0)/ir;
        } else {
            refraction_ratio = ir;
        }

        Real cos_theta = fmin(dot(-ray_in, n), 1.0);
        Real sin_theta = sqrt(Real(1.0) - cos_theta*cos_theta);

        Vector3 direction;

        if (refraction_ratio * sin_theta > 1.0){ // reflect
            direction = ray_in - 2.0*dot(ray_in,n)*n;
        } else { // refract
            Vector3 r_out_perp =  refraction_ratio * (ray_in + cos_theta*n);
            Vector3 r_out_parallel = -sqrt(fabs(1.0 - length_squared(r_out_perp))) * n;
            direction = r_out_perp + r_out_parallel;
        }

        Real t_hit = -1.0;
        int hit_obj = -1;
        for (int obj = 0; obj < shapes.size(); obj++){
            Real t_temp = hit_sphere(shapes.at(obj), direction, pt);
            if (t_temp >= 0.0 && (t_hit < 0.0 || t_hit > t_temp) && (distance(pt, pt + direction * t_temp) > eps)){
                t_hit = t_temp;
                hit_obj = obj;
            }
        }
        if (t_hit < 0.0){
            color = Vector3{0.5,0.5,0.5};
        } else {
            Vector3 hit_pt = pt + direction * t_hit;
            Vector3 hit_kd = scene.materials.at(scene.shapes.at(hit_obj).material_id).color;
            Vector3 hit_center = scene.shapes.at(hit_obj).center;
            color = get_color(scene, direction, hit_pt, eps, hit_kd, hit_center,  scene.materials.at(scene.shapes.at(hit_obj).material_id).type);
        }
    } else {
        for (int lit = 0; lit < lights.size(); lit++){ // iterate all lights
            Vector3 light_pos = lights.at(lit).position;
            Vector3 light_ray = normalize(pt-light_pos);
            Real t_shadow = -1.0;
            for (int obj = 0; obj < shapes.size(); obj++){
                Real t_temp = hit_sphere(shapes.at(obj), light_ray, light_pos);
                if (t_temp >= 0.0 && (t_shadow < 0 || t_shadow > t_temp)){ // new closest obj to light
                    t_shadow = t_temp;
                }
            }
            // check for occlusion
            Real hit_dist = distance(light_pos, light_pos + light_ray * t_shadow);
            if (hit_dist > eps &&
                    hit_dist < (1-eps)*distance(light_pos, pt)){
                continue;
            } else {
                Vector3 l = Real(-1.0) * light_ray;
                Vector3 I = lights.at(lit).intensity;
                Real d_squared = distance_squared(pt, light_pos);
                Real nl = dot(n, l);
                if (nl < 0.0){
                    nl = dot(Real(-1.0)*n,l);
                }
                color += ((kd * nl)/c_PI)*(I/d_squared);
            }
        }
    }
    return color;
}

Image3 hw_1_1(const std::vector<std::string> &/*params*/) {
    // Homework 1.1: generate camera rays and output the ray directions
    // The camera is positioned at (0, 0, 0), facing towards (0, 0, -1),
    // with an up vector (0, 1, 0) and a vertical field of view of 90 degree.

    Image3 img(640 /* width */, 480 /* height */);
    Real ratio = Real(img.width)/img.height;
    for (int y = 0; y < img.height; y++) {
        for (int x = 0; x < img.width; x++) {
            img(x, y) = 
                normalize(Vector3{ ratio * (2 * ((x + Real(0.5)) / img.width) - 1),
                        1 - 2 * ((y + Real(0.5)) / img.height),
                        Real(-1)});
        }
    }
    return img;
}

Image3 hw_1_2(const std::vector<std::string> &/*params*/) {
    // Homework 1.2: intersect the rays generated from hw_1_1
    // with a unit sphere located at (0, 0, -2)

    Image3 img(640 /* width */, 480 /* height */);
    Sphere sphere = Sphere{Vector3{0,0,-2}, Real(1.0), 0};
    Real ratio = Real(img.width)/img.height;

    for (int y = 0; y < img.height; y++) {
        for (int x = 0; x < img.width; x++) {
            Vector3 ray = normalize(Vector3{ ratio * (2 * ((x + Real(0.5)) / img.width) - 1),
                             1 - 2 * ((y + Real(0.5)) / img.height),
                             Real(-1)});
            Real t = hit_sphere(sphere, ray, Vector3{0.0,0.0,0.0});
            if (t < 0.0){
                img(x, y) = Vector3{0.5,0.5,0.5};
            } else {
                img(x ,y) = 
                    (((ray * t) - sphere.center) + Vector3{1.0,1.0,1.0})/Real(2.0);
            }
        }
    }
    return img;
}

Image3 hw_1_3(const std::vector<std::string> &params) {
    // Homework 1.3: add camera control to hw_1_2. 
    // We will use a look at transform:
    // The inputs are "lookfrom" (camera position),
    //                "lookat" (target),
    //                and the up vector
    // and the vertical field of view (in degrees).
    // If the user did not specify, fall back to the default
    // values below.
    // If you use the default values, it should render
    // the same image as hw_1_2.

    Vector3 lookfrom = Vector3{0, 0,  0};
    Vector3 lookat   = Vector3{0, 0, -2};
    Vector3 up       = Vector3{0, 1,  0};
    Real    vfov     = 90;
    for (int i = 0; i < (int)params.size(); i++) {
        if (params[i] == "-lookfrom") {
            Real x = std::stof(params[++i]);
            Real y = std::stof(params[++i]);
            Real z = std::stof(params[++i]);
            lookfrom = Vector3{x, y, z};
        } else if (params[i] == "-lookat") {
            Real x = std::stof(params[++i]);
            Real y = std::stof(params[++i]);
            Real z = std::stof(params[++i]);
            lookat = Vector3{x, y, z};
        } else if (params[i] == "-up") {
            Real x = std::stof(params[++i]);
            Real y = std::stof(params[++i]);
            Real z = std::stof(params[++i]);
            up = Vector3{x, y, z};
        } else if (params[i] == "-vfov") {
            vfov = std::stof(params[++i]);
        }
    }

    Image3 img(640 /* width */, 480 /* height */);
    Real theta = radians(vfov);
    Real h = tan(theta/2);
    Sphere sphere = Sphere{Vector3{0,0,-2}, Real(1.0), 0};
    Real ratio = Real(img.width)/img.height;

    Vector3 w = normalize(lookfrom - lookat);
    Vector3 u = normalize(cross(up, w));
    Vector3 v = cross(w, u);

    Vector3 vertical = 2.0 * h * v;
    Vector3 horizontal = ratio * 2.0 * h * u;
    Vector3 llc = lookfrom - horizontal/Real(2.0) + vertical/Real(2.0) - w;

    for (int y = 0; y < img.height; y++) {
        for (int x = 0; x < img.width; x++) {
            Vector3 ray = normalize(llc + ((x + Real(0.5)) / img.width)*horizontal - ((y + Real(0.5)) / img.height)*vertical - lookfrom);
            Real t = hit_sphere(sphere, ray, lookfrom);
            if (t < 0.0){
                img(x, y) = Vector3{0.5,0.5,0.5};
            } else {
                img(x ,y) = 
                     (((lookfrom + ray * t) - sphere.center) + Vector3{1.0,1.0,1.0}) / Real(2.0);
            }
        }
    }
    return img;
}

Image3 hw_1_4(const std::vector<std::string> &params) {
    // Homework 1.4: render the scenes defined in hw1_scenes.h
    // output their diffuse color directly.
    if (params.size() == 0) {
        return Image3(0, 0);
    }

    int scene_id = std::stoi(params[0]);
    // Your scene is hw1_scenes[scene_id]
    Scene scene = hw1_scenes[scene_id];
    Vector3 lookfrom = scene.camera.lookfrom;

    Image3 img(640 /* width */, 480 /* height */);
    pcg32_state pcg_state = init_pcg32();

    for (int y = 0; y < img.height; y++) {
        for (int x = 0; x < img.width; x++) {
            Vector3 ray = get_ray(scene, x, y, img.width, img.height, false, pcg_state);
            Real t = -1.0;
            img(x, y) = Vector3{0.5,0.5,0.5};
            for (int obj = 0; obj < scene.shapes.size(); obj++){
                Real t_temp = hit_sphere(scene.shapes.at(obj), ray, lookfrom);
                if (t_temp >= 0.0 && (t < 0 || t > t_temp)){ // new closest obj
                    t = t_temp;
                    img(x,y) = scene.materials.at(scene.shapes.at(obj).material_id).color;
                }
            }
        }
    }
    return img;
}

Image3 hw_1_5(const std::vector<std::string> &params) {
    // Homework 1.5: render the scenes defined in hw1_scenes.h,
    // light them using the point lights in the scene.
    if (params.size() == 0) {
        return Image3(0, 0);
    }

    int scene_id = std::stoi(params[0]);
    Scene scene = hw1_scenes[scene_id];
    Vector3 lookfrom = scene.camera.lookfrom;
    // Your scene is hw1_scenes[scene_id]

    Image3 img(640 /* width */, 480 /* height */);

    Real eps = 0.0001;
    pcg32_state pcg_state = init_pcg32();

    for (int y = 0; y < img.height; y++) {
        for (int x = 0; x < img.width; x++) {
            Vector3 ray = get_ray(scene, x, y, img.width, img.height, false, pcg_state);
            Real t = -1.0;
            img(x, y) = Vector3{0.5,0.5,0.5};
            int object_id = -1;
            for (int obj = 0; obj < scene.shapes.size(); obj++){
                Real t_temp = hit_sphere(scene.shapes.at(obj), ray, lookfrom);
                if (t_temp >= 0.0 && (t < 0 || t > t_temp)){ // new closest obj
                    t = t_temp;
                    object_id = obj;
                }
            }
            if (t >= 0.0){ // lighting
                Vector3 pt = lookfrom + ray * t;
                Vector3 color = Vector3{0.0,0.0,0.0};
                Vector3 kd = scene.materials.at(scene.shapes.at(object_id).material_id).color;
                Vector3 obj_center = scene.shapes.at(object_id).center;
                img(x,y) = get_color(scene, ray, pt, eps, kd, obj_center, MaterialType::Diffuse);
            }
        }
    }
    return img;
}

Image3 hw_1_6(const std::vector<std::string> &params) {
    // Homework 1.6: add antialiasing to homework 1.5
    if (params.size() == 0) {
        return Image3(0, 0);
    }

    int scene_id = 0;
    int spp = 64;
    for (int i = 0; i < (int)params.size(); i++) {
        if (params[i] == "-spp") {
            spp = std::stoi(params[++i]);
        } else {
            scene_id = std::stoi(params[i]);
        }
    }
    Scene scene = hw1_scenes[scene_id];
    Vector3 lookfrom = scene.camera.lookfrom;

    // Your scene is hw1_scenes[scene_id]

    Image3 img(160 /* width */, 120 /* height */);

    Real eps = 0.0001;
    pcg32_state pcg_state = init_pcg32();

    for (int y = 0; y < img.height; y++) {
        for (int x = 0; x < img.width; x++) {
            Vector3 color_sum = Vector3{0.0,0.0,0.0};
            for (int sample = 0; sample < spp; sample++){
                Vector3 ray = get_ray(scene, x, y, img.width, img.height, true, pcg_state);
                Real t = -1.0;
                Vector3 color = Vector3{0.5,0.5,0.5};
                int object_id = -1;
                for (int obj = 0; obj < scene.shapes.size(); obj++){
                    Real t_temp = hit_sphere(scene.shapes.at(obj), ray, lookfrom);
                    if (t_temp >= 0.0 && (t < 0 || t > t_temp)){ // new closest obj
                        t = t_temp;
                        object_id = obj;
                    }
                }
                if (t >= 0.0){ // lighting
                    Vector3 pt = lookfrom + ray * t;
                    Vector3 kd = scene.materials.at(scene.shapes.at(object_id).material_id).color;
                    Vector3 obj_center = scene.shapes.at(object_id).center;
                    color = get_color(scene, ray, pt, eps, kd, obj_center, MaterialType::Diffuse);
                }
                color_sum += color;
            }
            img(x,y) = color_sum/Real(spp);
        }
    }
    return img;
}

Image3 hw_1_7(const std::vector<std::string> &params) {
    // Homework 1.7: add mirror materials to homework 1.6
    if (params.size() == 0) {
        return Image3(0, 0);
    }

    int scene_id = 0;
    int spp = 64;
    for (int i = 0; i < (int)params.size(); i++) {
        if (params[i] == "-spp") {
            spp = std::stoi(params[++i]);
        } else {
            scene_id = std::stoi(params[i]);
        }
    }

    Scene scene = hw1_scenes[scene_id];
    Vector3 lookfrom = scene.camera.lookfrom;
    // Your scene is hw1_scenes[scene_id]

    Image3 img(640 /* width */, 480 /* height */);
    Real eps = 0.0001;
    pcg32_state pcg_state = init_pcg32();

    for (int y = 0; y < img.height; y++) {
        for (int x = 0; x < img.width; x++) {
            Vector3 color_sum = Vector3{0.0,0.0,0.0};
            for (int sample = 0; sample < spp; sample++){
                Vector3 ray = get_ray(scene, x, y, img.width, img.height, true, pcg_state);
                Real t = -1.0;
                Vector3 color = Vector3{0.5,0.5,0.5};
                int object_id = -1;
                for (int obj = 0; obj < scene.shapes.size(); obj++){
                    Real t_temp = hit_sphere(scene.shapes.at(obj), ray, lookfrom);
                    if (t_temp >= 0.0 && (t < 0 || t > t_temp)){ // new closest obj
                        t = t_temp;
                        object_id = obj;
                    }
                }
                if (t >= 0.0){ // lighting
                    Vector3 pt = lookfrom + ray * t;
                    Vector3 kd = scene.materials.at(scene.shapes.at(object_id).material_id).color;
                    Vector3 obj_center = scene.shapes.at(object_id).center;
                    color = get_color(scene, ray, pt, eps, kd, obj_center, scene.materials.at(scene.shapes.at(object_id).material_id).type);
                }
                color_sum += color;
            }
            img(x,y) = color_sum/Real(spp);
        }
    }
    return img;
}

Image3 hw_1_8(const std::vector<std::string> &params) {
    // Homework 1.8: parallelize HW 1.7
    if (params.size() == 0) {
        return Image3(0, 0);
    }

    int scene_id = 0;
    int spp = 64;
    for (int i = 0; i < (int)params.size(); i++) {
        if (params[i] == "-spp") {
            spp = std::stoi(params[++i]);
        } else {
            scene_id = std::stoi(params[i]);
        }
    }

    Scene scene = hw1_scenes[scene_id];
    Vector3 lookfrom = scene.camera.lookfrom;
    const Real eps = 0.0001;
    // Your scene is hw1_scenes[scene_id]

    Image3 img(1280 /* width */, 960 /* height */);
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
        int x, y, sample, obj, object_id;
        Vector3 color_sum, ray, color, pt, kd, obj_center;
        Real t, t_temp;
        for (y = y0; y < y1; y++) {
            for (x = x0; x < x1; x++) {
                color_sum = Vector3{0.0,0.0,0.0};
                for (sample = 0; sample < spp; sample++){
                    ray = get_ray(scene, x, y, w, h, true, pcg_state);
                    t = -1.0;
                    color = Vector3{0.5,0.5,0.5};
                    object_id = -1;
                    for (obj = 0; obj < scene.shapes.size(); obj++){
                        t_temp = hit_sphere(scene.shapes.at(obj), ray, lookfrom);
                        if (t_temp >= 0.0 && (t < 0 || t > t_temp)){ // new closest obj
                            t = t_temp;
                            object_id = obj;
                        }
                    }
                    if (t >= 0.0){ // lighting
                        pt = lookfrom + ray * t;
                        kd = scene.materials.at(scene.shapes.at(object_id).material_id).color;
                        obj_center = scene.shapes.at(object_id).center;
                        color = get_color(scene, ray, pt, eps, kd, obj_center, scene.materials.at(scene.shapes.at(object_id).material_id).type);
                    }
                    color_sum += color;
                }
            img(x,y) = color_sum/Real(spp);
            }
        }
    }, Vector2i(num_tiles_x, num_tiles_y));
    return img;
}



Image3 hw_1_9(const std::vector<std::string> &params) {
    // Homework 1_9, my own scene
    int spp = 64;

    // generate with balls.py
    Scene scene{
        Camera{
            Vector3{0, 1,  0}, // lookfrom
            Vector3{0, 0, -1}, // lookat
            Vector3{0, 1,  0}, // up
            60                 // vfov
        },
        std::vector<Sphere>{
            {Vector3{ 0.0, -100.5, -3.0}, 100.0, 0},
            {Vector3{ 1.0, 0.0, -2.0}, 0.12053668025532305, 2},
            {Vector3{ 0.970941817426052, 0.0, -1.7606843357124422}, 0.12053668025532305, 3},
            {Vector3{ 0.8854560256532099, 0.0, -1.5352768279562314}, 0.12053668025532305, 2},
            {Vector3{ 0.7485107481711011, 0.0, -1.3368773417592048}, 0.12053668025532305, 3},
            {Vector3{ 0.5680647467311559, 0.0, -1.1770161341063436}, 0.12053668025532305, 2},
            {Vector3{ 0.35460488704253557, 0.0, -1.064983757314585}, 0.12053668025532305, 3},
            {Vector3{ 0.120536680255323, 0.0, -1.0072911259019461}, 0.12053668025532305, 2},
            {Vector3{ -0.1205366802553229, 0.0, -1.0072911259019461}, 0.12053668025532305, 3},
            {Vector3{ -0.35460488704253545, 0.0, -1.064983757314585}, 0.12053668025532305, 2},
            {Vector3{ -0.5680647467311556, 0.0, -1.1770161341063434}, 0.12053668025532305, 3},
            {Vector3{ -0.7485107481711012, 0.0, -1.3368773417592048}, 0.12053668025532305, 2},
            {Vector3{ -0.8854560256532096, 0.0, -1.535276827956231}, 0.12053668025532305, 3},
            {Vector3{ -0.970941817426052, 0.0, -1.7606843357124424}, 0.12053668025532305, 2},
            {Vector3{ -1.0, 0.0, -2.0000000000000004}, 0.12053668025532305, 3},
            {Vector3{ -0.9709418174260521, 0.0, -2.2393156642875574}, 0.12053668025532305, 2},
            {Vector3{ -0.8854560256532101, 0.0, -2.464723172043768}, 0.12053668025532305, 3},
            {Vector3{ -0.7485107481711013, 0.0, -2.6631226582407947}, 0.12053668025532305, 2},
            {Vector3{ -0.5680647467311559, 0.0, -2.822983865893656}, 0.12053668025532305, 3},
            {Vector3{ -0.3546048870425359, 0.0, -2.935016242685415}, 0.12053668025532305, 2},
            {Vector3{ -0.12053668025532356, 0.0, -2.992708874098054}, 0.12053668025532305, 3},
            {Vector3{ 0.12053668025532321, 0.0, -2.992708874098054}, 0.12053668025532305, 2},
            {Vector3{ 0.35460488704253557, 0.0, -2.935016242685415}, 0.12053668025532305, 3},
            {Vector3{ 0.5680647467311548, 0.0, -2.822983865893657}, 0.12053668025532305, 2},
            {Vector3{ 0.7485107481711009, 0.0, -2.6631226582407956}, 0.12053668025532305, 3},
            {Vector3{ 0.88545602565321, 0.0, -2.4647231720437683}, 0.12053668025532305, 2},
            {Vector3{ 0.970941817426052, 0.0, -2.239315664287558}, 0.12053668025532305, 3}

        },
        std::vector<Material>{
            {MaterialType::Diffuse, Vector3{0.80, 0.80, 0.20}},
            {MaterialType::Diffuse, Vector3{0.75, 0.25, 0.25}},
            {MaterialType::Mirror , Vector3{0.75, 0.25, 0.75}},
            {MaterialType::Mirror, Vector3{0.25, 0.75, 0.75}}
        },
        std::vector<PointLight>{
            {Vector3{100, 100, 100}, Vector3{ 5, 5,  2}},
            {Vector3{ 10,  10,  10}, Vector3{-5, 5,  1}},
            {Vector3{  2,   2,   2}, Vector3{ 0, 5, -5}}
        }
    };
    
    Vector3 lookfrom = scene.camera.lookfrom;
    const Real eps = 0.0001;

    Image3 img(1280 /* width */, 960 /* height */);
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
        int x, y, sample, obj, object_id;
        Vector3 color_sum, ray, color, pt, kd, obj_center;
        Real t, t_temp;
        for (y = y0; y < y1; y++) {
            for (x = x0; x < x1; x++) {
                color_sum = Vector3{0.0,0.0,0.0};
                for (sample = 0; sample < spp; sample++){
                    ray = get_ray(scene, x, y, w, h, true, pcg_state);
                    t = -1.0;
                    color = Vector3{0.5,0.5,0.5};
                    object_id = -1;
                    for (obj = 0; obj < scene.shapes.size(); obj++){
                        t_temp = hit_sphere(scene.shapes.at(obj), ray, lookfrom);
                        if (t_temp >= 0.0 && (t < 0 || t > t_temp)){ // new closest obj
                            t = t_temp;
                            object_id = obj;
                        }
                    }
                    if (t >= 0.0){ // lighting
                        pt = lookfrom + ray * t;
                        kd = scene.materials.at(scene.shapes.at(object_id).material_id).color;
                        obj_center = scene.shapes.at(object_id).center;
                        color = get_color(scene, ray, pt, eps, kd, obj_center, scene.materials.at(scene.shapes.at(object_id).material_id).type);
                    }
                    color_sum += color;
                }
            img(x,y) = color_sum/Real(spp);
            }
        }
    }, Vector2i(num_tiles_x, num_tiles_y));
    return img;
}

Image3 hw_1_10(const std::vector<std::string> &params) {

    int spp = 64;
    for (int i = 0; i < (int)params.size(); i++) {
        if (params[i] == "-spp") {
            spp = std::stoi(params[++i]);
        }
    }

    Scene scene = hw1_scenes[1];
    scene.materials.at(3).type = MaterialType::Dielectric;
    scene.camera.lookfrom = Vector3{0,2,2};
    Vector3 lookfrom = scene.camera.lookfrom;
    const Real eps = 0.0001;

    Image3 img(1280 /* width */, 960 /* height */);
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
        int x, y, sample, obj, object_id;
        Vector3 color_sum, ray, color, pt, kd, obj_center;
        Real t, t_temp;
        for (y = y0; y < y1; y++) {
            for (x = x0; x < x1; x++) {
                color_sum = Vector3{0.0,0.0,0.0};
                for (sample = 0; sample < spp; sample++){
                    ray = get_ray(scene, x, y, w, h, true, pcg_state);
                    t = -1.0;
                    color = Vector3{0.5,0.5,0.5};
                    object_id = -1;
                    for (obj = 0; obj < scene.shapes.size(); obj++){
                        t_temp = hit_sphere(scene.shapes.at(obj), ray, lookfrom);
                        if (t_temp >= 0.0 && (t < 0 || t > t_temp)){ // new closest obj
                            t = t_temp;
                            object_id = obj;
                        }
                    }
                    if (t >= 0.0){ // lighting
                        pt = lookfrom + ray * t;
                        kd = scene.materials.at(scene.shapes.at(object_id).material_id).color;
                        obj_center = scene.shapes.at(object_id).center;
                        color = get_color(scene, ray, pt, eps, kd, obj_center, scene.materials.at(scene.shapes.at(object_id).material_id).type);
                    }
                    color_sum += color;
                }
            img(x,y) = color_sum/Real(spp);
            }
        }
    }, Vector2i(num_tiles_x, num_tiles_y));
    return img;
}