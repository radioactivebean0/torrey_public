#include "bvh.h"
#include "3rdparty/pcg.h"
#include "hw2.h"
#include <immintrin.h>
#include <emmintrin.h>

#define NUM_BINS 4

pcg32_state bvh_state = init_pcg32();

void set_surrounding_aabb(std::shared_ptr<AABB> &parent, const std::shared_ptr<AABB> &a, const std::shared_ptr<AABB> &b){
    parent->a = Vector3{
        min(a->a.x, b->a.x),
        min(a->a.y, b->a.y),
        min(a->a.z, b->a.z)
    };
    parent->b = Vector3{
        max(a->b.x, b->b.x),
        max(a->b.y, b->b.y),
        max(a->b.z, b->b.z)
    };
    parent->shapes = std::vector<Shape*>();
    parent->shapes.insert(parent->shapes.end(), a->shapes.begin(), a->shapes.end());
    parent->shapes.insert(parent->shapes.end(), b->shapes.begin(), b->shapes.end());
    return;
}

Real get_union_aabb_surface(std::vector<std::shared_ptr<AABB>> &boxes, size_t start, size_t end){
    Vector3 min_vec = boxes.at(start)->a;
    Vector3 max_vec = boxes.at(start)->b;
    for (int i = start+1; i < end; i++){
        min_vec = min(min_vec, boxes[i]->a);
        max_vec = max(max_vec, boxes[i]->b);
    }
    Vector3 diag = max_vec - min_vec;
    return 2 * (diag.x * diag.y + diag.x * diag.z + diag.y * diag.z);
}

void get_bounds_aabb_list(std::vector<std::shared_ptr<AABB>> &boxes, size_t start, size_t end, Vector3 &a, Vector3 &b){
    Vector3 min_vec = boxes.at(start)->a;
    Vector3 max_vec = boxes.at(start)->b;
    for (int i = start+1; i < end; i++){
        min_vec = min(min_vec, boxes[i]->a);
        max_vec = max(max_vec, boxes[i]->b);
    }
    a = min_vec;
    b = max_vec;
}

Real surface_area_one(Vector3 a, Vector3 b){
    Vector3 diag = b - a;
    return 2 * (diag.x * diag.y + diag.x * diag.z + diag.y * diag.z);
}

Real surface_area_two(Vector3 a1, Vector3 b1, Vector3 a2, Vector3 b2){
    Vector3 diag = max(b2, b1) - min(a1,a2);
    return 2 * (diag.x * diag.y + diag.x * diag.z + diag.y * diag.z);
}

Real surface_area_three(Vector3 a1, Vector3 b1, Vector3 a2, Vector3 b2, Vector3 a3, Vector3 b3){
    Vector3 diag = max(max(b2, b1),b3) - min(min(a1,a2),a3);
    return 2 * (diag.x * diag.y + diag.x * diag.z + diag.y * diag.z);
}

Real get_best_four_split(Vector3 a[], Vector3 b[]){
    Real best_sah;
    Real temp_sah;
    best_sah = surface_area_one(a[0],b[0])+ surface_area_three(a[1],b[1],a[2],b[2],a[3],b[3]);
    temp_sah = surface_area_two(a[0],b[0],a[1],b[1])+surface_area_two(a[2],b[2],a[3],b[3]);
    
}

std::shared_ptr<AABB> build_sah_bvh_helper(std::vector<std::shared_ptr<AABB>> &boxes, size_t start, size_t end, int split_method){
    // base cases
    if (end-start == 1) {
        return boxes.at(start);
    } 
    std::shared_ptr<AABB> node = std::make_shared<AABB>();
    uint32_t axis = 0;
    uint32_t split_ind = -1;
    Real best_sah;
    Real temp_sah;
    if (end - start ==2){
        node->left = boxes.at(start);
        node->right = boxes.at(start+1);
    } else if (end-start == 3){
        // sort by x
        std::sort(boxes.begin()+start, boxes.begin()+end, aabb_x_compare);
        best_sah = get_union_aabb_surface(boxes, start, start+1)+get_union_aabb_surface(boxes, start+1, start+3);
        split_ind = 1;
        temp_sah = get_union_aabb_surface(boxes, start, start+2)+get_union_aabb_surface(boxes, start+2, start+3);
        if (temp_sah < best_sah){
            best_sah = temp_sah;
            split_ind = 2;
        }
        // sort by y
        std::sort(boxes.begin()+start, boxes.begin()+end, aabb_y_compare);
        temp_sah = get_union_aabb_surface(boxes, start, start+1)+get_union_aabb_surface(boxes, start+1, start+3);
        if (temp_sah < best_sah){
            split_ind = 1;
            axis = 1;
            best_sah = temp_sah;
        }
        temp_sah = get_union_aabb_surface(boxes, start, start+2)+get_union_aabb_surface(boxes, start+2, start+3);
        if (temp_sah < best_sah){
            split_ind = 2;
            axis = 1;
            best_sah = temp_sah;
        }
        // sort by z
        std::sort(boxes.begin()+start, boxes.begin()+end, aabb_z_compare);
        temp_sah = get_union_aabb_surface(boxes, start, start+1)+get_union_aabb_surface(boxes, start+1, start+3);
        if (temp_sah < best_sah){
            split_ind = 1;
            axis = 2;
            best_sah = temp_sah;
        }
        temp_sah = get_union_aabb_surface(boxes, start, start+2)+get_union_aabb_surface(boxes, start+2, start+3);
        if (temp_sah < best_sah){
            split_ind = 2;
            axis = 2;
            best_sah = temp_sah;
        }
    } else { // split into even 4 on each axis and try
        int chunksize = (end - start)/NUM_BINS;
        int remainderchunks = (end - start)%NUM_BINS;
        Vector3 chunks_a[4];
        Vector3 chunks_b[4];
        int i;

        // sort by x
        std::sort(boxes.begin()+start, boxes.begin()+end, aabb_x_compare);
        i = 0;
        while (i < remainderchunks){
            get_bounds_aabb_list(boxes, start+((chunksize+1)*i), start+((chunksize+1)*(i+1)), chunks_a[i], chunks_b[i]);
            i++;
        }
        while (i < NUM_BINS){
            get_bounds_aabb_list(boxes, start+((chunksize+1)*remainderchunks)+((i-remainderchunks)*chunksize), start+((chunksize+1)*remainderchunks)+((i+1-remainderchunks)*chunksize), chunks_a[i], chunks_b[i]);
            i++;
        }
        best_sah = get_best_four_split(chunks_a, chunks_b);
    }
}

std::shared_ptr<AABB> build_bvh_helper(std::vector<std::shared_ptr<AABB>> &boxes, size_t start, size_t end, int split_method){
    // base cases
    if (end-start == 1) {
        return boxes.at(start);
    } 
    std::shared_ptr<AABB> node = std::make_shared<AABB>();
    uint32_t axis = 0;
    if (split_method == 0) { // random split
        axis = nextUInt(bvh_state, 3);
    } else if (split_method == 1){ // split axis with largest extent
        Real min_x = (*std::min_element(boxes.begin()+start, boxes.begin()+end, aabb_x_compare))->a.x;
        Real min_y = (*std::min_element(boxes.begin()+start, boxes.begin()+end, aabb_y_compare))->a.y;
        Real min_z = (*std::min_element(boxes.begin()+start, boxes.begin()+end, aabb_z_compare))->a.z;
        Real max_x = (*std::max_element(boxes.begin()+start, boxes.begin()+end, aabb_x_compare_max))->b.x;
        Real max_y = (*std::max_element(boxes.begin()+start, boxes.begin()+end, aabb_y_compare_max))->b.y;
        Real max_z = (*std::max_element(boxes.begin()+start, boxes.begin()+end, aabb_z_compare_max))->b.z;
        Real z_diff = max_z - min_z;
        Real y_diff = max_y - min_y;
        Real x_diff = max_x - min_x;
        if (z_diff > y_diff && z_diff > x_diff){
            axis = 2;
        } else if (y_diff > z_diff && y_diff > x_diff){
            axis = 1;
        } else {
            axis = 0;
        }
    } else {
        assert(false);
    }
    // select random axis
    auto comparator = (axis == 0) ? aabb_x_compare
                    : (axis == 1) ? aabb_y_compare
                                  : aabb_z_compare;
    node->axis = axis;
    if (end - start == 2){
        if (comparator(boxes.at(start), boxes.at(start+1))) {
            node->left = boxes.at(start);
            node->right = boxes.at(start+1);
        } else {
            node->left = boxes.at(start+1);
            node->right = boxes.at(start);
        }
        set_surrounding_aabb(node, node->left, node->right);
        return node;
    } else {
        // sort subvector on chosen axis
        std::sort(boxes.begin()+start, boxes.begin()+end, comparator);
        // split apart and recurse
        size_t middle = start + (end-start)/2;
        node->left = build_bvh_helper(boxes, start, middle, split_method);
        node->right = build_bvh_helper(boxes, middle, end, split_method);
        set_surrounding_aabb(node, node->left, node->right);
        return node;
    }
}

AABB build_bvh(std::vector<Shape> &shapes, int split_method = 0){
    std::vector<std::shared_ptr<AABB>> boxes;
    Vector3 p0, p1, p2;

    for (int shape = 0; shape < shapes.size(); ++shape){
        std::shared_ptr<AABB> box = std::make_shared<AABB>();
        if (auto *sph = std::get_if<Sphere>(&shapes.at(shape))){
            box->a = sph->center - sph->radius;
            box->b = sph->center + sph->radius;
        } else if (auto *tri = std::get_if<Triangle>(&shapes.at(shape))) {
            p0 = tri->mesh->positions.at(tri->mesh->indices.at(tri->face_index).x);
            p1 = tri->mesh->positions.at(tri->mesh->indices.at(tri->face_index).y);
            p2 = tri->mesh->positions.at(tri->mesh->indices.at(tri->face_index).z);
            box->a = Vector3{min(p2.x, min(p0.x, p1.x)), min(p2.y, min(p0.y, p1.y)), min(p2.z, min(p0.z, p1.z))};
            box->b = Vector3{max(p2.x, max(p0.x, p1.x)), max(p2.y, max(p0.y, p1.y)), max(p2.z, max(p0.z, p1.z))};

        } else {
            assert(false);
        }
        box->shapes = std::vector<Shape*>{&shapes.at(shape)};
        boxes.push_back(box);
        // if (split_method == 2){
        //     box->surface_area = get_surface_area(&shapes.at(shape));
        // }
    }
    if (split_method == 2){
        return *build_sah_bvh_helper(boxes, 0, boxes.size());
    }
    return *build_bvh_helper(boxes, 0, boxes.size(), split_method);
}
// }
// bool hit_box_simd(const AABB &box, const Vector3 &ray, const Vector3 &lookfrom, Real t_min, Real t_max){
//     Real invD_realx = 1.0/ray[0];
//     __m128d invDx = _mm_set_pd1(invD_realx);
//     __m128d packed_lfx = _mm_set_pd1(lookfrom[0]);
//     __m128d tx = _mm_set_pd(box.a[0], box.b[0]);
//     Real invD_realy = 1.0/ray[1];
//     __m128d invDy = _mm_set_pd1(invD_realy);
//     __m128d packed_lfy = _mm_set_pd1(lookfrom[1]);
//     __m128d ty = _mm_set_pd(box.a[1], box.b[1]);
//     Real invD_realz = 1.0/ray[2];
//     __m128d invDz = _mm_set_pd1(invD_realz);
//     __m128d packed_lfz = _mm_set_pd1(lookfrom[2]);
//     __m128d tz = _mm_set_pd(box.a[2], box.b[2]);
//     tx = _mm_sub_pd(tx, packed_lfx);
//     tx = _mm_mul_pd(tx, invDx);
//     ty = _mm_sub_pd(ty, packed_lfy);
//     ty = _mm_mul_pd(ty, invDy);
//     tz = _mm_sub_pd(tz, packed_lfz);
//     tz = _mm_mul_pd(tz, invDz);
//     if (invD_realx < 0.0){
//         tx = _mm_permute_pd(tx, 0x1); // simd swap
//     }
//     if (invD_realy < 0.0){
//         ty = _mm_permute_pd(ty, 0x1); // simd swap
//     }
//     if (invD_realz < 0.0){
//         tz = _mm_permute_pd(tz, 0x1); // simd swap
//     }
//     double t0, t1;
//     __m128d t = _mm_set_pd(t_min, t_max);
//     __m128d temp = _mm_min_sd(tx, t);
//     _mm_storel_pd(&t1, temp);
//     temp = _mm_max_pd(tx, t);
//     _mm_storeh_pd(&t0, temp);
//     if(t1 < t0){
//         return false;
//     }
//     temp = _mm_min_sd(ty, t);
//     _mm_storel_pd(&t1, temp);
//     temp = _mm_max_pd(ty, t);
//     _mm_storeh_pd(&t0, temp);
//     if(t1 < t0){
//         return false;
//     }
//     temp = _mm_min_sd(tz, t);
//     _mm_storel_pd(&t1, temp);
//     temp = _mm_max_pd(tz, t);
//     _mm_storeh_pd(&t0, temp);
//     if(t1 < t0){
//         return false;
//     }
//     return true;
//     // __m128d t = _mm_set_pd(t_min, t_max);
//     // __m128d temp = _mm_max_sd(tx, ty);
//     // temp = _mm_max_sd(tz, temp);
//     // t = _mm_min_sd(t, temp); // do the floor function
//     // _mm_storel_pd(&t1, t);
//     // //_mm_storeh_pd(&t0, t);
//     // temp = _mm_min_pd(tx, ty);
//     // temp = _mm_min_pd(temp, tz);
//     // t = _mm_max_pd(t, temp);
//     // _mm_storeh_pd(&t0, t);
//     //return t1 >= t0;
// }
// int hit_boxes_simd(const AABB &box, const Vector3 &ray, const Vector3 &lookfrom, Real t_min, Real t_max){
//     const __m128d lookfromx = _mm_set_pd1(lookfrom[0]);
//     const __m128d lookfromy = _mm_set_pd1(lookfrom[1]);
//     const __m128d lookfromz = _mm_set_pd1(lookfrom[2]);
//     const Real real_invDx = 1.0 / ray[0];
//     const Real real_invDy = 1.0 / ray[1];
//     const Real real_invDz = 1.0 / ray[2];
//     const __m128d invDx = _mm_set_pd1(real_invDx);
//     const __m128d invDy = _mm_set_pd1(real_invDy);
//     const __m128d invDz = _mm_set_pd1(real_invDz);
//     __m128d hitacc = _mm_setzero_pd();
//     __m128d vect_min, vect_max;
//     //__m128d t0x, t0y, t0z, t1x, t1y, t1z;
//     __m128d t0, t1;
//     vect_min = _mm_set_pd1(t_min);
//     vect_max = _mm_set_pd1(t_max);
//     t0 = _mm_set_pd(box.left->a[0], box.right->a[0]);
//     t1 = _mm_set_pd(box.left->b[0], box.right->b[0]);
    
//     t0 = _mm_sub_pd(t0, lookfromx);
//     t1 = _mm_sub_pd(t1, lookfromx);
//     t0 = _mm_mul_pd(t0, invDx);
//     t1 = _mm_mul_pd(t1, invDx);
//     if (real_invDx < 0.0){
//         std::swap(t0, t1);
//     }
//     vect_min = _mm_max_pd(t0, vect_min);
//     vect_max = _mm_min_pd(t1, vect_max);
//     hitacc = _mm_or_pd(_mm_cmplt_pd(vect_max, vect_min), hitacc);
//     t0 = _mm_set_pd(box.left->a[1], box.right->a[1]);
//     t1 = _mm_set_pd(box.left->b[1], box.right->b[1]);

//     t0 = _mm_sub_pd(t0, lookfromy);
//     t1 = _mm_sub_pd(t1, lookfromy);
//     t0 = _mm_mul_pd(t0, invDy);
//     t1 = _mm_mul_pd(t1, invDy);
//     if (real_invDy < 0.0){
//         std::swap(t0, t1);
//     }
//     vect_min = _mm_max_pd(t0, vect_min);
//     vect_max = _mm_min_pd(t1, vect_max);
//     hitacc = _mm_or_pd(_mm_cmplt_pd(vect_max, vect_min), hitacc);
//     t0 = _mm_set_pd(box.left->a[2], box.right->a[2]);
//     t1 = _mm_set_pd(box.left->b[2], box.right->b[2]);
//     t0 = _mm_sub_pd(t0, lookfromz);
//     t1 = _mm_sub_pd(t1, lookfromz);
//     t0 = _mm_mul_pd(t0, invDz);
//     t1 = _mm_mul_pd(t1, invDz);
//     if (real_invDz < 0.0){
//         std::swap(t0, t1);
//     }

//     vect_min = _mm_max_pd(t0, vect_min);
//     vect_max = _mm_min_pd(t1, vect_max);
//     hitacc = _mm_or_pd(_mm_cmplt_pd(vect_max, vect_min), hitacc);
//     return _mm_movemask_pd(hitacc);
// }
int hit_boxes_simd(const AABB &box, const Vector3 &ray, const Vector3 &lookfrom, Real t_min, Real t_max){
    __m128d t0, t1, m128_lookfrom, invD, vect_min, vect_max, hitacc, temp;
    vect_min = _mm_set_pd1(t_min);
    vect_max = _mm_set_pd1(t_max);
    hitacc = _mm_setzero_pd();
    // Real t_minasdf = t_min;
    // Real t_maxasdf = t_max;
    // Real t_minasdfa = t_min;
    // Real t_maxasdfa = t_max;
    // bool hit1 = true;
    // bool hit2 = true;
    for (int d = 0; d < 3; d++) {
        Real real_invD = 1.0 / ray[d];
        m128_lookfrom = _mm_set_pd1(lookfrom[d]);
        invD = _mm_set_pd1(real_invD);
        t0 = _mm_set_pd(box.left->a[d], box.right->a[d]);
        t1 = _mm_set_pd(box.left->b[d], box.right->b[d]);
        t0 = _mm_sub_pd(t0, m128_lookfrom);
        t1 = _mm_sub_pd(t1, m128_lookfrom);
        t0 = _mm_mul_pd(t0, invD);
        t1 = _mm_mul_pd(t1, invD);
        if (real_invD < 0.0){
            std::swap(t0, t1);
        }
        vect_min = _mm_max_pd(t0, vect_min);
        vect_max = _mm_min_pd(t1, vect_max);
        // double vmina, vminb;
        // _mm_storeh_pd(&vmina, vect_min);
        // _mm_storel_pd(&vminb, vect_min);
        // double vmaxa, vmaxb;
        // _mm_storeh_pd(&vmaxa, vect_max);
        // _mm_storel_pd(&vmaxb, vect_max);
        // Real invDasdf = 1.0 / ray[d];
        // auto t0asdf = (box.left->a[d] - lookfrom[d]) * invDasdf;
        // auto t1asdf = (box.left->b[d] - lookfrom[d]) * invDasdf;
        // if (invDasdf < 0.0)
        //     std::swap(t0asdf, t1asdf);
        // t_minasdf = t0asdf > t_minasdf ? t0asdf : t_minasdf;
        // t_maxasdf = t1asdf < t_maxasdf ? t1asdf : t_maxasdf;
        //         if (t_maxasdf < t_minasdf){
        //     hit1 = false;
        // }
        // auto t0asdfa = (box.right->a[d] - lookfrom[d]) * invDasdf;
        // auto t1asdfa = (box.right->b[d] - lookfrom[d]) * invDasdf;
        // if (invDasdf < 0.0)
        //     std::swap(t0asdfa, t1asdfa);
        // t_minasdfa = t0asdfa > t_minasdfa ? t0asdfa : t_minasdfa;
        // t_maxasdfa = t1asdfa < t_maxasdfa ? t1asdfa : t_maxasdfa;
        // if (t_maxasdfa < t_minasdfa){
        //     hit2 = false;
        // }
        // if (vmaxa != t_maxasdf){
        //     std::cout << "SIMD amax , expected: " << vmaxa << "," << t_maxasdf << std::endl;
        // }
        // if (vmaxb != t_maxasdfa){
        //     std::cout << "SIMD bmax , expected: " << vmaxb << "," << t_maxasdfa << std::endl;
        // }
        // if (vmina != t_minasdf){
        //     std::cout << "SIMD amin , expected: " << vmina << "," << t_minasdf << std::endl;
        // }
        // if (vminb != t_minasdfa){
        //     std::cout << "SIMD bmin , expected: " << vminb << "," << t_minasdfa << std::endl;
        // }
        // std::cout <<"SIMD: " <<  vmaxa << "," << vmaxb << "   " << vmina << "," << vminb << std::endl;
        // std::cout << "expected: " << t_maxasdf << "," << t_maxasdfa << "   " << t_minasdf << "," << t_minasdfa << std::endl;
        temp = _mm_cmplt_pd(vect_max, vect_min);
        hitacc = _mm_or_pd(temp, hitacc);
    }
    int res = _mm_movemask_pd(hitacc);
    // std::cout << "SIMD result: " << res << std::endl;
    // std::cout << "expected result: " << hit1 << "," << hit2 << std::endl;
    return res;

    //return true;
}
bool hit_box(const AABB &box, const Vector3 &ray, const Vector3 &lookfrom, Real t_min, Real t_max){
    //return hit_box_simd(box, ray, lookfrom, t_min, t_max);
    for (int d = 0; d < 3; d++) {
        Real invD = 1.0 / ray[d];
        auto t0 = (box.a[d] - lookfrom[d]) * invD;
        auto t1 = (box.b[d] - lookfrom[d]) * invD;
        if (invD < 0.0)
            std::swap(t0, t1);
        t_min = t0 > t_min ? t0 : t_min;
        t_max = t1 < t_max ? t1 : t_max;
        if (t_max < t_min){
            return false;
        }
    }
    return true;
}

bool hit_bvh(const AABB &bvh, const Vector3 &ray, const Vector3 &ray_origin, const Real eps, Shape **hit_shape, Real &t_val, Vector3 &b_coord, bool with_simd){
    // base case
    if (bvh.shapes.size()==1){
       // std::cout << "hit base" << std::endl;
        Real temp_t_val;
        Shape* temp_shape = bvh.shapes.at(0);
        if (auto *sph = std::get_if<Sphere>(temp_shape)){
            temp_t_val = hit_sphere(*sph, ray, ray_origin);
            if (temp_t_val > 0.0){
                if ((t_val < 0.0 || temp_t_val < t_val)&& distance(ray_origin, ray_origin + ray * temp_t_val)> eps){
                    *hit_shape = temp_shape;
                    t_val = temp_t_val;
                    return true;
                } else {
                    return false;
                }
            } else {
                return false;
            }
        } else if (auto *tri = std::get_if<Triangle>(temp_shape)){
            Vector3 p0 = tri->mesh->positions.at(tri->mesh->indices.at(tri->face_index).x);
            Vector3 p1 = tri->mesh->positions.at(tri->mesh->indices.at(tri->face_index).y);
            Vector3 p2 = tri->mesh->positions.at(tri->mesh->indices.at(tri->face_index).z);
            if ( get_tri_intersect(p0, p1, p2, ray, ray_origin, eps, b_coord, temp_t_val)){
                if ((t_val < 0.0 || temp_t_val < t_val)&& distance(ray_origin, ray_origin + ray * temp_t_val)> eps){
                    *hit_shape = temp_shape;
                    t_val = temp_t_val;
                    return true;
                } else {
                    return false;
                }
            } else {
                return false;
            }
        } else {
            assert(false);
        }
    } else {
        bool l_hit = false, r_hit = false;
        if (with_simd){
            int hits = hit_boxes_simd(bvh, ray, ray_origin, eps, infinity<Real>());
            //std::cout << hits << std::endl;
            if (hits == 3){
                // if (hit_box(*bvh.left, ray, ray_origin, eps, infinity<Real>()) ||hit_box(*bvh.right, ray, ray_origin, eps, infinity<Real>())){
                //     std::cout << "expected hit: " << bvh.a << "," << bvh.b << std::endl;
                //     std::cout << bvh.left->a << "," << bvh.left->b << " hitbox: "<<hit_box(*bvh.left, ray, ray_origin, eps, infinity<Real>()) <<std::endl;
                //     std::cout << bvh.right->a << "," << bvh.right->b << " hitbox: "<<hit_box(*bvh.right, ray, ray_origin, eps, infinity<Real>()) <<std::endl;

                // }

                return false;
            }
            if (hits == 0){
                l_hit = hit_bvh(*bvh.left, ray, ray_origin, eps, hit_shape, t_val, b_coord, with_simd);
                r_hit = hit_bvh(*bvh.right, ray, ray_origin, eps, hit_shape, t_val, b_coord, with_simd);
                return l_hit || r_hit;

                // if (ray[bvh.axis] < 0.0){
                //     r_hit = hit_bvh(*bvh.right, ray, ray_origin, eps, hit_shape, t_val, b_coord, with_simd);
                //     if (!r_hit) return false;
                //     l_hit = hit_bvh(*bvh.left, ray, ray_origin, eps, hit_shape, t_val, b_coord, with_simd);
                //     return l_hit || r_hit;
                // } else {
                //     l_hit = hit_bvh(*bvh.left, ray, ray_origin, eps, hit_shape, t_val, b_coord, with_simd);
                //     if (!l_hit) return false;
                //     r_hit = hit_bvh(*bvh.right, ray, ray_origin, eps, hit_shape, t_val, b_coord, with_simd);
                //     return l_hit || r_hit;
                // }

            } else if (hits==2){
                // if (hit_box(*bvh.left, ray, ray_origin, eps, infinity<Real>())){
                //     std::cout << "expected hit: " << hits << " "<< bvh.a << "," << bvh.b << std::endl;
                //     std::cout << bvh.left->a << "," << bvh.left->b << " hitbox: "<<hit_box(*bvh.left, ray, ray_origin, eps, infinity<Real>()) <<std::endl;
                //     std::cout << bvh.right->a << "," << bvh.right->b << " hitbox: "<<hit_box(*bvh.right, ray, ray_origin, eps, infinity<Real>()) <<std::endl;

                // }
                return hit_bvh(*bvh.right, ray, ray_origin, eps, hit_shape, t_val, b_coord, with_simd);
            }
            else if (hits==1){
                //  if (hit_box(*bvh.right, ray, ray_origin, eps, infinity<Real>())){
                //     std::cout << "expected hit: " << hits << " "<< bvh.a << "," << bvh.b << std::endl;
                //     std::cout << bvh.left->a << "," << bvh.left->b << " hitbox: "<<hit_box(*bvh.left, ray, ray_origin, eps, infinity<Real>()) <<std::endl;
                //     std::cout << bvh.right->a << "," << bvh.right->b << " hitbox: "<<hit_box(*bvh.right, ray, ray_origin, eps, infinity<Real>()) <<std::endl;

                // }
                return hit_bvh(*bvh.left, ray, ray_origin, eps, hit_shape, t_val, b_coord, with_simd);
            } else {
                //             if (hit_box(*bvh.left, ray, ray_origin, eps, infinity<Real>()) ||hit_box(*bvh.right, ray, ray_origin, eps, infinity<Real>())){
                //     std::cout << "expected hit: " << bvh.a << "," << bvh.b << std::endl;
                //     std::cout << bvh.left->a << "," << bvh.left->b << " hitbox: "<<hit_box(*bvh.left, ray, ray_origin, eps, infinity<Real>()) <<std::endl;
                //     std::cout << bvh.right->a << "," << bvh.right->b << " hitbox: "<<hit_box(*bvh.right, ray, ray_origin, eps, infinity<Real>()) <<std::endl;

                // }
                return false;
            }
        } else {
            if (!hit_box(bvh, ray, ray_origin, eps, infinity<Real>())){
                return false;
            } else {
                l_hit = hit_bvh(*bvh.left, ray, ray_origin, eps, hit_shape, t_val, b_coord, with_simd);
                r_hit = hit_bvh(*bvh.right, ray, ray_origin, eps, hit_shape, t_val, b_coord, with_simd);
                return l_hit || r_hit;
            }
        }

        //std::cout << "hit" << std::endl;

    }
}