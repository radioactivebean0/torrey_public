#include "wide_bvh.h"


void set_surrounding_wide_aabb(std::shared_ptr<WideAABB> &parent, const std::shared_ptr<WideAABB> &a, const std::shared_ptr<WideAABB> &b){
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
void set_surrounding_wide_aabb(std::shared_ptr<WideAABB> &parent, const std::shared_ptr<WideAABB> &a, const std::shared_ptr<WideAABB> &b, const std::shared_ptr<WideAABB> &c){
    parent->a = min(min(a->a, b->a), c->a);
    parent->b = max(max(a->b, b->b), c->b);
    parent->shapes = std::vector<Shape*>();
    parent->shapes.insert(parent->shapes.end(), a->shapes.begin(), a->shapes.end());
    parent->shapes.insert(parent->shapes.end(), b->shapes.begin(), b->shapes.end());
    parent->shapes.insert(parent->shapes.end(), c->shapes.begin(), c->shapes.end());
    return;
}
void set_surrounding_wide_aabb(std::shared_ptr<WideAABB> &parent, const std::shared_ptr<WideAABB> &a, const std::shared_ptr<WideAABB> &b, const std::shared_ptr<WideAABB> &c, const std::shared_ptr<WideAABB> &d){
    parent->a = min(min(min(a->a, b->a), c->a), d->a);
    parent->b = max(max(max(a->b, b->b), c->b), d->b);
    parent->shapes = std::vector<Shape*>();
    parent->shapes.insert(parent->shapes.end(), a->shapes.begin(), a->shapes.end());
    parent->shapes.insert(parent->shapes.end(), b->shapes.begin(), b->shapes.end());
    parent->shapes.insert(parent->shapes.end(), c->shapes.begin(), c->shapes.end());
    parent->shapes.insert(parent->shapes.end(), d->shapes.begin(), d->shapes.end());
    return;
}
std::shared_ptr<WideAABB> build_wide_bvh_helper(std::vector<std::shared_ptr<WideAABB>> &boxes, size_t start, size_t end){
    // base case
    if (end-start == 1) {
        return boxes.at(start);
    } 
    std::shared_ptr<WideAABB> node = std::make_shared<WideAABB>();
    uint32_t axis = 0;

    Real min_x = (*std::min_element(boxes.begin()+start, boxes.begin()+end, wide_aabb_x_compare))->a.x;
    Real min_y = (*std::min_element(boxes.begin()+start, boxes.begin()+end, wide_aabb_y_compare))->a.y;
    Real min_z = (*std::min_element(boxes.begin()+start, boxes.begin()+end, wide_aabb_z_compare))->a.z;
    Real max_x = (*std::max_element(boxes.begin()+start, boxes.begin()+end, wide_aabb_x_compare_max))->b.x;
    Real max_y = (*std::max_element(boxes.begin()+start, boxes.begin()+end, wide_aabb_y_compare_max))->b.y;
    Real max_z = (*std::max_element(boxes.begin()+start, boxes.begin()+end, wide_aabb_z_compare_max))->b.z;
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

    auto comparator = (axis == 0) ? wide_aabb_x_compare
                    : (axis == 1) ? wide_aabb_y_compare
                                  : wide_aabb_z_compare;
    // other base cases
    if (end - start == 2){
        if (comparator(boxes.at(start), boxes.at(start+1))) {
            node->c1 = boxes.at(start);
            node->c2 = boxes.at(start+1);
        } else {
            node->c1 = boxes.at(start+1);
            node->c2 = boxes.at(start);
        }
        set_surrounding_wide_aabb(node, node->c1, node->c2);
        return node;
    } else if (end - start == 3){
        if (comparator(boxes.at(start), boxes.at(start+1))) {
            if (comparator(boxes.at(start+1), boxes.at(start+2))){
                node->c1 = boxes.at(start);
                node->c2 = boxes.at(start+1);
                node->c3 = boxes.at(start+2);
            } else if (comparator(boxes.at(start+2), boxes.at(start))){
                node->c1 = boxes.at(start+2);
                node->c2 = boxes.at(start);
                node->c3 = boxes.at(start+1);
            } else {
                node->c1 = boxes.at(start);
                node->c2 = boxes.at(start+2);
                node->c3 = boxes.at(start+1);
            }
        } else {
            if (comparator(boxes.at(start), boxes.at(start+2))){
                node->c1 = boxes.at(start+1);
                node->c2 = boxes.at(start);
                node->c3 = boxes.at(start+2);
            } else if (comparator(boxes.at(start+2), boxes.at(start+1))){
                node->c1 = boxes.at(start+2);
                node->c2 = boxes.at(start+1);
                node->c3 = boxes.at(start);
            } else {
                node->c1 = boxes.at(start+1);
                node->c2 = boxes.at(start+2);
                node->c3 = boxes.at(start);
            }
        }
    } else if (end - start == 4){
        node->c1 = build_wide_bvh_helper(boxes, start, start);
        node->c2 = build_wide_bvh_helper(boxes, start+1, start+1);
        node->c3 = build_wide_bvh_helper(boxes, start+2, start+2);
        node->c4 = build_wide_bvh_helper(boxes, start+3, start+3);
        set_surrounding_wide_aabb(node, node->c1, node->c2, node->c3, node->c4);
    } else {
        // sort subvector on chosen axis
        // std::sort(boxes.begin()+start, boxes.begin()+end, comparator);
        // // split apart and recurse
        // size_t middle = start + (end-start)/2;
        // node->left = build_bvh_helper(boxes, start, middle, split_method);
        // node->right = build_bvh_helper(boxes, middle, end, split_method);
        // set_surrounding_aabb(node, node->left, node->right);
        // return node;
    }
}

WideAABB build_wide_bvh(std::vector<Shape> &shapes){
    std::vector<std::shared_ptr<WideAABB>> boxes;
    Vector3 p0, p1, p2;

    for (int shape = 0; shape < shapes.size(); ++shape){
        std::shared_ptr<WideAABB> box = std::make_shared<WideAABB>();
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
    }
    return *build_wide_bvh_helper(boxes, 0, boxes.size());
}



int hit_wide_box_simd(const WideAABB &box, const Vector3 &ray, const Vector3 &lookfrom, Real t_min, Real t_max){
    
}