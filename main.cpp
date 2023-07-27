#include <iostream>
#include <vector>
#include <fstream>
#include <bvh/triangle.hpp>
#include <bvh/bvh.hpp>
#include <bvh/sweep_sah_builder.hpp>
#include "happly/happly.h"

typedef bvh::Bvh<float> bvh_t;
typedef bvh::Triangle<float> triangle_t;
typedef bvh::Vector3<float> vector_t;
typedef bvh::SweepSahBuilder<bvh_t> builder_t;
typedef bvh_t::Node node_t;

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "usage: ./a.out MODEL_FILE" << std::endl;
        exit(EXIT_FAILURE);
    }

    char *model_filename = argv[1];

    std::cout << "model_filename = " << model_filename << std::endl;

    happly::PLYData ply_data(model_filename);
    std::vector<std::array<double, 3>> v_pos = ply_data.getVertexPositions();
    std::vector<std::vector<size_t>> f_idx = ply_data.getFaceIndices<size_t>();

    std::vector<triangle_t> triangles;
    for (auto &face : f_idx) {
        triangles.emplace_back(vector_t((float)v_pos[face[0]][0], (float)v_pos[face[0]][1], (float)v_pos[face[0]][2]),
                               vector_t((float)v_pos[face[1]][0], (float)v_pos[face[1]][1], (float)v_pos[face[1]][2]),
                               vector_t((float)v_pos[face[2]][0], (float)v_pos[face[2]][1], (float)v_pos[face[2]][2]));
    }

    auto [bboxes, centers] = bvh::compute_bounding_boxes_and_centers(triangles.data(), triangles.size());
    auto global_bbox = bvh::compute_bounding_boxes_union(bboxes.get(), triangles.size());
    std::cout << "global_bbox = ("
              << global_bbox.min[0] << ", " << global_bbox.min[1] << ", " << global_bbox.min[2] << "), ("
              << global_bbox.max[0] << ", " << global_bbox.max[1] << ", " << global_bbox.max[2] << ")" << std::endl;

    std::cout << "building..." << std::endl;
    bvh_t bvh;
    builder_t builder(bvh);
    builder.build(global_bbox, bboxes.get(), centers.get(), triangles.size());

    std::vector<float> plane_x, plane_y, plane_z;
    std::vector<uint32_t> depths;
    std::queue<std::pair<size_t, uint32_t>> queue;
    queue.emplace(0, 0);
    while (!queue.empty()) {
        auto [curr_idx, depth] = queue.front();

        node_t &curr_node = bvh.nodes[curr_idx];
        queue.pop();

        plane_x.push_back(curr_node.bounds[0]);
        plane_x.push_back(curr_node.bounds[1]);
        plane_y.push_back(curr_node.bounds[2]);
        plane_y.push_back(curr_node.bounds[3]);
        plane_z.push_back(curr_node.bounds[4]);
        plane_z.push_back(curr_node.bounds[5]);
        depths.push_back(depth);
        depths.push_back(depth);

        if (!curr_node.is_leaf()) {
            queue.emplace(curr_node.first_child_or_primitive, depth + 1);
            queue.emplace(curr_node.first_child_or_primitive + 1, depth + 1);
        }
    }

    std::ofstream plane_x_file("plane_x.bin", std::ios::out | std::ios::binary);
    std::ofstream plane_y_file("plane_y.bin", std::ios::out | std::ios::binary);
    std::ofstream plane_z_file("plane_z.bin", std::ios::out | std::ios::binary);
    std::ofstream depths_file("depths.bin", std::ios::out | std::ios::binary);
    plane_x_file.write((char*)(plane_x.data()), plane_x.size() * sizeof(float));
    plane_y_file.write((char*)(plane_y.data()), plane_y.size() * sizeof(float));
    plane_z_file.write((char*)(plane_z.data()), plane_z.size() * sizeof(float));
    depths_file.write((char*)(depths.data()), depths.size() * sizeof(uint32_t));
}
