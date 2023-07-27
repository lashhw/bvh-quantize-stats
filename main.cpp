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

    std::vector<float> len_x, len_y, len_z;
    std::vector<uint32_t> depths;
    std::queue<std::pair<size_t, uint32_t>> queue;
    queue.emplace(0, 0);
    while (!queue.empty()) {
        auto [curr_idx, depth] = queue.front();

        node_t &curr_node = bvh.nodes[curr_idx];
        queue.pop();

        len_x.push_back(curr_node.bounds[1] - curr_node.bounds[0]);
        len_y.push_back(curr_node.bounds[3] - curr_node.bounds[2]);
        len_z.push_back(curr_node.bounds[5] - curr_node.bounds[4]);
        depths.push_back(depth);

        if (!curr_node.is_leaf()) {
            queue.emplace(curr_node.first_child_or_primitive, depth + 1);
            queue.emplace(curr_node.first_child_or_primitive + 1, depth + 1);
        }
    }

    std::ofstream len_x_file("len_x.bin", std::ios::out | std::ios::binary);
    std::ofstream len_y_file("len_y.bin", std::ios::out | std::ios::binary);
    std::ofstream len_z_file("len_z.bin", std::ios::out | std::ios::binary);
    std::ofstream depths_file("depths.bin", std::ios::out | std::ios::binary);
    len_x_file.write((char*)(len_x.data()), len_x.size() * sizeof(float));
    len_y_file.write((char*)(len_y.data()), len_y.size() * sizeof(float));
    len_z_file.write((char*)(len_z.data()), len_z.size() * sizeof(float));
    depths_file.write((char*)(depths.data()), depths.size() * sizeof(uint32_t));
}
