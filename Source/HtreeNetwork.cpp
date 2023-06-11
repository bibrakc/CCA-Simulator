/*
BSD 3-Clause License

Copyright (c) 2023, Bibrak Qamar

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "HtreeNetwork.hpp"
#include "HtreeNode.hpp"

#include "operators.hpp"
#include "types.hpp"

#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <stdlib.h>
#include <utility>
#include <vector>

// Increase the lane size at each recursive joint
#define BANDWIDTH_SCALE_FACTOR 2

Coordinates
find_min(const Coordinates& c1, const Coordinates& c2, const Coordinates& c3, const Coordinates& c4)
{
    Coordinates coordinates_min;
    coordinates_min.first = c1.first;
    coordinates_min.second = c1.second;

    // Find the minimum x-values
    if (c2.first < coordinates_min.first)
        coordinates_min.first = c2.first;
    if (c3.first < coordinates_min.first)
        coordinates_min.first = c3.first;
    if (c4.first < coordinates_min.first)
        coordinates_min.first = c4.first;

    // Find the minimum y-values
    if (c2.second < coordinates_min.second)
        coordinates_min.second = c2.second;
    if (c3.second < coordinates_min.second)
        coordinates_min.second = c3.second;
    if (c4.second < coordinates_min.second)
        coordinates_min.second = c4.second;

    return coordinates_min;
}

Coordinates
find_max(const Coordinates& c1, const Coordinates& c2, const Coordinates& c3, const Coordinates& c4)
{
    Coordinates coordinates_max;
    coordinates_max.first = c1.first;
    coordinates_max.second = c1.second;

    // Find the maximum x-values
    if (c2.first > coordinates_max.first)
        coordinates_max.first = c2.first;
    if (c3.first > coordinates_max.first)
        coordinates_max.first = c3.first;
    if (c4.first > coordinates_max.first)
        coordinates_max.first = c4.first;

    // Find the maximum y-values
    if (c2.second > coordinates_max.second)
        coordinates_max.second = c2.second;
    if (c3.second > coordinates_max.second)
        coordinates_max.second = c3.second;
    if (c4.second > coordinates_max.second)
        coordinates_max.second = c4.second;

    return coordinates_max;
}

std::pair<Coordinates, Coordinates>
union_coverage_ranges(const Coordinates& c1,
                      const Coordinates& c2,
                      const Coordinates& c3,
                      const Coordinates& c4)
{
    return std::pair<Coordinates, Coordinates>(find_min(c1, c2, c3, c4), find_max(c1, c2, c3, c4));
}

u_int32_t
find_closest_value(const std::vector<u_int32_t>& values, u_int32_t point)
{
    int left = 0;
    int right = values.size() - 1;
    u_int32_t closest = values[0];
    int point_signed_integer = static_cast<int>(point);

    while (left <= right) {
        int mid = (left + right) / 2;

        int mid_value_signed_integer = static_cast<int>(values[mid]);

        if (abs(mid_value_signed_integer - point_signed_integer) <
            abs(static_cast<int>(closest) - point_signed_integer)) {
            closest = values[mid];
        }

        if (values[mid] > point) {
            right = mid - 1;
        } else {
            left = mid + 1;
        }
    }
    return closest;
}

std::shared_ptr<HtreeNode>
create_vertical(int hx,
                int hy,
                const std::vector<u_int32_t>& all_possible_rows,
                const std::vector<u_int32_t>& all_possible_cols,
                int initial_row,
                int initial_col,
                int hx_factor,
                int hy_factor,
                int current_row,
                int current_col,
                Coordinates coverage_top_left_in,
                Coordinates coverage_bottom_right_in,
                int depth,
                u_int32_t bandwidth_max,
                u_int32_t& index);

std::shared_ptr<HtreeNode>
create_horizontal(int hx,
                  int hy,
                  const std::vector<u_int32_t>& all_possible_rows,
                  const std::vector<u_int32_t>& all_possible_cols,
                  int initial_row,
                  int initial_col,
                  int hx_factor,
                  int hy_factor,
                  int current_row,
                  int current_col,
                  Coordinates coverage_top_left_in,
                  Coordinates coverage_bottom_right_in,
                  int depth,
                  u_int32_t bandwidth_max,
                  u_int32_t& index)
{

    if (depth < 0) {
        return nullptr;
    }
    std::shared_ptr<HtreeNode> left = create_vertical(
        hx,
        hy,
        all_possible_rows,
        all_possible_cols,
        initial_row,
        initial_col,
        hx_factor,
        2 * hy_factor,
        current_row,
        current_col - (initial_col / hy_factor),
        coverage_top_left_in,
        Coordinates(coverage_bottom_right_in.first - coverage_bottom_right_in.first / 2,
                    coverage_bottom_right_in.second),
        depth - 1,
        bandwidth_max,
        index);

    std::shared_ptr<HtreeNode> right =
        create_vertical(hx,
                        hy,
                        all_possible_rows,
                        all_possible_cols,
                        initial_row,
                        initial_col,
                        hx_factor,
                        2 * hy_factor,
                        current_row,
                        current_col + (initial_col / hy_factor),
                        Coordinates(coverage_top_left_in.first + coverage_bottom_right_in.first / 2,
                                    coverage_top_left_in.second),
                        coverage_bottom_right_in,
                        depth - 1,
                        bandwidth_max,
                        index);

    u_int32_t out_bandwidth_value = 0;
    u_int32_t in_bandwidth_value = 0;
    std::shared_ptr<HtreeNode> center = nullptr;

    // Set the bandwidth
    if (depth == 0) {
        // TODO: Later instead of hardcoding `4` use the shape of the sink cell to determine
        // the value. Or some other initial thickness of the link
        out_bandwidth_value = 4;
        in_bandwidth_value = 0;
    } else {
        out_bandwidth_value = right->out_bandwidth * BANDWIDTH_SCALE_FACTOR;

        if (out_bandwidth_value > bandwidth_max) {
            out_bandwidth_value = bandwidth_max;
        }

        in_bandwidth_value = right->out_bandwidth;
    }

    // This means that it is an end node
    if (depth == 0) {
        center = std::make_shared<HtreeNode>(
            index,
            find_closest_value(all_possible_cols, current_col) - 1, // -1 for C zero-based index
            find_closest_value(all_possible_rows, current_row) - 1, // -1 for C zero-based index
            in_bandwidth_value,
            out_bandwidth_value);

        center->in_first = nullptr;
        center->in_second = nullptr;
        index++;

        center->coverage_top_left = Coordinates(center->cooridinates.first - (hy / 2),
                                                center->cooridinates.second - (hx / 2));
        center->coverage_bottom_right = Coordinates(center->cooridinates.first + (hy / 2),
                                                    center->cooridinates.second + (hx / 2));

        return center;
    }

    // Join left and right
    center = std::make_shared<HtreeNode>(
        index, current_col, current_row, in_bandwidth_value, out_bandwidth_value);
    index++;

    center->in_first = left;

    // I will put in recv_channel[2] of right, since my left's point of view I am his right
    center->remote_index_recv_channel_in_first = 2;
    Coordinates left_coverage_top_left;
    Coordinates left_coverage_bottom_right;
    if (left) {
        left->out = center;

        left->recv_channel[center->remote_index_recv_channel_in_first] =
            std::make_shared<FixedSizeQueue<CoordinatedOperon>>(center->in_bandwidth);
        left->send_channel[center->remote_index_recv_channel_in_first] =
            std::make_shared<FixedSizeQueue<CoordinatedOperon>>(center->in_bandwidth);

        // For left the `center` is to the right, therefore left will put in recv_channel[0] of
        // center
        left->remote_index_recv_channel_out = 0;
        left->local_index_send_channel_out = center->remote_index_recv_channel_in_first;

        center->local_index_send_channel_in_first = left->remote_index_recv_channel_out;

        center->recv_channel[center->local_index_send_channel_in_first] =
            std::make_shared<FixedSizeQueue<CoordinatedOperon>>(center->in_bandwidth);
        center->send_channel[center->local_index_send_channel_in_first] =
            std::make_shared<FixedSizeQueue<CoordinatedOperon>>(center->in_bandwidth);

        left_coverage_top_left = left->coverage_top_left;
        left_coverage_bottom_right = left->coverage_bottom_right;
    }
    // Work on right
    center->in_second = right;

    // For the center its `in_second` is right, therefore from the `in_second` point of view it
    // is 0. `center` will put in recv_channel[0] of right
    center->remote_index_recv_channel_in_second = 0;

    Coordinates right_coverage_top_left;
    Coordinates right_coverage_bottom_right;
    if (right) {
        right->out = center;

        right->recv_channel[center->remote_index_recv_channel_in_second] =
            std::make_shared<FixedSizeQueue<CoordinatedOperon>>(center->in_bandwidth);
        right->send_channel[center->remote_index_recv_channel_in_second] =
            std::make_shared<FixedSizeQueue<CoordinatedOperon>>(center->in_bandwidth);

        // For right the `center` is to the left, therefore right will put in recv_channel[2] of
        // center
        right->remote_index_recv_channel_out = 2;
        right->local_index_send_channel_out = center->remote_index_recv_channel_in_second;

        center->local_index_send_channel_in_second = right->remote_index_recv_channel_out;

        center->recv_channel[center->local_index_send_channel_in_second] =
            std::make_shared<FixedSizeQueue<CoordinatedOperon>>(center->in_bandwidth);
        center->send_channel[center->local_index_send_channel_in_second] =
            std::make_shared<FixedSizeQueue<CoordinatedOperon>>(center->in_bandwidth);

        right_coverage_top_left = right->coverage_top_left;
        right_coverage_bottom_right = right->coverage_bottom_right;
    }

    auto [union_coverage_top_left, union_coverage_bottom_right] =
        union_coverage_ranges(left_coverage_top_left,
                              left_coverage_bottom_right,
                              right_coverage_top_left,
                              right_coverage_bottom_right);

    center->coverage_top_left = union_coverage_top_left;
    center->coverage_bottom_right = union_coverage_bottom_right;

    return center;
}

std::shared_ptr<HtreeNode>
create_vertical(int hx,
                int hy,
                const std::vector<u_int32_t>& all_possible_rows,
                const std::vector<u_int32_t>& all_possible_cols,
                int initial_row,
                int initial_col,
                int hx_factor,
                int hy_factor,
                int current_row,
                int current_col,
                Coordinates coverage_top_left_in,
                Coordinates coverage_bottom_right_in,
                int depth,
                u_int32_t bandwidth_max,
                u_int32_t& index)
{

    if (depth < 0) {
        return nullptr;
    }
    std::shared_ptr<HtreeNode> up = create_horizontal(
        hx,
        hy,
        all_possible_rows,
        all_possible_cols,
        initial_row,
        initial_col,
        2 * hx_factor,
        hy_factor,
        current_row - (initial_row / hx_factor),
        current_col,
        coverage_top_left_in,
        Coordinates(coverage_bottom_right_in.first, coverage_bottom_right_in.second / 2),
        depth,
        bandwidth_max,
        index);

    std::shared_ptr<HtreeNode> down = create_horizontal(
        hx,
        hy,
        all_possible_rows,
        all_possible_cols,
        initial_row,
        initial_col,
        2 * hx_factor,
        hy_factor,
        current_row + (initial_row / hx_factor),
        current_col,
        Coordinates(coverage_top_left_in.first,
                    coverage_top_left_in.second + (coverage_bottom_right_in.second / 2) + 1),
        coverage_bottom_right_in,
        depth,
        bandwidth_max,
        index);

    u_int32_t out_bandwidth_value = 0;
    u_int32_t in_bandwidth_value = 0;

    out_bandwidth_value = up->out_bandwidth * BANDWIDTH_SCALE_FACTOR;

    if (out_bandwidth_value > bandwidth_max) {
        out_bandwidth_value = bandwidth_max;
    }

    in_bandwidth_value = up->out_bandwidth;

    std::shared_ptr<HtreeNode> center = std::make_shared<HtreeNode>(
        index, current_col, current_row, in_bandwidth_value, out_bandwidth_value);
    index++;

    center->in_first = up;

    // I will put in recv_channel[3] of up
    center->remote_index_recv_channel_in_first = 3;
    Coordinates up_coverage_top_left;
    Coordinates up_coverage_bottom_right;
    if (up) {
        up->out = center;

        // Center will out its operons to the recv_channel[3] of up
        up->recv_channel[center->remote_index_recv_channel_in_first] =
            std::make_shared<FixedSizeQueue<CoordinatedOperon>>(center->in_bandwidth);
        up->send_channel[center->remote_index_recv_channel_in_first] =
            std::make_shared<FixedSizeQueue<CoordinatedOperon>>(center->in_bandwidth);

        // For `up` the `center` is down, therefore `up` will put in recv_channel[1] of center
        up->remote_index_recv_channel_out = 1;

        // For end htree nodes set their local out
        up->local_index_send_channel_out = 3;

        center->local_index_send_channel_in_first = up->remote_index_recv_channel_out;

        center->recv_channel[center->local_index_send_channel_in_first] =
            std::make_shared<FixedSizeQueue<CoordinatedOperon>>(center->in_bandwidth);
        center->send_channel[center->local_index_send_channel_in_first] =
            std::make_shared<FixedSizeQueue<CoordinatedOperon>>(center->in_bandwidth);

        up_coverage_top_left = up->coverage_top_left;
        up_coverage_bottom_right = up->coverage_bottom_right;
    }

    // Work on the down
    center->in_second = down;

    // For the center its `in_second` is down, therefore from the `in_second` point of view it
    // is 1. `center` will put in recv_channel[1] of down
    center->remote_index_recv_channel_in_second = 1;
    Coordinates down_coverage_top_left;
    Coordinates down_coverage_bottom_right;
    if (down) {
        down->out = center;

        down->recv_channel[center->remote_index_recv_channel_in_second] =
            std::make_shared<FixedSizeQueue<CoordinatedOperon>>(center->in_bandwidth);
        down->send_channel[center->remote_index_recv_channel_in_second] =
            std::make_shared<FixedSizeQueue<CoordinatedOperon>>(center->in_bandwidth);

        // For down the `center` is up, therefore down will put in recv_channel[3] of center
        down->remote_index_recv_channel_out = 3;

        // For end htree nodes set their local out
        down->local_index_send_channel_out = 1;

        center->local_index_send_channel_in_second = down->remote_index_recv_channel_out;

        center->recv_channel[center->local_index_send_channel_in_second] =
            std::make_shared<FixedSizeQueue<CoordinatedOperon>>(center->in_bandwidth);
        center->send_channel[center->local_index_send_channel_in_second] =
            std::make_shared<FixedSizeQueue<CoordinatedOperon>>(center->in_bandwidth);

        down_coverage_top_left = down->coverage_top_left;
        down_coverage_bottom_right = down->coverage_bottom_right;
    }

    auto [union_coverage_top_left, union_coverage_bottom_right] =
        union_coverage_ranges(up_coverage_top_left,
                              up_coverage_bottom_right,
                              down_coverage_top_left,
                              down_coverage_bottom_right);

    center->coverage_top_left = union_coverage_top_left;
    center->coverage_bottom_right = union_coverage_bottom_right;

    return center;
}

int
get_htree_dims(int dim, int depth)
{

    if (depth == 0) {
        return dim;
    }

    if (depth == 1) {
        return 2 * dim;
    }
    return 2 * get_htree_dims(dim, depth - 1);
}

std::shared_ptr<HtreeNode>
create_htree(u_int32_t hx,
             u_int32_t hy,
             u_int32_t depth,
             u_int32_t bandwidth_max,
             const std::vector<u_int32_t>& all_possible_rows,
             const std::vector<u_int32_t>& all_possible_cols)
{
    if (depth == 0) {
        return nullptr;
    }
    // Create two verticals and join them
    u_int32_t index = 0;
    u_int32_t dim_x = get_htree_dims(hx, depth);
    u_int32_t dim_y = get_htree_dims(hy, depth);

    u_int32_t chip_center_x = (dim_x / 2); // row
    u_int32_t chip_center_y = (dim_y / 2); // col

    std::cout << "Chip dimenssions: " << dim_x << " x " << dim_y << "\n";
    std::cout << "Creating Htree with center at: (" << chip_center_x << ", " << chip_center_y
              << ")\n";

    int left_sub_htree_initial_row = chip_center_x;
    int left_sub_htree_initial_col = chip_center_y - (chip_center_y / 2);

    Coordinates chip_coverage_top_left = Coordinates(0, 0);
    Coordinates chip_coverage_bottom_right = Coordinates(dim_y - 1, dim_x - 1);

    std::shared_ptr<HtreeNode> left =
        create_vertical(hx,
                        hy,
                        all_possible_rows,
                        all_possible_cols,
                        chip_center_x,
                        chip_center_y,
                        2, // divisor factor for the next division in rows
                        4, // divisor factor for the next division in cols
                        left_sub_htree_initial_row,
                        left_sub_htree_initial_col,
                        chip_coverage_top_left,
                        Coordinates(chip_center_y - 1, chip_coverage_bottom_right.second),
                        depth - 1,
                        bandwidth_max,
                        index);

    int right_sub_htree_initial_row = chip_center_x;
    int right_sub_htree_initial_col = chip_center_y + (chip_center_y / 2);

    std::shared_ptr<HtreeNode> right =
        create_vertical(hx,
                        hy,
                        all_possible_rows,
                        all_possible_cols,
                        chip_center_x,
                        chip_center_y,
                        2, // divisor factor for the next division in rows
                        4, // divisor factor for the next division in cols
                        right_sub_htree_initial_row,
                        right_sub_htree_initial_col,
                        Coordinates(chip_center_y + 1, chip_coverage_top_left.second),
                        chip_coverage_bottom_right,
                        depth - 1,
                        bandwidth_max,
                        index);

    u_int32_t out_bandwidth_value = 0;
    u_int32_t in_bandwidth_value = 0;
    if (right) {
        out_bandwidth_value = right->out_bandwidth * BANDWIDTH_SCALE_FACTOR;
        if (out_bandwidth_value > bandwidth_max) {
            out_bandwidth_value = bandwidth_max;
        }
        in_bandwidth_value = right->out_bandwidth;
    }

    // Join left and right
    std::shared_ptr<HtreeNode> center = std::make_shared<HtreeNode>(
        index, chip_center_y, chip_center_x, in_bandwidth_value, out_bandwidth_value);
    index++;

    // Work on left
    center->in_first = left;

    center->remote_index_recv_channel_in_first = 2; // I will put in recv_channel[2]
    if (left) {
        left->out = center;

        left->recv_channel[center->remote_index_recv_channel_in_first] =
            std::make_shared<FixedSizeQueue<CoordinatedOperon>>(center->in_bandwidth);
        left->send_channel[center->remote_index_recv_channel_in_first] =
            std::make_shared<FixedSizeQueue<CoordinatedOperon>>(center->in_bandwidth);

        left->remote_index_recv_channel_out = 0; // Meaning it will use recv_channel[0]
        left->local_index_send_channel_out = center->remote_index_recv_channel_in_first;

        center->local_index_send_channel_in_first = left->remote_index_recv_channel_out;

        center->recv_channel[center->local_index_send_channel_in_first] =
            std::make_shared<FixedSizeQueue<CoordinatedOperon>>(center->in_bandwidth);
        center->send_channel[center->local_index_send_channel_in_first] =
            std::make_shared<FixedSizeQueue<CoordinatedOperon>>(center->in_bandwidth);
    }

    // Work on right
    center->in_second = right;

    // TODO: delete this as center will be delted and left and right parts of the base Htree
    // will be combined together
    center->remote_index_recv_channel_in_second = 0;
    if (right) {
        right->out = center;

        right->recv_channel[center->remote_index_recv_channel_in_second] =
            std::make_shared<FixedSizeQueue<CoordinatedOperon>>(center->in_bandwidth);
        right->send_channel[center->remote_index_recv_channel_in_second] =
            std::make_shared<FixedSizeQueue<CoordinatedOperon>>(center->in_bandwidth);

        right->remote_index_recv_channel_out = 2; // Meaning it will use recv_channel[2]
        right->local_index_send_channel_out = center->remote_index_recv_channel_in_second;

        center->local_index_send_channel_in_second = right->remote_index_recv_channel_out;

        center->recv_channel[center->local_index_send_channel_in_second] =
            std::make_shared<FixedSizeQueue<CoordinatedOperon>>(center->in_bandwidth);
        center->send_channel[center->local_index_send_channel_in_second] =
            std::make_shared<FixedSizeQueue<CoordinatedOperon>>(center->in_bandwidth);
    }

    center->out = nullptr;

    center->coverage_top_left = chip_coverage_top_left;
    center->coverage_bottom_right = chip_coverage_bottom_right;

    return center;
}

// Print the Htree in this order: top-left, down-left, top-right, down-right
void
print_htree(std::shared_ptr<HtreeNode>& root)
{
    if (root != nullptr) {
        print_htree(root->in_first);
        print_htree(root->in_second);

        std::cout << root;
    }
}
void
populate_id_to_ptr_vector(std::vector<std::shared_ptr<HtreeNode>>& htree_all_nodes,
                          std::shared_ptr<HtreeNode>& root)
{
    if (root != nullptr) {
        populate_id_to_ptr_vector(htree_all_nodes, root->in_first);
        populate_id_to_ptr_vector(htree_all_nodes, root->in_second);

        htree_all_nodes.push_back(root);
    }
}

// Traverse the Htree in this order: top-left, down-left, top-right, down-right
void
populate_coorodinates_to_ptr_map(std::map<Coordinates, std::shared_ptr<HtreeNode>>& htree_end_nodes,
                                 std::shared_ptr<HtreeNode>& root)
{
    if (root != nullptr) {
        populate_coorodinates_to_ptr_map(htree_end_nodes, root->in_first);
        populate_coorodinates_to_ptr_map(htree_end_nodes, root->in_second);

        // End node in the Htree
        if (root->in_first == nullptr || root->in_second == nullptr) {
            htree_end_nodes[root->cooridinates] = root;
        }
    }
}

void
print_details_of_an_htree_node(std::vector<std::shared_ptr<HtreeNode>>& htree_all_nodes,
                               u_int32_t id)
{

    std::cout << "Htree Node: " << htree_all_nodes[id]->id << " vector index = " << id << "\n";
    if (htree_all_nodes[id]->in_first) {
        std::cout << "\tin_first: " << htree_all_nodes[id]->in_first->id
                  << ", remote relationship: "
                  << htree_all_nodes[id]->remote_index_recv_channel_in_first
                  << ", local relationship: "
                  << htree_all_nodes[id]->local_index_send_channel_in_first
                  << " in_bandwidth: " << htree_all_nodes[id]->in_bandwidth << "\n";
    }
    if (htree_all_nodes[id]->in_second) {
        std::cout << "\tin_second: " << htree_all_nodes[id]->in_second->id
                  << ", remote relationship: "
                  << htree_all_nodes[id]->remote_index_recv_channel_in_second
                  << ", local relationship: "
                  << htree_all_nodes[id]->local_index_send_channel_in_second << "\n";
    }

    if (htree_all_nodes[id]->out) {
        std::cout << "\tout: " << htree_all_nodes[id]->out->id
                  << ", remote relationship: " << htree_all_nodes[id]->remote_index_recv_channel_out
                  << ", local relationship: " << htree_all_nodes[id]->local_index_send_channel_out
                  << ", out_bandwidth: " << htree_all_nodes[id]->out_bandwidth << "\n";
    }
}

void
HtreeNetwork::construct_htree_network()
{
    if (this->hdepth == 0) {
        return;
    }

    u_int32_t total_rows_cols_with_htree_nodes = static_cast<u_int32_t>(pow(2, this->hdepth));

    u_int32_t first_row = std::ceil(this->hx / 2.0);
    u_int32_t first_col = std::ceil(this->hy / 2.0);

    u_int32_t range_rows = total_rows_cols_with_htree_nodes * this->hx;
    u_int32_t range_cols = total_rows_cols_with_htree_nodes * this->hy;

    // All possible coordiantes per row and cols. Useful to find the exact end node using the
    // find_closest_value() method
    std::vector<u_int32_t> all_possible_rows;
    std::vector<u_int32_t> all_possible_cols;

    for (u_int32_t i = first_row; i < range_rows; i += this->hx) {
        all_possible_rows.push_back(i);
    }

    for (u_int32_t i = first_col; i < range_cols; i += this->hy) {
        all_possible_cols.push_back(i);
    }

    // Print the vector elements
    /* std::cout << "All Possible Rows" << std::endl;
    for (u_int32_t i = 0; i < this->all_possible_rows.size(); ++i) {
        std::cout << all_possible_rows[i] << " ";
    }
    std::cout << std::endl;

    std::cout << "All Possible Cols" << std::endl;
    for (u_int32_t i = 0; i < this->all_possible_cols.size(); ++i) {
        std::cout << all_possible_cols[i] << " ";
    }
    std::cout << std::endl; */

    root = create_htree(this->hx,
                        this->hy,
                        this->hdepth,
                        this->bandwidth_max,
                        all_possible_rows,
                        all_possible_cols);
    std::cout << "root index = " << root->id << "\n";

    // Print the tree using traversal
    /*     cout << "Traversal:\n";
        print_htree(root);
        cout << endl; */

    populate_coorodinates_to_ptr_map(this->htree_end_nodes, root);
    /*
        std::cout << "Printing the map: " << std::endl;
        // Print the map elements
        for (const auto& entry : this->htree_end_nodes) {
            std::cout << entry.first << ": ";
            std::cout << entry.second;
        }

        std::cout << std::endl;
     */
    populate_id_to_ptr_vector(this->htree_all_nodes, root);

    /*     std::cout << "Printing the vector of all htree nodes: " << std::endl;
        // Print the vector elements
        for (const auto& htree_node : this->htree_all_nodes) {
            std::cout << htree_node;
        }
        std::cout << std::endl; */

    // Remove the root
    this->htree_all_nodes.pop_back();

    // Merge two halves of the Htree and remove the root
    std::shared_ptr<HtreeNode> root_in_first_temp = root->in_first;
    root_in_first_temp->out = root->in_second;
    root->in_second->out = root->in_first;

    print_details_of_an_htree_node(this->htree_all_nodes, 0);
    print_details_of_an_htree_node(this->htree_all_nodes, root_in_first_temp->id);
}
