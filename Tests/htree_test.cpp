#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <queue>
#include <vector>

using namespace std;

typedef std::pair<u_int32_t, u_int32_t> Coordinates;

// CC Id and numerical payload instead of action for development
typedef std::pair<u_int32_t, u_int32_t> Operon;

// For Htree routing. Later take the Coordinates out and send to the CCA chip though the sink cell
typedef std::pair<Coordinates, Operon> CoordinatedOperon;

Coordinates
findMin(const Coordinates& c1, const Coordinates& c2, const Coordinates& c3, const Coordinates& c4)
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
findMax(const Coordinates& c1, const Coordinates& c2, const Coordinates& c3, const Coordinates& c4)
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
    return std::pair<Coordinates, Coordinates>(findMin(c1, c2, c3, c4), findMax(c1, c2, c3, c4));
}

// Overload printing for Coordinates
std::ostream&
operator<<(std::ostream& os, const Coordinates coordinates)
{
    os << "(" << coordinates.first << ", " << coordinates.second << ")";
    return os;
}

// Fixed size queue to be used for storing Operons. The fixed queue size is the bandwidth at that
// HtreeNode
template<typename T>
class FixedSizeQueue
{
  private:
    std::queue<T> underlying_queue;
    u_int32_t size_max;

  public:
    FixedSizeQueue(u_int32_t size_max_in)
        : size_max(size_max_in)
    {
    }

    bool push(const T& value)
    {
        // Not able to enqueue. Return false
        if (underlying_queue.size() == this->size_max) {
            return false;
        }
        this->underlying_queue.push(value);
        return true;
    }

    // Dequeue FIFO
    T front() const { return underlying_queue.front(); }
    // Pop
    void pop() { underlying_queue.pop(); }

    // Recturn the current size of the queue
    u_int32_t size() const { return underlying_queue.size(); }
};

// TODO: see if we keep this globally here or use the HtreeNode function?
bool
is_coordinate_in_range(const Coordinates start, const Coordinates end, const Coordinates point)
{
    // Check if the point is within the range
    return ((point.first >= start.first) && (point.first <= end.first) &&
            (point.second >= start.second) && (point.second <= end.second));
}

// H-Tree Node
struct HtreeNode
{

    bool put_operon_from_sink_cell(const CoordinatedOperon operon)
    {
        return this->recv_channel_from_sink_cell->push(operon);
    }

    bool put_operon_from_within_htree_node(const CoordinatedOperon operon,
                                           u_int32_t relationship_of_sender)
    {
        std::shared_ptr<FixedSizeQueue<CoordinatedOperon>> recv_channel_to_use;
        if (this->recv_channel[relationship_of_sender]) {
            recv_channel_to_use = this->recv_channel[relationship_of_sender].value();
        } else {
            std::cerr << "Bug! Using a recv_channel that doesn't exist\n";
            exit(0);
        }
        return recv_channel_to_use->push(operon);
    }

    bool is_coordinate_in_range(const Coordinates point)
    {
        // Check if the point is within the range
        return ((point.first >= this->coverage_top_left.first) &&
                (point.first <= this->coverage_bottom_right.first) &&
                (point.second >= this->coverage_top_left.second) &&
                (point.second <= this->coverage_bottom_right.second));
    }

    bool is_end_htree_node() { return (!this->in_first && !this->in_second); }

    int id;
    Coordinates cooridinates;

    std::shared_ptr<HtreeNode> in_first;  // up or left
    std::shared_ptr<HtreeNode> in_second; // down or right
    std::shared_ptr<HtreeNode> out;       // outside of this htree

    // How to know if this node is up, down, left, or right of its neighbor node?
    // The neighbor node itself will set this to either 0, 1, 2 or 3.
    // 0: I am your left neighbor, which means that when you output operons to me please put
    // in my `recv_in_channel[0]`.
    // 1: Likewie this is up, meaning out in my `recv_in_channel[1]`
    // 2: right `recv_in_channel[2]`
    // 3: down `recv_in_channel[3]`
    u_int32_t relationship_with_my_out;

    u_int32_t relationship_with_my_in_first;
    u_int32_t relationship_with_my_in_second;

    int in_bandwidth;  // Number of channels per in lane
    int out_bandwidth; // Number of channels per out lane

    Coordinates coverage_top_left;
    Coordinates coverage_bottom_right;

    // ID of the sink cell that this HtreeNode connects to. Only if it is an end htree node.
    std::optional<u_int32_t> sink_cell_connector;

    // Only to be used for end htree nodes that connect with the CCA chip through sink cells
    std::shared_ptr<FixedSizeQueue<Operon>> send_channel_to_sink_cell;
    std::shared_ptr<FixedSizeQueue<CoordinatedOperon>> recv_channel_from_sink_cell;

    std::optional<std::shared_ptr<FixedSizeQueue<CoordinatedOperon>>> send_channel[4];
    std::optional<std::shared_ptr<FixedSizeQueue<CoordinatedOperon>>> recv_channel[4];

    /*
    std::shared_ptr<FixedSizeQueue<CoordinatedOperon>> send_out_channel;
    std::shared_ptr<FixedSizeQueue<CoordinatedOperon>> recv_out_channel;
    */

    HtreeNode(int index, int x, int y, int in_bandhwidth_in, int out_bandhwidth_in)
    {
        this->id = index;
        this->cooridinates.first = x;
        this->cooridinates.second = y;

        this->in_first = nullptr;
        this->in_second = nullptr;
        this->out = nullptr;

        this->in_bandwidth = in_bandhwidth_in;
        this->out_bandwidth = out_bandhwidth_in;

        // Later for each end node htree node that connects with a sink cell in the CCA chip these
        // will be assigned their proper values
        this->sink_cell_connector = std::nullopt;

        this->send_channel_to_sink_cell = nullptr;
        this->recv_channel_from_sink_cell = nullptr;

        // This means that it is an end htree node that is connected to a sink cell in the CCA chip
        if (in_bandhwidth_in == 0) {

            // End node has 4 links to the square type shape sink cell
            this->send_channel_to_sink_cell =
                std::make_shared<FixedSizeQueue<Operon>>(out_bandhwidth_in);
            this->recv_channel_from_sink_cell =
                std::make_shared<FixedSizeQueue<CoordinatedOperon>>(out_bandhwidth_in);

            /* this->send_in_channel[0] = nullptr;
            this->recv_in_channel[1] = nullptr;

            this->send_in_channel[0] = nullptr;
            this->recv_in_channel[1] = nullptr; */
        } /* else {

              this->send_in_first_channel =
                 std::make_shared<FixedSizeQueue<CoordinatedOperon>>(this->in_bandwidth);
             this->recv_in_first_channel =
                 std::make_shared<FixedSizeQueue<CoordinatedOperon>>(this->in_bandwidth);

             this->send_in_second_channel =
                 std::make_shared<FixedSizeQueue<CoordinatedOperon>>(this->in_bandwidth);
             this->recv_in_second_channel =
                 std::make_shared<FixedSizeQueue<CoordinatedOperon>>(this->in_bandwidth);
        } */

        /* this->send_out_channel =
            std::make_shared<FixedSizeQueue<CoordinatedOperon>>(this->out_bandwidth);
        this->recv_out_channel =
            std::make_shared<FixedSizeQueue<CoordinatedOperon>>(this->out_bandwidth); */

        for (int i = 0; i < 4; i++) {
            this->recv_channel[i] = std::nullopt;
            this->send_channel[i] = std::nullopt;
        }
    }
};

// Overload printing for HtreeNode
std::ostream&
operator<<(std::ostream& os, const HtreeNode* node)
{

    if (node->in_first == nullptr && node->in_second == nullptr) {
        os << "[" << node->id << " -> " << node->cooridinates
           << " {Coverage 1: " << node->coverage_top_left
           << ", Coverage 2: " << node->coverage_bottom_right << "}]\n";
    } else {
        os << node->id << " {Coverage 1: " << node->coverage_top_left
           << ", Coverage 2: " << node->coverage_bottom_right << "}\n";
    }
    return os;
}

int
findClosestValue(const std::vector<int>& values, int point)
{
    int left = 0;
    int right = values.size() - 1;
    int closest = values[0];

    while (left <= right) {
        int mid = (left + right) / 2;

        if (abs(values[mid] - point) < abs(closest - point)) {
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
                const std::vector<int>& all_possible_rows,
                const std::vector<int>& all_possible_cols,
                int initial_row,
                int initial_col,
                int hx_factor,
                int hy_factor,
                int current_row,
                int current_col,
                Coordinates coverage_top_left_in,
                Coordinates coverage_bottom_right_in,
                int depth,
                int& index);

std::shared_ptr<HtreeNode>
create_horizontal(int hx,
                  int hy,
                  const std::vector<int>& all_possible_rows,
                  const std::vector<int>& all_possible_cols,
                  int initial_row,
                  int initial_col,
                  int hx_factor,
                  int hy_factor,
                  int current_row,
                  int current_col,
                  Coordinates coverage_top_left_in,
                  Coordinates coverage_bottom_right_in,
                  int depth,
                  int& index)
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
                        index);

    u_int32_t out_bandwidth_value = 0;
    u_int32_t in_bandwidth_value = 0;
    std::shared_ptr<HtreeNode> center = nullptr;

    // Set the bandwidth
    if (depth == 0) {
        // TODO: Later instead of hardcoding `4` instead use the shape of the sink cell to determine
        // the value
        out_bandwidth_value = 4;
        in_bandwidth_value = 0;
    } else if (depth == 1) {
        out_bandwidth_value = 8;
        in_bandwidth_value = 4;
    } else {
        out_bandwidth_value = right->out_bandwidth * 2;
        in_bandwidth_value = right->out_bandwidth;
    }

    // This means that it is an end node
    if (depth == 0) {
        center = std::make_shared<HtreeNode>(
            index,
            findClosestValue(all_possible_cols, current_col) - 1, // -1 for C zero-based index
            findClosestValue(all_possible_rows, current_row) - 1, // -1 for C zero-based index
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
    // I will put in recv_channel[2] of up
    center->relationship_with_my_in_first = 2;
    Coordinates left_coverage_top_left;
    Coordinates left_coverage_bottom_right;
    if (left) {
        left->out = center;

        // For left the `center` is to the right, therefore left will put in recv_channel[0] of
        // center
        left->relationship_with_my_out = 0;

        left_coverage_top_left = left->coverage_top_left;
        left_coverage_bottom_right = left->coverage_bottom_right;
    }

    center->in_second = right;
    // For the center its `in_second` is right, therefore from the `in_second` point of view it is
    // 0. `center` will put in recv_channel[0] of right
    center->relationship_with_my_in_second = 0;
    Coordinates right_coverage_top_left;
    Coordinates right_coverage_bottom_right;
    if (right) {
        right->out = center;

        // For right the `center` is to the left, therefore right will put in recv_channel[2] of
        // center
        center->relationship_with_my_out = 2;

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
                const std::vector<int>& all_possible_rows,
                const std::vector<int>& all_possible_cols,
                int initial_row,
                int initial_col,
                int hx_factor,
                int hy_factor,
                int current_row,
                int current_col,
                Coordinates coverage_top_left_in,
                Coordinates coverage_bottom_right_in,
                int depth,
                int& index)
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
        index);

    u_int32_t out_bandwidth_value = 0;
    u_int32_t in_bandwidth_value = 0;

    out_bandwidth_value = up->out_bandwidth * 2;
    in_bandwidth_value = up->out_bandwidth;

    std::shared_ptr<HtreeNode> center = std::make_shared<HtreeNode>(
        index, current_col, current_row, in_bandwidth_value, out_bandwidth_value);
    index++;

    center->in_first = up;
    // I will put in recv_channel[3] of up
    center->relationship_with_my_in_first = 3;
    Coordinates up_coverage_top_left;
    Coordinates up_coverage_bottom_right;
    if (up) {
        up->out = center;

        // For up the `center` is down, therefore up will put in recv_channel[1] of center
        up->relationship_with_my_out = 1;

        up_coverage_top_left = up->coverage_top_left;
        up_coverage_bottom_right = up->coverage_bottom_right;
    }

    center->in_second = down;
    // For the center its `in_second` is down, therefore from the `in_second` point of view it is 1.
    // `center` will put in recv_channel[1] of down
    center->relationship_with_my_in_second = 1;
    Coordinates down_coverage_top_left;
    Coordinates down_coverage_bottom_right;
    if (down) {
        down->out = center;

        // For down the `center` is up, therefore down will put in recv_channel[3] of center
        center->relationship_with_my_out = 3;

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
create_htree(int hx,
             int hy,
             int depth,
             const std::vector<int>& all_possible_rows,
             const std::vector<int>& all_possible_cols)
{
    if (depth == 0) {
        return nullptr;
    }
    // Create two verticals and join them
    int index = 0;
    int dim_x = get_htree_dims(hx, depth);
    int dim_y = get_htree_dims(hy, depth);

    int chip_center_x = (dim_x / 2); // row
    int chip_center_y = (dim_y / 2); // col

    cout << "Chip dimenssions: " << dim_x << " x " << dim_y << "\n";
    cout << "Creating Htree with center at: (" << chip_center_x << ", " << chip_center_y << ")\n";

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
                        index);

    u_int32_t out_bandwidth_value = 0;
    u_int32_t in_bandwidth_value = 0;
    if (right) {
        out_bandwidth_value = right->out_bandwidth * 2;
        in_bandwidth_value = right->out_bandwidth;
    }

    // Join left and right
    std::shared_ptr<HtreeNode> center = std::make_shared<HtreeNode>(
        index, chip_center_y, chip_center_x, in_bandwidth_value, out_bandwidth_value);
    index++;

    center->in_first = left;
    center->relationship_with_my_in_first = 2; // I will put in recv_channel[2]
    if (left) {
        left->out = center;
        left->relationship_with_my_out = 0; // Meaning it will use recv_channel[0]
    }

    center->in_second = right;
    // TODO: delete this as center will be delted and left and right parts of the base Htree will be
    // combined together
    center->relationship_with_my_in_second = 0;
    if (right) {
        right->out = center;
        right->relationship_with_my_out = 2; // Meaning it will use recv_channel[2]
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

        cout << root;
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

// Overload printing for HtreeNode
void
print_details_of_an_htree_node(std::vector<std::shared_ptr<HtreeNode>>& htree_all_nodes,
                               u_int32_t id)
{

    std::cout << "Htree Node: " << id << "\n";
    if (htree_all_nodes[id]->in_first) {
        std::cout << "\tin_first: " << htree_all_nodes[id]->in_first->id
                  << ", relationship: " << htree_all_nodes[id]->relationship_with_my_in_first
                  << "\n";
    }
    if (htree_all_nodes[id]->in_second) {
        std::cout << "\tin_second: " << htree_all_nodes[id]->in_second->id
                  << ", relationship: " << htree_all_nodes[id]->relationship_with_my_in_second
                  << "\n";
    }

    if (htree_all_nodes[id]->out) {
        std::cout << "\tout: " << htree_all_nodes[id]->out->id
                  << ", relationship: " << htree_all_nodes[id]->relationship_with_my_out << "\n";
    }

    Coordinates cc(2, 3);
    std::cout << "Where to route " << cc << "?\n";

    // First check if this is the end htree node
    if (htree_all_nodes[id]->is_end_htree_node()) {
        // Does the route needs to go thought the sink channel?
        if (htree_all_nodes[id]->is_coordinate_in_range(cc)) {
            std::cout << "\tSend to sink cell\n";
        } else {
            std::cout << "\tSend to out\n";
        }
        // Check if it can go to `in_first`?
    } else if (is_coordinate_in_range(htree_all_nodes[id]->in_first->coverage_top_left,
                                      htree_all_nodes[id]->in_first->coverage_bottom_right,
                                      cc)) {

        std::cout << "\tSend to in_first\n";
    } else if (is_coordinate_in_range(htree_all_nodes[id]->in_second->coverage_top_left,
                                      htree_all_nodes[id]->in_second->coverage_bottom_right,
                                      cc)) {
        std::cout << "\tSend to in_second\n";
    } else {
        std::cout << "\tSend to out\n";
    }
}

int
main(int argc, char* argv[])
{
    if (argc < 4) {
        std::cout << "Usage: ./Htree_Creator.out <hx> <hy> <depth>" << std::endl;
        return 1;
    }

    int hx = std::atoi(argv[1]);
    int hy = std::atoi(argv[2]);
    int depth = std::atoi(argv[3]);

    std::cout << "hx: " << hx << " hy: " << hy << " depth: " << depth << std::endl;

    std::vector<int> all_possible_rows;
    std::vector<int> all_possible_cols;

    int total_rows_cols_with_htree_nodes = static_cast<int>(pow(2, depth));

    int first_row = std::ceil(hx / 2.0);
    int first_col = std::ceil(hy / 2.0);

    int range_rows = total_rows_cols_with_htree_nodes * hx;
    int range_cols = total_rows_cols_with_htree_nodes * hy;

    for (int i = first_row; i < range_rows; i += hx) {
        all_possible_rows.push_back(i);
    }

    for (int i = first_col; i < range_cols; i += hy) {
        all_possible_cols.push_back(i);
    }

    // Print the vector elements
    std::cout << "All Possible Rows" << std::endl;
    for (int i = 0; i < all_possible_rows.size(); ++i) {
        std::cout << all_possible_rows[i] << " ";
    }
    std::cout << std::endl;

    std::cout << "All Possible Cols" << std::endl;
    for (int i = 0; i < all_possible_cols.size(); ++i) {
        std::cout << all_possible_cols[i] << " ";
    }
    std::cout << std::endl;

    std::shared_ptr<HtreeNode> root = nullptr;

    // Insert nodes into the binary tree
    root = create_htree(hx, hy, depth, all_possible_rows, all_possible_cols);

    cout << "root index = " << root->id << "\n";

    // Print the tree using traversal
    /*     cout << "Traversal:\n";
        print_htree(root);
        cout << endl; */

    std::map<Coordinates, std::shared_ptr<HtreeNode>> htree_end_nodes;
    populate_coorodinates_to_ptr_map(htree_end_nodes, root);

    /*
    cout << "Printing the map: " << endl;
    // Print the map elements
    for (const auto& entry : htree_end_nodes) {
         std::cout << entry.first << ": " << entry.second;
    }

    std::cout << std::endl;
    */

    std::vector<std::shared_ptr<HtreeNode>> htree_all_nodes;
    populate_id_to_ptr_vector(htree_all_nodes, root);

    cout << "Printing the vector of all htree nodes: " << endl;
    // Print the vector elements
    for (const auto& htree_node : htree_all_nodes) {
        std::cout << htree_node;
    }
    std::cout << std::endl;

    print_details_of_an_htree_node(htree_all_nodes, 0);

    print_details_of_an_htree_node(htree_all_nodes, 6);

    print_details_of_an_htree_node(htree_all_nodes, 1);

    print_details_of_an_htree_node(htree_all_nodes, 4);

    return 0;
}
