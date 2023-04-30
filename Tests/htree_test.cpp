#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <vector>

using namespace std;

typedef std::pair<u_int32_t, u_int32_t> Coordinates;

Coordinates
findMin(const Coordinates& c1, const Coordinates& c2, const Coordinates& c3, const Coordinates& c4)
{
    Coordinates unionCoord;
    unionCoord.first = c1.first;
    unionCoord.second = c1.second;

    // Find the minimum x-values
    if (c2.first < unionCoord.first)
        unionCoord.first = c2.first;
    if (c3.first < unionCoord.first)
        unionCoord.first = c3.first;
    if (c4.first < unionCoord.first)
        unionCoord.first = c4.first;

    // Find the minimum y-values
    if (c2.second < unionCoord.second)
        unionCoord.second = c2.second;
    if (c3.second < unionCoord.second)
        unionCoord.second = c3.second;
    if (c4.second < unionCoord.second)
        unionCoord.second = c4.second;

    return unionCoord;
}
Coordinates
findMax(const Coordinates& c1, const Coordinates& c2, const Coordinates& c3, const Coordinates& c4)
{
    Coordinates unionCoord;
    unionCoord.first = c1.first;
    unionCoord.second = c1.second;

    // Find the maximum x-values
    if (c2.first > unionCoord.first)
        unionCoord.first = c2.first;
    if (c3.first > unionCoord.first)
        unionCoord.first = c3.first;
    if (c4.first > unionCoord.first)
        unionCoord.first = c4.first;

    // Find the maximum y-values
    if (c2.second > unionCoord.second)
        unionCoord.second = c2.second;
    if (c3.second > unionCoord.second)
        unionCoord.second = c3.second;
    if (c4.second > unionCoord.second)
        unionCoord.second = c4.second;

    return unionCoord;
}

std::pair<Coordinates, Coordinates>
union_coverage_ranges(const Coordinates& c1,
                      const Coordinates& c2,
                      const Coordinates& c3,
                      const Coordinates& c4)
{
    return std::pair<Coordinates, Coordinates>(findMin(c1, c2, c3, c4), findMax(c1, c2, c3, c4));
}

// Overload printing for Node
std::ostream&
operator<<(std::ostream& os, const Coordinates coordinates)
{
    os << "(" << coordinates.first << ", " << coordinates.second << ")";
    return os;
}

// H-Tree Node
struct Node
{
    int id;
    Coordinates cooridinates;

    std::shared_ptr<Node> in_first;  // up or left
    std::shared_ptr<Node> in_second; // down or right
    std::shared_ptr<Node> out;       // outside of this htree

    int in_bandwidth;
    int out_bandwidth;

    Coordinates coverage_top_left;
    Coordinates coverage_bottom_right;

    Node(int value, int x, int y, int in_bandhwidth_in, int out_bandhwidth_in)
    {
        id = value;
        cooridinates.first = x;
        cooridinates.second = y;

        in_first = nullptr;
        in_second = nullptr;
        out = nullptr;

        this->in_bandwidth = in_bandhwidth_in;
        this->out_bandwidth = out_bandhwidth_in;
    }
};

// Overload printing for Node
std::ostream&
operator<<(std::ostream& os, const Node* node)
{
    /*
     if (node->in_first == nullptr && node->in_second == nullptr) {
        os << "[" << node->id << " -> (" << node->cooridinate_x << ", " << node->cooridinate_y
           << "){out: " << node->out_bandwidth << ", in: " << node->in_bandwidth << "}], ";
    } else {
        os << "(" << node->id << ", out: " << node->out_bandwidth << ", in: " << node->in_bandwidth
           << "), ";
    } */
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
findClosestValue(const std::vector<int>& arr, int x)
{
    int left = 0;
    int right = arr.size() - 1;
    int closest = arr[0];

    while (left <= right) {
        int mid = (left + right) / 2;

        if (abs(arr[mid] - x) < abs(closest - x)) {
            closest = arr[mid];
        }

        if (arr[mid] > x) {
            right = mid - 1;
        } else {
            left = mid + 1;
        }
    }
    //std::cout << "closest for " << x << " is " << closest << "\n";

    return closest;
}

std::shared_ptr<Node>
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
                int& value);

std::shared_ptr<Node>
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
                  int& value)
{

    if (depth < 0) {
        return nullptr;
    }
    std::shared_ptr<Node> left = create_vertical(
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
        value);

    std::shared_ptr<Node> right =
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
                        value);

    u_int32_t out_bandwidth_value = 0;
    u_int32_t in_bandwidth_value = 0;
    std::shared_ptr<Node> center = nullptr;

    // This means that it is an end node
    if (depth == 0) {
        out_bandwidth_value = 4;
        in_bandwidth_value = 0;

        center = std::make_shared<Node>(
            value,
            findClosestValue(all_possible_cols, current_col) - 1, // -1 for C zero-based index
            findClosestValue(all_possible_rows, current_row) - 1, // -1 for C zero-based index
            in_bandwidth_value,
            out_bandwidth_value);

        center->in_first = nullptr;
        center->in_second = nullptr;
        value++;

        center->coverage_top_left = Coordinates(center->cooridinates.first - (hy / 2),
                                                center->cooridinates.second - (hx / 2));
        center->coverage_bottom_right = Coordinates(center->cooridinates.first + (hy / 2),
                                                    center->cooridinates.second + (hx / 2));

        return center;

    } else if (depth == 1) {
        out_bandwidth_value = 8;
        in_bandwidth_value = 4;
    } else {
        out_bandwidth_value = right->out_bandwidth * 2;
        in_bandwidth_value = right->out_bandwidth;
    }

    // Join left and right
    center = std::make_shared<Node>(
        value, current_col, current_row, in_bandwidth_value, out_bandwidth_value);
    value++;

    center->in_first = left;
    Coordinates left_coverage_top_left;
    Coordinates left_coverage_bottom_right;
    if (left) {
        left->out = center;
        left_coverage_top_left = left->coverage_top_left;
        left_coverage_bottom_right = left->coverage_bottom_right;
    }

    center->in_second = right;
    Coordinates right_coverage_top_left;
    Coordinates right_coverage_bottom_right;
    if (right) {
        right->out = center;
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

std::shared_ptr<Node>
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
                int& value)
{

    if (depth < 0) {
        return nullptr;
    }
    std::shared_ptr<Node> up = create_horizontal(
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
        value);

    std::shared_ptr<Node> down = create_horizontal(
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
        value);

    u_int32_t out_bandwidth_value = 0;
    u_int32_t in_bandwidth_value = 0;

    out_bandwidth_value = up->out_bandwidth * 2;
    in_bandwidth_value = up->out_bandwidth;

    std::shared_ptr<Node> center = std::make_shared<Node>(
        value, current_col, current_row, in_bandwidth_value, out_bandwidth_value);
    value++;

    center->in_first = up;
    Coordinates up_coverage_top_left;
    Coordinates up_coverage_bottom_right;
    if (up) {
        up->out = center;
        up_coverage_top_left = up->coverage_top_left;
        up_coverage_bottom_right = up->coverage_bottom_right;
    }

    center->in_second = down;
    Coordinates down_coverage_top_left;
    Coordinates down_coverage_bottom_right;
    if (down) {
        down->out = center;
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

std::shared_ptr<Node>
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
    int value = 0;
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

    std::shared_ptr<Node> left =
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
                        value);

    int right_sub_htree_initial_row = chip_center_x;
    int right_sub_htree_initial_col = chip_center_y + (chip_center_y / 2);

    std::shared_ptr<Node> right =
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
                        value);

    u_int32_t out_bandwidth_value = 0;
    u_int32_t in_bandwidth_value = 0;
    if (right) {
        out_bandwidth_value = right->out_bandwidth * 2;
        in_bandwidth_value = right->out_bandwidth;
    }

    // Join left and right
    std::shared_ptr<Node> center = std::make_shared<Node>(
        value, chip_center_y, chip_center_x, in_bandwidth_value, out_bandwidth_value);
    value++;

    center->in_first = left;
    if (left) {
        left->out = center;
    }

    center->in_second = right;
    if (right) {
        right->out = center;
    }

    center->out = nullptr;

    center->coverage_top_left = chip_coverage_top_left;
    center->coverage_bottom_right = chip_coverage_bottom_right;

    return center;
}

// Print the Htree in this order: top-left, down-left, top-right, down-right
void
print_htree(std::shared_ptr<Node>& root)
{
    if (root != nullptr) {
        print_htree(root->in_first);
        print_htree(root->in_second);

        cout << root;
    }
}
// Print the Htree in this order: top-left, down-left, top-right, down-right
void
populate_coorodinates_to_ptr_map(std::map<Coordinates, std::shared_ptr<Node>>& htree_end_nodes,
                                 std::shared_ptr<Node>& root)
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

    std::shared_ptr<Node> root = nullptr;

    // Insert nodes into the binary tree
    root = create_htree(hx, hy, depth, all_possible_rows, all_possible_cols);

    cout << "root value = " << root->id << "\n";

    // Print the tree using traversal
    cout << "Traversal:\n";
    print_htree(root);
    cout << endl;

    std::map<Coordinates, std::shared_ptr<Node>> htree_end_nodes;
    populate_coorodinates_to_ptr_map(htree_end_nodes, root);

    cout << "Printing the map: " << endl;

    // Print the map elements
    for (const auto& entry : htree_end_nodes) {
        std::cout << entry.first << ": " << entry.second << std::endl;
    }

    return 0;
}
