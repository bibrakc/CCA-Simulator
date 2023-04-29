#include <iostream>

using namespace std;

// H-Tree Node
struct Node
{
    int id;
    int cooridinate_x;
    int cooridinate_y;

    Node* in_first;  // up or left
    Node* in_second; // down or right
    Node* out;       // outside of this htree

    int in_bandwidth;
    int out_bandwidth;

    Node(int value, int x, int y, int in_bandhwidth_in, int out_bandhwidth_in)
    {
        id = value;
        cooridinate_x = x;
        cooridinate_y = y;

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
        os << "[" << node->id << " -> (" << node->cooridinate_x << ", " << node->cooridinate_y
           << ")], ";
    } else {
        os << node->id << ", ";
    }

    return os;
}

Node*
create_vertical(int initial_row,
                int initial_col,
                int hx_factor,
                int hy_factor,
                int up_alternate_minus_one,
                int left_alternate_minus_one,
                int down_alternate_plus_one,
                int right_alternate_plus_one,
                int current_row,
                int current_col,
                int depth,
                int& value);

Node*
create_horizontal(int initial_row,
                  int initial_col,
                  int hx_factor,
                  int hy_factor,
                  int up_alternate_minus_one,
                  int left_alternate_minus_one,
                  int down_alternate_plus_one,
                  int right_alternate_plus_one,
                  int current_row,
                  int current_col,
                  int depth,
                  int& value)
{

    if (depth < 0) {
        return nullptr;
    }
    Node* left = create_vertical(initial_row,
                                 initial_col,
                                 hx_factor,
                                 2 * hy_factor,
                                 up_alternate_minus_one,
                                 (left_alternate_minus_one + 1) % 2,
                                 down_alternate_plus_one,
                                 right_alternate_plus_one,
                                 current_row,
                                 current_col - (initial_col / hy_factor) - left_alternate_minus_one,
                                 depth - 1,
                                 value);

    Node* right =
        create_vertical(initial_row,
                        initial_col,
                        hx_factor,
                        2 * hy_factor,
                        up_alternate_minus_one,
                        left_alternate_minus_one,
                        down_alternate_plus_one,
                        (right_alternate_plus_one + 1) % 2,
                        current_row,
                        current_col + (initial_col / hy_factor) + 1, // right_alternate_plus_one,
                        depth - 1,
                        value);

    u_int32_t out_bandwidth_value = 0;
    u_int32_t in_bandwidth_value = 0;

    // This means that it is an end node
    if (depth == 0) {
        out_bandwidth_value = 4;
        in_bandwidth_value = 0;
    } else if (depth == 1) {
        out_bandwidth_value = 8;
        in_bandwidth_value = 4;
    } else {
        out_bandwidth_value = right->out_bandwidth * 2;
        in_bandwidth_value = right->out_bandwidth;
    }

    // Join left and right
    Node* center =
        new Node(value, current_col, current_row, in_bandwidth_value, out_bandwidth_value);
    value++;

    center->in_first = left;
    if (left) {
        left->out = center;
    }

    center->in_second = right;
    if (right) {
        right->out = center;
    }
    return center;
}

Node*
create_vertical(int initial_row,
                int initial_col,
                int hx_factor,
                int hy_factor,
                int up_alternate_minus_one,
                int left_alternate_minus_one,
                int down_alternate_plus_one,
                int right_alternate_plus_one,
                int current_row,
                int current_col,
                int depth,
                int& value)
{

    if (depth < 0) {
        return nullptr;
    }
    Node* up = create_horizontal(initial_row,
                                 initial_col,
                                 2 * hx_factor,
                                 hy_factor,
                                 (up_alternate_minus_one + 1) % 2,
                                 left_alternate_minus_one,
                                 down_alternate_plus_one,
                                 right_alternate_plus_one,
                                 current_row - (initial_row / hx_factor) - up_alternate_minus_one,
                                 current_col,
                                 depth,
                                 value);

    Node* down =
        create_horizontal(initial_row,
                          initial_col,
                          2 * hx_factor,
                          hy_factor,
                          up_alternate_minus_one,
                          left_alternate_minus_one,
                          (down_alternate_plus_one + 1) % 2,
                          right_alternate_plus_one,
                          current_row + (initial_row / hx_factor) + down_alternate_plus_one,
                          current_col,
                          depth,
                          value);

    u_int32_t out_bandwidth_value = 0;
    u_int32_t in_bandwidth_value = 0;

    /*     // This means that its neighbors are end nodes
        if (depth == 1) {
            // if (up->in_first == nullptr && up->in_second == nullptr) {
            out_bandwidth_value = 8;
            in_bandwidth_value = 4;
        } else { */
    out_bandwidth_value = up->out_bandwidth * 2;
    in_bandwidth_value = up->out_bandwidth;
    // }

    Node* center =
        new Node(value, current_col, current_row, in_bandwidth_value, out_bandwidth_value);
    value++;

    center->in_first = up;
    if (up) {
        up->out = center;
    }
    center->in_second = down;
    if (down) {
        down->out = center;
    }

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

Node*
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

    int chip_center_x = (dim_x / 2) - 1;
    int chip_center_y = (dim_y / 2) - 1;

    cout << "Chip dimenssions: " << dim_x << " x " << dim_y << "\n";
    cout << "Creating Htree with center at: (" << chip_center_x << ", " << chip_center_y << ")\n";

    int left_sub_htree_initial_row = chip_center_x;
    int left_sub_htree_initial_col = chip_center_y - (chip_center_y / 2) - 1;

    Node* left = create_vertical(chip_center_x,
                                 chip_center_y,
                                 2, // divisor factor for the next division in rows
                                 4, // divisor factor for the next division in cols
                                 0, // up_alternate_minus_one,
                                 0, // (left_alternate_minus_one + 1) % 2,
                                 0, // down_alternate_plus_one,
                                 0, // right_alternate_plus_one,
                                 left_sub_htree_initial_row,
                                 left_sub_htree_initial_col,
                                 depth - 1,
                                 value);

    int right_sub_htree_initial_row = chip_center_x;
    int right_sub_htree_initial_col = chip_center_y + (chip_center_y / 2) + 1;

    Node* right = create_vertical(chip_center_x,
                                  chip_center_y,
                                  2, // divisor factor for the next division in rows
                                  4, // divisor factor for the next division in cols
                                  0, // up_alternate_minus_one,
                                  0, // left_alternate_minus_one,
                                  0, // down_alternate_plus_one,
                                  0, // (right_alternate_plus_one + 1) % 2,
                                  right_sub_htree_initial_row,
                                  right_sub_htree_initial_col,
                                  depth - 1,
                                  value);

    u_int32_t out_bandwidth_value = 0;
    u_int32_t in_bandwidth_value = 0;
    if (right) {
        out_bandwidth_value = right->out_bandwidth * 2;
        in_bandwidth_value = right->out_bandwidth;
    }

    // Join left and right
    Node* center =
        new Node(value, chip_center_y, chip_center_x, in_bandwidth_value, out_bandwidth_value);
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

    return center;
}

// In-order traversal of the binary tree
void
Traversal(Node* root)
{
    if (root != nullptr) {
        Traversal(root->in_first);

        Traversal(root->in_second);

        cout << root;
    }
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

    return closest;
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

    int total_rows_cols_with_htree_nodes = static_cast<int>(pow(2, 3));

    int first_row = std::ceil(hx / 2.0);
    int first_col = std::ceil(hy / 2.0);

    int range_rows = total_rows_cols_with_htree_nodes * hx;
    int range_cols = total_rows_cols_with_htree_nodes * hy;
    std::cout << "range_rows: " << range_rows << ", first_row: " << first_row
              << "range_cols: " << range_cols << ", first_col: " << first_col << "\n";

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

    Node* root = nullptr;

    // Insert nodes into the binary tree
    root = create_htree(hx, hy, depth, all_possible_rows, all_possible_cols);

    cout << "root value = " << root->id << "\n";

    // Print the tree using traversal
    cout << "Traversal: ";
    Traversal(root);
    cout << endl;

    return 0;
}
