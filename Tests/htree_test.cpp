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

    // Get from front FIFO
    T front() const { return underlying_queue.front(); }

    // Pop/Dequeue
    void pop() { underlying_queue.pop(); }

    // Return the current size of the queue
    u_int32_t size() const { return underlying_queue.size(); }

    // Return the max size of the queue
    u_int32_t queue_size_max() const { return this->size_max; }
};

// TODO: see if we keep this globally here or use the HtreeNode function?
bool
is_coordinate_in_a_particular_range(const Coordinates start,
                                    const Coordinates end,
                                    const Coordinates point)
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

    /*     bool put_operon_from_within_htree_node(const CoordinatedOperon operon,
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
        } */

    bool is_coordinate_in_my_range(const Coordinates point)
    {
        // Check if the point is within the range
        return ((point.first >= this->coverage_top_left.first) &&
                (point.first <= this->coverage_bottom_right.first) &&
                (point.second >= this->coverage_top_left.second) &&
                (point.second <= this->coverage_bottom_right.second));
    }

    bool is_end_htree_node() { return (!this->in_first && !this->in_second); }

    bool is_htree_node_active()
    {
        bool send_channels_active = false;
        bool recv_channels_active = false;
        for (int i = 0; i < 4; i++) {
            if (this->send_channel[i]) {
                if (this->send_channel[i].value()->size() != 0) {
                    send_channels_active = true;
                    break;
                }
            }
            if (this->recv_channel[i]) {
                if (this->recv_channel[i].value()->size() != 0) {
                    recv_channels_active = true;
                    break;
                }
            }
        }

        bool send_sink_active = false;
        bool recv_sink_active = false;
        if (this->is_end_htree_node()) {
            if (this->send_channel_to_sink_cell->size() != 0) {
                send_sink_active = true;
            }
            if (this->recv_channel_from_sink_cell->size() != 0) {
                recv_sink_active = true;
            }
        }

        return (send_channels_active || recv_channels_active || send_sink_active ||
                recv_sink_active);
    }

    void transfer(std::shared_ptr<FixedSizeQueue<CoordinatedOperon>> recv,
                  std::optional<std::shared_ptr<FixedSizeQueue<CoordinatedOperon>>> send,
                  CoordinatedOperon operon)
    {
        if (send == std::nullopt) {
            std::cerr << this->id << ": Bug! transfer: send_channel cannot be null\n";
            exit(0);
        }
        std::shared_ptr<FixedSizeQueue<CoordinatedOperon>> current_send_channel = send.value();

        if (!current_send_channel->push(operon)) {

            // Put this back since it was not sent in this cycle due to the send_channel
            // being full
            recv->push(operon);
            std::cout << this->id << ":\trecv->push(), recv->size(): " << recv->size()
                      << " current_send_channel.size = " << current_send_channel->size() << "\n";
        }
    }

    void transfer_send_to_recv(
        std::optional<std::shared_ptr<FixedSizeQueue<CoordinatedOperon>>> send,
        std::optional<std::shared_ptr<FixedSizeQueue<CoordinatedOperon>>> recv)
    {

        if (send == std::nullopt) {
            std::cerr << this->id << ": Bug! transfer_send_to_recv: send_channel cannot be null\n";
            exit(0);
        }
        if (recv == std::nullopt) {
            std::cerr << this->id << ": Bug! transfer_send_to_recv: recv_channel cannot be null\n";
            exit(0);
        }

        while (send.value()->size()) {

            CoordinatedOperon operon = send.value()->front();
            if (recv.value()->push(operon)) {
                send.value()->pop();
            } else {
                // recv is full
                break;
            }
        }
    }

    void shift_from_a_single_recv_channel_to_send_channels(
        std::shared_ptr<FixedSizeQueue<CoordinatedOperon>> recv,
        std::optional<std::shared_ptr<FixedSizeQueue<CoordinatedOperon>>> send[])
    {

        // Pop all operons into a vector to avoid deadlock on a send_channel if it is full.
        // In case a send_channel is full then just push back that operon into the recv channel
        std::vector<CoordinatedOperon> recv_operons;
        while (recv->size()) {
            recv_operons.push_back(recv->front());
            recv->pop();
        }

        for (CoordinatedOperon operon : recv_operons) {

            Coordinates destination_cc_coorinates = operon.first;

            // Find route.

            // First check if this is the end htree node
            if (this->is_end_htree_node()) {
                // Does the route needs to go thought the sink channel?
                if (this->is_coordinate_in_my_range(destination_cc_coorinates)) {
                    /*  std::cout << this->id << ":\tSend " << destination_cc_coorinates
                               << " to sink cell\n"; */

                    Operon simple_operon = operon.second;
                    if (!this->send_channel_to_sink_cell->push(simple_operon)) {

                        // Put this back since it was not sent in this cycle due to the send_channel
                        // being full
                        recv->push(operon);
                        std::cout << this->id << ":\trecv->push(), recv->size(): " << recv->size()
                                  << "\n";
                    }

                } else {

                    transfer(recv, send[this->local_index_send_channel_out], operon);
                    /*     std::cout << this->id << ":\tSend " << destination_cc_coorinates
                                  << " to out | this->local_index_send_channel_out = "
                                  << this->local_index_send_channel_out << "\n"; */
                }
                // Check if it can go to `in_first`?
            } else if (is_coordinate_in_a_particular_range(this->in_first->coverage_top_left,
                                                           this->in_first->coverage_bottom_right,
                                                           destination_cc_coorinates)) {

                // Send to in_first

                transfer(recv, send[this->local_index_send_channel_in_first], operon);

            } else if (is_coordinate_in_a_particular_range(this->in_second->coverage_top_left,
                                                           this->in_second->coverage_bottom_right,
                                                           destination_cc_coorinates)) {

                // Send to in_second
                transfer(recv, send[this->local_index_send_channel_in_second], operon);
            } else {

                // Send to out
                transfer(recv, send[this->local_index_send_channel_out], operon);
            }
        }
    }

    void prepare_communication_cycle()
    {
        //  std::cout << this->id << ": in prepare_communication_cycle()\n";
        if (!this->is_htree_node_active()) {
            return;
        }
        // std::cout << this->id << ":\tStarting prepare_communication_cycle()\n";
        // Shift from recv_channel queues to send_channel queues

        // First recv from sink cell
        if (this->is_end_htree_node()) {
            shift_from_a_single_recv_channel_to_send_channels(this->recv_channel_from_sink_cell,
                                                              this->send_channel);
        }

        // Then from in and out recv channels
        u_int32_t recv_channel_index = this->current_recv_channel_to_start_a_cycle;
        for (int i = 0; i < 4; i++) {

            if (this->recv_channel[recv_channel_index] != std::nullopt) {
                shift_from_a_single_recv_channel_to_send_channels(
                    this->recv_channel[recv_channel_index].value(), this->send_channel);
            }

            recv_channel_index = (recv_channel_index + 1) % 4;
        }

        this->current_recv_channel_to_start_a_cycle =
            (this->current_recv_channel_to_start_a_cycle + 1) % 4;

        // std::cout << this->id << ":\tleaving prepare_communication_cycle()\n";
    }

    void run_a_communication_cylce()
    {
        //   std::cout << this->id << ": in run_a_communication_cylce\n";
        if (!this->is_htree_node_active()) {
            return;
        }
        // std::cout << this->id << ":\t starting run_a_communication_cylce\n";
        // Send from `send_*` queues to remote `recv_*` queues

        // First send from end node to sink cell
        if (this->is_end_htree_node()) {

            while (this->send_channel_to_sink_cell->size()) {
                Operon operon = this->send_channel_to_sink_cell->front();
                std::cout << this->id << ": Operon with cc id: " << operon.first
                          << " will be sent to sink cell depending on its recv queue\n";
                // TODO: implements tihs connection to the CCA chip later
                // use a vector as is being used in
                // shift_from_a_single_recv_channel_to_send_channels to avoid deadlock
                this->send_channel_to_sink_cell->pop();
            }
        }

        if (!this->is_end_htree_node()) { // End nodes dont have in_first and in_second
            // Then send from in_first
            transfer_send_to_recv(
                this->send_channel[this->local_index_send_channel_in_first],
                this->in_first->recv_channel[this->remote_index_recv_channel_in_first]);

            // Then send from in_second
            transfer_send_to_recv(
                this->send_channel[this->local_index_send_channel_in_second],
                this->in_second->recv_channel[this->remote_index_recv_channel_in_second]);
        }

        // Then send from out
        transfer_send_to_recv(this->send_channel[this->local_index_send_channel_out],
                              this->out->recv_channel[this->remote_index_recv_channel_out]);

        //  std::cout << this->id << ":\t leaving run_a_communication_cylce\n";
    }

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
    u_int32_t remote_index_recv_channel_out;

    u_int32_t remote_index_recv_channel_in_first;
    u_int32_t remote_index_recv_channel_in_second;

    u_int32_t local_index_send_channel_out;

    u_int32_t local_index_send_channel_in_first;
    u_int32_t local_index_send_channel_in_second;

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

    // This is used to be fair in routing. When we start a communication cycle from the same
    // recv_channel what will happen is that that channel will get priority in sending its operons
    // at expense of thoer channels/neighbors. We want to start every cycle with a different
    // starting recv_channel and then alternate between them. This will provide fairness and not
    // cause congestion at any one link.
    u_int32_t current_recv_channel_to_start_a_cycle{};

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
        }

        for (int i = 0; i < 4; i++) {
            this->recv_channel[i] = std::nullopt;
            this->send_channel[i] = std::nullopt;
        }

        // Start from 0th and then alternate by % 4
        this->current_recv_channel_to_start_a_cycle = 0;
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
        // TODO: Later instead of hardcoding `4` use the shape of the sink cell to determine
        // the value. Or some other initial thickness of the link
        out_bandwidth_value = 4;
        in_bandwidth_value = 0;
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

    // For the center its `in_second` is right, therefore from the `in_second` point of view it is
    // 0. `center` will put in recv_channel[0] of right
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

    // For the center its `in_second` is down, therefore from the `in_second` point of view it is 1.
    // `center` will put in recv_channel[1] of down
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

    // TODO: delete this as center will be delted and left and right parts of the base Htree will be
    // combined together
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
    /*
        Coordinates cc(2, 3);
        std::cout << "Where to route " << cc << "?\n";

        // First check if this is the end htree node
        if (htree_all_nodes[id]->is_end_htree_node()) {
            // Does the route needs to go thought the sink channel?
            if (htree_all_nodes[id]->is_coordinate_in_my_range(cc)) {
                std::cout << "\tSend to sink cell\n";
            } else {
                std::cout << "\tSend to out\n";
            }
            // Check if it can go to `in_first`?
        } else if (is_coordinate_in_a_particular_range(
                       htree_all_nodes[id]->in_first->coverage_top_left,
                       htree_all_nodes[id]->in_first->coverage_bottom_right,
                       cc)) {

            std::cout << "\tSend to in_first\n";
        } else if (is_coordinate_in_a_particular_range(
                       htree_all_nodes[id]->in_second->coverage_top_left,
                       htree_all_nodes[id]->in_second->coverage_bottom_right,
                       cc)) {
            std::cout << "\tSend to in_second\n";
        } else {
            std::cout << "\tSend to out\n";
        }
        */
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

    /*
    cout << "Printing the vector of all htree nodes: " << endl;
    // Print the vector elements
    for (const auto& htree_node : htree_all_nodes) {
        std::cout << htree_node;
    }
    std::cout << std::endl;
    */

    print_details_of_an_htree_node(htree_all_nodes, 0);

    // print_details_of_an_htree_node(htree_all_nodes, 1);

    /*
    print_details_of_an_htree_node(htree_all_nodes, 2);

        print_details_of_an_htree_node(htree_all_nodes, 6);

        print_details_of_an_htree_node(htree_all_nodes, 14);

        print_details_of_an_htree_node(htree_all_nodes, 29);

        print_details_of_an_htree_node(htree_all_nodes, 30); 
        */

    // Merge two halves of the Htree and remove the root
    std::shared_ptr<HtreeNode> root_in_first_temp = root->in_first;
    root_in_first_temp->out = root->in_second;
    root->in_second->out = root->in_first;

    print_details_of_an_htree_node(htree_all_nodes, root_in_first_temp->id);

    /*     print_details_of_an_htree_node(htree_all_nodes, 14);
        print_details_of_an_htree_node(htree_all_nodes, 29); */

    std::cout << std::endl;
    cout << "Testing Routing: " << endl;
    std::cout << std::endl;

    // Insert seed Operon from sink cell to an end node cell
    Coordinates cc(60, 21); // Final destination CC
    Operon operon(101, 42);
    CoordinatedOperon seed_operon(cc, operon);

    htree_all_nodes[0]->recv_channel_from_sink_cell->push(seed_operon);
    htree_all_nodes[0]->recv_channel_from_sink_cell->push(seed_operon);
    htree_all_nodes[1]->recv_channel_from_sink_cell->push(seed_operon);
    htree_all_nodes[3]->recv_channel_from_sink_cell->push(seed_operon);

    htree_all_nodes[4]->recv_channel_from_sink_cell->push(seed_operon);
    htree_all_nodes[4]->recv_channel_from_sink_cell->push(seed_operon);

    htree_all_nodes[7]->recv_channel_from_sink_cell->push(seed_operon);
    htree_all_nodes[7]->recv_channel_from_sink_cell->push(seed_operon);
    htree_all_nodes[7]->recv_channel_from_sink_cell->push(seed_operon);
    htree_all_nodes[7]->recv_channel_from_sink_cell->push(seed_operon);

    htree_all_nodes[26]->recv_channel_from_sink_cell->push(seed_operon);

    Coordinates cc_zero(0, 0); // Final destination CC
    Operon operon_zero(400, 42);
    CoordinatedOperon seed_operon_zero(cc_zero, operon_zero);
    htree_all_nodes[26]->recv_channel_from_sink_cell->push(seed_operon_zero);

    // Run simulation
    u_int32_t total_cycles = 0;
    bool global_active_htree = true;

    while (global_active_htree) {
        global_active_htree = false;

        std::cout << "prepare_communication_cycle # " << total_cycles << "\n";
        for (int i = 0; i < htree_all_nodes.size(); i++) {
            htree_all_nodes[i]->prepare_communication_cycle();
        }

        std::cout << "run_a_communication_cylce # " << total_cycles << "\n";
        for (int i = 0; i < htree_all_nodes.size(); i++) {
            htree_all_nodes[i]->run_a_communication_cylce();
        }

        // std::cout << "is_htree_node_active # " << total_cycles << "\n";
        for (int i = 0; i < htree_all_nodes.size(); i++) {
            global_active_htree |= htree_all_nodes[i]->is_htree_node_active();
        }
        total_cycles++;
        std::cout << std::endl;
    }

    return 0;
}
