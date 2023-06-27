

#include "Action.hpp"
#include "HtreeNetwork.hpp"
#include "HtreeNode.hpp"

#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <queue>
#include <vector>

using namespace std;

class DummyAction : public Action
{
  public:
    DummyAction() {}

    ~DummyAction() override {}
};

int
main(int argc, char* argv[])
{
    // CAUTION: The project has moved on. This test case is a bit depreciated. Before using it
    // please update it comparing the current state of the HtreeNetwork.

    if (argc < 4) {
        std::cout << "Usage: ./Htree_Creator.out <hx> <hy> <depth>" << std::endl;
        return 1;
    }

    u_int32_t hx = std::atoi(argv[1]);
    u_int32_t hy = std::atoi(argv[2]);
    u_int32_t depth = std::atoi(argv[3]);
    u_int32_t max_bandwidth = std::atoi(argv[4]);

    std::cout << "hx: " << hx << " hy: " << hy << " depth: " << depth
              << " max_bandwidth: " << max_bandwidth << std::endl;

    HtreeNetwork test_htree_network(hx, hy, depth, max_bandwidth);

    std::cout << std::endl;
    cout << "Testing Routing: " << endl;
    std::cout << std::endl;

    // Insert seed Operon from sink cell to an end node cell
    Coordinates cc(61, 21); // Final destination CC
    Operon operon(SourceDestinationPair(0, 6000), DummyAction());
    CoordinatedOperon seed_operon(cc, operon);

    test_htree_network.htree_all_nodes[0]->recv_channel_from_sink_cell->push(seed_operon);
    test_htree_network.htree_all_nodes[0]->recv_channel_from_sink_cell->push(seed_operon);
    test_htree_network.htree_all_nodes[1]->recv_channel_from_sink_cell->push(seed_operon);
    test_htree_network.htree_all_nodes[3]->recv_channel_from_sink_cell->push(seed_operon);

    test_htree_network.htree_all_nodes[4]->recv_channel_from_sink_cell->push(seed_operon);
    test_htree_network.htree_all_nodes[4]->recv_channel_from_sink_cell->push(seed_operon);

    test_htree_network.htree_all_nodes[7]->recv_channel_from_sink_cell->push(seed_operon);
    test_htree_network.htree_all_nodes[7]->recv_channel_from_sink_cell->push(seed_operon);
    test_htree_network.htree_all_nodes[7]->recv_channel_from_sink_cell->push(seed_operon);
    test_htree_network.htree_all_nodes[7]->recv_channel_from_sink_cell->push(seed_operon);

    test_htree_network.htree_all_nodes[26]->recv_channel_from_sink_cell->push(seed_operon);

    /*     Coordinates cc_zero(0, 0); // Final destination CC
        Operon operon_zero(SourceDestinationPair(2, 10), DummyAction());
        CoordinatedOperon seed_operon_zero(cc_zero, operon_zero);

        test_htree_network.htree_all_nodes[26]->recv_channel_from_sink_cell->push(seed_operon_zero);

        test_htree_network.htree_end_nodes[Coordinates(49, 58)]->recv_channel_from_sink_cell->push(
            seed_operon_zero);
        test_htree_network.htree_end_nodes[Coordinates(60, 31)]->recv_channel_from_sink_cell->push(
            seed_operon_zero);
        test_htree_network.htree_end_nodes[Coordinates(71, 31)]->recv_channel_from_sink_cell->push(
            seed_operon_zero); */

    // Run simulation
    u_int32_t total_cycles = 0;
    bool global_active_htree = true;

    while (global_active_htree) {
        global_active_htree = false;

        std::cout << "prepare_communication_cycle # " << total_cycles << "\n";
        for (int i = 0; i < test_htree_network.htree_all_nodes.size(); i++) {
            test_htree_network.htree_all_nodes[i]->prepare_communication_cycle();
        }

        std::cout << "run_a_communication_cylce # " << total_cycles << "\n";
        for (int i = 0; i < test_htree_network.htree_all_nodes.size(); i++) {
            test_htree_network.htree_all_nodes[i]->run_a_communication_cylce();
        }

        // std::cout << "is_htree_node_active # " << total_cycles << "\n";
        for (int i = 0; i < test_htree_network.htree_all_nodes.size(); i++) {
            global_active_htree |= test_htree_network.htree_all_nodes[i]->is_htree_node_active();
        }
        total_cycles++;
        std::cout << std::endl;
    }

    return 0;
}
