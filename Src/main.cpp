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

#include "Action.hpp"
#include "Address.hpp"
#include "Operon.hpp"
#include "Task.hpp"
#include "Enums.hpp"

#include "Memory_Management.hpp"

#include <iostream>
#include <stdlib.h>

#include <map>
#include <queue>

#include <chrono>

#include <omp.h>

// TODO: find a good way to do this, perhaps but this in a separate file
// Compile with: -DDEBUG_CODE=true
// Or maybe remove this altogether.
inline constexpr bool debug_code = DEBUG_CODE;

class SSSPAction : public Action
{
  public:
    SSSPAction(const Address vertex_addr_in,
               actionType type,
               const bool ready,
               const int nargs_in,
               const std::shared_ptr<int[]>& args_in,
               eventId predicate_in,
               eventId work_in,
               eventId diffuse_in)
    {
        // std::cout << "sssp action constructor\n";

        // this->vertex_addr = std::make_shared<Address>(vertex_addr);
        this->vertex_addr = vertex_addr_in;

        this->action_type = type;
        this->is_ready = ready;

        this->nargs = nargs_in;
        this->args = args_in;

        this->predicate = predicate_in;
        this->work = work_in;
        this->diffuse = diffuse_in;
    }
};

inline constexpr u_int32_t edges_max = 2;
struct SimpleVertex
{
    u_int32_t id;
    Address edges[edges_max];
    u_int32_t number_of_edges;
    u_int32_t sssp_distance;
};

void
print_SimpleVertex(u_int32_t obj_addr, const std::unique_ptr<char[]>& memory)
{
    SimpleVertex* vertex = (SimpleVertex*)(memory.get() + obj_addr);
    std::cout << vertex->id << "\n";

    for (int i = 0; i < vertex->number_of_edges; i++) {
        std::cout << vertex->edges[i] << "\n";
    }
    std::cout << std::endl;
}

enum class computeCellShape : u_int32_t
{
    block_1D = 0,
    triangular,
    sqaure,
    hexagon,
    computeCellShape_count
};

std::map<computeCellShape, u_int32_t> computeCellShape_num_channels = {
    { computeCellShape::block_1D, 2 },
    { computeCellShape::triangular, 3 },
    { computeCellShape::sqaure, 4 },
    { computeCellShape::hexagon, 6 }
};

// Note this class is not thread-safe, which is ok as we don't intend to use multithreading.
class ComputeCell
{
  public:
    void* get_object(Address addr_in) { return (this->memory.get() + addr_in.addr); }

    // TODO: remove this later
    void print_SimpleVertex(const Address& vertex_addr)
    {
        if (vertex_addr.cc_id != this->id) {
            std::cout << "Invalid addr! The vertex does not exist on this CC\n";
            return;
        }

        SimpleVertex* vertex = (SimpleVertex*)this->get_object(
            vertex_addr); // (SimpleVertex*)(this->memory.get() + vertex_addr.addr);
        std::cout << "Vertex ID: " << vertex->id << "\n";

        for (int i = 0; i < vertex->number_of_edges; i++) {
            std::cout << vertex->edges[i] << ", ";
        }
        std::cout << std::endl;
    }

    // Return the memory used in bytes
    u_int32_t get_memory_used() { return this->memory_curr_ptr - this->memory_raw_ptr; }

    // In bytes
    u_int32_t get_memory_curr_ptr_offset() { return get_memory_used(); }

    // Get memory left in bytes
    u_int32_t memory_available_in_bytes() { return this->memory_size_in_bytes - get_memory_used(); }

    // Returns the offset in memory for this newly created object
    template<typename T>
    std::optional<Address> create_object_in_memory(T obj)
    {
        if (this->memory_available_in_bytes() < sizeof(T)) {
            return std::nullopt;
        }

        u_int32_t obj_memory_addr_offset = get_memory_curr_ptr_offset();
        memcpy(this->memory_curr_ptr, &obj, sizeof(T));
        this->memory_curr_ptr = this->memory_curr_ptr + sizeof(T);

        return Address(this->id, obj_memory_addr_offset);
    }

    void insert_action(const std::shared_ptr<Action>& action) { this->action_queue.push(action); }

    void execute_action();

    // Execute a single cycle for this Compute Cell
    // Return whether this compute cell is still active, meaning run_a_cycle needs to be called
    // again
    bool run_a_cycle();

    // Checks if the compute cell is active or not
    // TODO: when communication is added then update checks for the communication buffer too
    bool is_compute_cell_active()
    {
        return (!this->action_queue.empty() || !this->task_queue.empty());
    }

    void add_neighbor(u_int32_t neighbor_compute_cell_id)
    {
        this->neighbor_compute_cells.push_back(neighbor_compute_cell_id);
    }

    // Identity of the Compute Cell
    u_int32_t id;

    // Communication of the Compute Cell

    // number_of_neighbors is the maximum number of connections a single Compute Cell can
    // architecturally have. A triangular CC has 3 neighbors, a square has 4, and hexagon has 6 etc.
    // TODO: Later read the shape from a config file and initialize it in the constructor
    u_int32_t number_of_neighbors;

    // IDs of the neighbors
    std::vector<u_int32_t> neighbor_compute_cells;
    // Per neighbor operon recieve queue.
    std::vector<std::queue<Operon>> channel_buffer_per_neighbor;

    // This is needed to satisty simulation. Because a sending CC can not just enqueue an operon
    // into the current working buffer of a neighbor CC. If it does then the neighbor may start
    // computation (or work) on that operon in the current simulation cycle. The receiving neighbor
    // must process it in the next cycle. Therefore, this send/recv buffer serves that purpose. At
    // the end of each simulation cycle each CC will move an operon from this buffer to its working
    // set of operons and will then execute those operons in the next cycle. This move is not part
    // of the computation but is only there for simulation so as not to break the
    // semantics/pragmatics of CCA.
    std::vector<Operon> send_recv_channel_buffer_per_neighbor;

    // Memory of the Compute Cell in bytes
    static constexpr u_int32_t memory_size_in_bytes = 2 * 1024 * 1024; // 2 MB
    std::unique_ptr<char[]> memory;
    char* memory_raw_ptr;
    char* memory_curr_ptr;

    // Actions queue of the Compute Cell
    std::queue<std::shared_ptr<Action>> action_queue;

    // TODO: maybe later make a function like this that gets from the queue in an intelligent matter
    // or depending on the policy. So it can be both FIFO and LIFO, maybe even better
    // std::shared_ptr<Action> get_an_action();

    // Tasks for the Compute Cell. These tasks exist only for this simulator and are not part of the
    // actual internals of any Compute Cell.
    // TODO:
    // std::queue<std::shared_ptr<Task>> task_queue;
    // TaskQueue task_queue;
    std::queue<Task> task_queue;

    // Constructor
    ComputeCell(u_int32_t id_in, computeCellShape shape)
    {
        /* cout << "Inside constructor of ComputeCell\n"; */

        this->id = id_in;
        this->number_of_neighbors = computeCellShape_num_channels[shape];
        // std::cout << "this->number_of_neighbors = " << this->number_of_neighbors << "\n";

        this->memory = std::make_unique<char[]>(this->memory_size_in_bytes);
        this->memory_raw_ptr = memory.get();
        this->memory_curr_ptr = memory_raw_ptr;
    }

    /* ~ComputeCell() { cout << "Inside destructor of ComputeCell\n"; } */
};

int
sssp_predicate(ComputeCell& cc, const Address& addr, int nargs, const std::shared_ptr<int[]>& args)
{
    // std::cout << "in sssp_predicate" << std::endl;
    return 0;
}
int
sssp_work(ComputeCell& cc, const Address& addr, int nargs, const std::shared_ptr<int[]>& args)
{
    // std::cout << "in sssp_work" << std::endl;
    int x = args[0];
    int y = args[1];

    int result = x + y;
    return 0;
}

int
sssp_diffuse(ComputeCell& cc, const Address& addr, int nargs, const std::shared_ptr<int[]>& args)
{
    // std::cout << "in sssp_diffuse: " << std::endl;
    //  std::cout << "(" << addr.cc_id << ", " << addr.addr << ")" << std::endl;
    // std::cout << addr  << std::endl;

    // cc.print_SimpleVertex(addr);
    SimpleVertex* v = static_cast<SimpleVertex*>(cc.get_object(addr));
    for (int i = 0; i < v->number_of_edges; i++) {
        std::string message = "Send from ";
        message += std::to_string(v->id);
        message += " --> (";
        message +=
            std::to_string(v->edges[i].cc_id) + ", " + std::to_string(v->edges[i].addr) + ")\n";
        cc.task_queue.push(send_operon(message));
    }
    // cc.task_queue.push([](std::string message) { send_operon(message); });
    return 0;
}

// TODO: Maybe later convert these too `std::function`
//       With perhaps a std::map of the functions
//       that way we can add new functions at runtime (if needed)
typedef int (*handler_func)(ComputeCell& cc,
                            const Address& addr,
                            int nargs,
                            const std::shared_ptr<int[]>& args);

handler_func event_handlers[] = { sssp_predicate, sssp_work, sssp_diffuse };

void
ComputeCell::execute_action()
{

    if (!this->action_queue.empty()) {
        std::shared_ptr<Action> action = this->action_queue.front();
        this->action_queue.pop();

        if constexpr (debug_code == true) {
            this->print_SimpleVertex(action->vertex_addr);
        }

        // TODO: actually put the ifs

        // if predicate
        event_handlers[get_underlying_enum_index(action->predicate)](
            *this, action->vertex_addr, action->nargs, action->args);

        // if work
        event_handlers[get_underlying_enum_index(action->work)](
            *this, action->vertex_addr, action->nargs, action->args);

        // if diffuse
        event_handlers[get_underlying_enum_index(action->diffuse)](
            *this, action->vertex_addr, action->nargs, action->args);
        return;
    }
    std::cout << "Cannot execute action as the action_queue is empty!\n";
}

bool
ComputeCell::run_a_cycle()
{

    // A single compute cell can perform work and communication in parallel in a single cycle
    // This function does both. First it performs work if there is any. Then it performs
    // communication

    // Perform execution of work
    // Exectute a task if the task_queue is not empty
    if (!this->task_queue.empty()) {
        // std::cout << "run_a_cycle | task | CC : " << this->id << "\n";
        //  Get a task from the task_queue
        Task current_task = this->task_queue.front();
        this->task_queue.pop();

        // Execute the task
        current_task("The MeSSaGe fRoM 2oo8");
    } else if (!this->action_queue
                    .empty()) { // Else execute an action if the action_queue is not empty

        // std::cout << "run_a_cycle | action | CC : " << this->id << "\n";

        this->execute_action();
    }

    // Perform communication

    // House Keeping: Copy communication operator from neighbor to the current communication
    // buffer of this CC

    // Return the active status of this CC and later it can be used to update the global active
    // compute cells count
    return this->is_compute_cell_active();
}

constexpr u_int32_t total_compute_cells = 3;
constexpr u_int32_t total_vertices = 4;

// Note: We could have put this function as part of the ComputeCell class but then it would have
// introduced application specific functionality into the compute cell. Perhaps, if we really want
// to introduce some special `insert_edge` instruction then we can rethink this. In anycase it makes
// no difference on the simulation. This is just a software engineering decision.
inline bool
insert_edge_by_address(std::vector<std::shared_ptr<ComputeCell>>& CCA_chip,
                       Address src_vertex_addr,
                       Address dst_vertex_addr)
{
    SimpleVertex* vertex =
        static_cast<SimpleVertex*>(CCA_chip[src_vertex_addr.cc_id]->get_object(src_vertex_addr));

    // Check if edges are not full
    // TODO: Later implement the hierarical parallel vertex object
    if (vertex->number_of_edges >= edges_max)
        return false;

    vertex->edges[vertex->number_of_edges] = dst_vertex_addr;
    vertex->number_of_edges++;

    return true;
}

inline bool
insert_edge_by_vertex_id(std::vector<std::shared_ptr<ComputeCell>>& CCA_chip,
                         u_int32_t src_vertex_id,
                         u_int32_t dst_vertex_id)
{

    std::cout << "Inserting " << src_vertex_id << " --> " << dst_vertex_id << "\n";
    Address src_vertex_addr = get_vertex_address_cyclic(
        src_vertex_id, total_vertices, sizeof(SimpleVertex), total_compute_cells);

    Address dst_vertex_addr = get_vertex_address_cyclic(
        dst_vertex_id, total_vertices, sizeof(SimpleVertex), total_compute_cells);

    return insert_edge_by_address(CCA_chip, src_vertex_addr, dst_vertex_addr);
}

int
main()
{
    // Create an Action queue and a *memory location
    // Populate the memory somehow
    // Insert actions in the queue that operate on objects in memory

    std::vector<std::shared_ptr<ComputeCell>> CCA_chip;

    std::cout << "Populating the CCA Chip: \n";
    // Cannot simply openmp parallelize this. It is very atomic.
    for (int i = 0; i < total_compute_cells; i++) {

        CCA_chip.push_back(std::make_shared<ComputeCell>(i, computeCellShape::block_1D));

        u_int32_t right_neighbor = (i == total_compute_cells - 1) ? 0 : i + 1;
        CCA_chip.back()->add_neighbor(right_neighbor);
        u_int32_t left_neighbor = (i == 0) ? total_compute_cells - 1 : i - 1;
        CCA_chip.back()->add_neighbor(left_neighbor);
    }

    std::cout << "Allocating vertices cyclically on the CCA Chip: \n";

    for (int i = 0; i < total_vertices; i++) {

        // put a vertex in memory
        SimpleVertex vertex_;
        vertex_.id = i;
        vertex_.number_of_edges = 0;

        // randomly populate the edges
        /*    for (int k = 0; k < edges_max; k++) {
               vertex_.edges[k] = 4;
               vertex_.number_of_edges++;
           }
    */
        Address vertex_addr_cyclic = get_vertex_address_cyclic(
            vertex_.id, total_vertices, sizeof(SimpleVertex), total_compute_cells);

        u_int32_t cc_id = vertex_addr_cyclic.cc_id;

        std::optional<Address> vertex_addr =
            CCA_chip[cc_id]->create_object_in_memory<SimpleVertex>(vertex_);

        if (!vertex_addr) {
            std::cout << "Memory not allocated! Vertex ID: " << vertex_.id << "\n";
            continue;
        }

        /* std::cout << "vertex_.id = " << vertex_.id
                  << ", CCA_chip[cc_id]->id = " << CCA_chip[cc_id]->id
                  << ", vertex_addr = " << vertex_addr.value() << ", get_vertex_address = " <<
        vertex_addr_cyclic
                  << "\n"; */

        std::shared_ptr<int[]> args_x = std::make_shared<int[]>(2);
        args_x[0] = 1;
        args_x[1] = 7;

        CCA_chip[cc_id]->insert_action(std::make_shared<SSSPAction>(vertex_addr.value(),
                                                                    actionType::application_action,
                                                                    true,
                                                                    2,
                                                                    args_x,
                                                                    eventId::sssp_predicate,
                                                                    eventId::sssp_work,
                                                                    eventId::sssp_diffuse));
    }

    std::cout << "Populating vertices by inserting edges: \n";
    for (int i = 0; i < total_vertices; i++) {

        if (!insert_edge_by_vertex_id(CCA_chip, i, i + 1)) {
            std::cout << "Error! Edge not inserted successfully.\n";
        }
    }

    bool global_active_cc = true;
    u_long total_cycles = 0;
    std::cout << "Starting Execution on the CCA Chip: \n";
    auto start = std::chrono::steady_clock::now();

    while (global_active_cc) {
        global_active_cc = false;
        // for (auto& cc : CCA_chip) {
#pragma omp parallel for reduction(| : global_active_cc)
        for (int i = 0; i < CCA_chip.size(); i++) {
            // std::cout << "Running CC : " << cc->id << "\n\n";
            if (CCA_chip[i]->is_compute_cell_active()) {
                global_active_cc |= CCA_chip[i]->run_a_cycle();
            }
        }
        total_cycles++;
    }
    auto end = std::chrono::steady_clock::now();

    std::cout << "Total Cycles: " << total_cycles << "\n";
    std::cout << "Elapsed time in nanoseconds: "
              << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() << " ns"
              << std::endl;

    std::cout << "Elapsed time in microseconds: "
              << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << " µs"
              << std::endl;

    std::cout << "Elapsed time in milliseconds: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms"
              << std::endl;

    std::cout << "Elapsed time in seconds: "
              << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() << " sec\n";
    /*   args_x = nullptr;
      std::cout << "in main(): args_x.use_count() == " << args_x.use_count() << " (object @ "
                << args_x << ")\n"; */

    return 0;
}
