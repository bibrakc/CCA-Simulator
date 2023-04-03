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

#include <stdlib.h>

#include <iostream>
#include <map>
#include <queue>

#include <chrono>

#include <omp.h>

using namespace std;

// TODO: find a good way to do this, perhaps but this in a separate file
// Compile with: -DDEBUG_CODE=true
// Or maybe remove this altogether.
inline constexpr bool debug_code = DEBUG_CODE;

class Address
{
  public:
    // Global ID of the compute cell where the address resides
    u_int32_t cc_id;
    // The offset to the memory of the compute cell
    u_int32_t addr;

    // Is true when this address is not pointing to any valid object
    bool is_valid;

    // Copy constructor
    Address(const Address& addr_in)
    {
        // std::cout << "in Copy constructor Address" << std::endl;
        this->cc_id = addr_in.cc_id;
        this->addr = addr_in.addr;
        this->is_valid = addr_in.is_valid;
    }

    Address(int id, int address_in, bool valid)
    {
        this->cc_id = id;
        this->addr = address_in;
        this->is_valid = valid;
    }
    friend ostream& operator<<(ostream& os, const Address& ad)
    {
        os << "(" << ad.cc_id << ", " << ad.addr << ")";
        return os;
    }
};

template<typename E>
constexpr typename std::underlying_type<E>::type
get_underlying_enum_index(E e)
{
    return static_cast<typename std::underlying_type<E>::type>(e);
}

enum class eventId : u_int32_t
{
    sssp_predicate = 0,
    sssp_work,
    sssp_diffuse,
    eventId_count
};

enum class actionType : u_int32_t
{
    internal_action = 0,
    application_action,
    actionType_count
};

// TODO: Should this be virtual?
class Action
{
  public:
    // Type of the action: application type, internal runtime work action,
    // or any other
    actionType action_type;

    // Sets to `true` when all dependencies for this action are satisfied
    // and this action is ready to be executed
    bool is_ready;

    // Number of arguments to the action function
    int nargs;

    // Payload that contains the data like the arguments to the action function
    std::shared_ptr<int[]> args;

    // Memory location of the object for which this action is destined
    std::shared_ptr<Address> vertex_addr;

    // Predicate
    eventId predicate;

    // Work function that does some computation and may change the state
    // of the object for which this action is destined
    eventId work;

    // Generate actions along the edges for the diffusion
    eventId diffuse;

    // We can't just delete the args here since we don't know
    ~Action()
    { /* std::cout << "In Action destructor" << std::endl; */
    }
};

class SSSPAction : public Action
{
  public:
    SSSPAction(const Address vertex_addr,
               actionType type,
               const bool ready,
               const int nargs_in,
               const std::shared_ptr<int[]>& args_in,
               eventId predicate_in,
               eventId work_in,
               eventId diffuse_in)
    {
        // std::cout << "sssp action constructor\n";

        this->vertex_addr = std::make_shared<Address>(vertex_addr);

        this->action_type = type;
        this->is_ready = ready;

        this->nargs = nargs_in;
        this->args = args_in;

        this->predicate = predicate_in;
        this->work = work_in;
        this->diffuse = diffuse_in;
    }
};

typedef std::pair<u_int32_t, Action> Operon;

// TODO: remove this later or put in a util to be used for debuging
void
fun(std::shared_ptr<int> sp)
{
    std::cout << "in fun(): sp.use_count() == " << sp.use_count() << " (object @ " << sp << ")\n";
}

inline constexpr u_int32_t edges_max = 6;
struct SimpleVertex
{
    u_int32_t id;
    u_int32_t edges[edges_max];
    u_int32_t sssp_distance;
};

void
print_SimpleVertex(u_int32_t obj_addr, const std::unique_ptr<char[]>& memory)
{
    SimpleVertex* vertex = (SimpleVertex*)(memory.get() + obj_addr);
    std::cout << vertex->id << "\n";

    for (int i = 0; i < 6; i++) {
        std::cout << vertex->edges[i] << "\n";
    }
    std::cout << std::endl;
}

// Task and TaskQueue related
typedef std::function<void(std::string)> Task;

/* class TaskQueue
{
  public:
    void insert_task(Task task) { this->task_queue.push(task); }

    bool is_not_empty() { return !this->task_queue.empty(); }
    std::queue<Task> task_queue;
};
 */

enum class computeCellShape : u_int32_t
{
    triangular = 0,
    sqaure,
    hexagon,
    block_1D,
    computeCellShape_count
};

std::map<computeCellShape, u_int32_t> computeCellShape_num_channels = {
    { computeCellShape::triangular, 3 },
    { computeCellShape::sqaure, 4 }
};

// Note this class is not thread-safe, which is ok as we don't intend to use multithreading.
class ComputeCell
{
  public:
    // TODO: remove this later
    void print_SimpleVertex(const Address& vertex_addr)
    {
        if (vertex_addr.cc_id != this->id) {
            std::cout << "Invalid addr! The vertex does not exist on this CC\n";
            return;
        }

        SimpleVertex* vertex = (SimpleVertex*)(this->memory.get() + vertex_addr.addr);
        std::cout << "Vertex ID: " << vertex->id << "\n";

        for (int i = 0; i < 6; i++) {
            std::cout << vertex->edges[i] << "\n";
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

        return Address(this->id, obj_memory_addr_offset, true);
    }

    void insert_action(const std::shared_ptr<Action>& action) { this->action_queue.push(action); }

    void execute_action();

    // Execute a single cycle for this Compute Cell
    // Return whether this compute cell is still active, meaning run_a_cycle needs to be called again
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
    inline static u_int32_t number_of_neighbors =
        computeCellShape_num_channels[computeCellShape::triangular];

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
    ComputeCell(u_int32_t id_in)
    {
        /* cout << "Inside constructor of ComputeCell\n"; */

        this->id = id_in;

        this->memory = std::make_unique<char[]>(this->memory_size_in_bytes);
        this->memory_raw_ptr = memory.get();
        this->memory_curr_ptr = memory_raw_ptr;
    }

    /* ~ComputeCell() { cout << "Inside destructor of ComputeCell\n"; } */
};

int
sssp_predicate(ComputeCell& cc, int nargs, const std::shared_ptr<int[]>& args)
{
    // std::cout << "in sssp_predicate" << std::endl;
    return 0;
}
int
sssp_work(ComputeCell& cc, int nargs, const std::shared_ptr<int[]>& args)
{
    // std::cout << "in sssp_work" << std::endl;
    int x = args[0];
    int y = args[1];

    int result = x + y;
    return 0;
}
int
sssp_diffuse(ComputeCell& cc, int nargs, const std::shared_ptr<int[]>& args)
{
    // std::cout << "in sssp_diffuse" << std::endl;

    cc.task_queue.push(
        [](std::string
               message) { /* cout << "Executed first task! message: " << message << "\n"; */ });

    cc.task_queue.push(
        [](std::string
               message) { /* cout << "Executed second task! message: " << message << "\n"; */ });
    return 0;
}

// TODO: Maybe later convert these too `std::function`
//       With perhaps a std::map of the functions
//       that way we can add new functions
typedef int (*handler_func)(ComputeCell& cc, int nargs, const std::shared_ptr<int[]>& args);

handler_func event_handlers[] = { sssp_predicate, sssp_work, sssp_diffuse };

void
ComputeCell::execute_action()
{

    if (!this->action_queue.empty()) {
        std::shared_ptr<Action> action = this->action_queue.front();
        this->action_queue.pop();

        if constexpr (debug_code == true) {
            // The `*` before action dereferences the shared_ptr `action->vertex_addr`
            this->print_SimpleVertex(*action->vertex_addr);
        }
        // if predicate
        event_handlers[get_underlying_enum_index(action->predicate)](
            *this, action->nargs, action->args);

        // if work
        event_handlers[get_underlying_enum_index(action->work)](*this, action->nargs, action->args);

        // if diffuse
        event_handlers[get_underlying_enum_index(action->diffuse)](
            *this, action->nargs, action->args);
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

int
main()
{
    // Create an Action queue and a *memory location
    // Populate the memory somehow
    // Insert actions in the queue that operate on objects in memory

    std::vector<std::shared_ptr<ComputeCell>> CCA_chip;
    constexpr u_int32_t total_compute_cells = 10;

    // Cannot simply openmp parallelize this. It is very atomic.
    for (int i = 0; i < total_compute_cells; i++) {
        CCA_chip.push_back(std::make_shared<ComputeCell>(i));
    }

#pragma omp parallel for
    for (int cc_id = 0; cc_id < CCA_chip.size(); cc_id++) {

        // for (auto& cc : CCA_chip) {
        //  std::cout << "Populating CC : " << cc->id << "\n\n";
        for (int i = 0; i < 2; i++) {

            // put a vertex in memory
            SimpleVertex vertex_root;
            vertex_root.id = (CCA_chip[cc_id]->id * 2) + i;

            vertex_root.edges[0] = vertex_root.id;
            vertex_root.edges[1] = vertex_root.id * 10 + vertex_root.edges[0];
            vertex_root.edges[2] = vertex_root.id * 100 + vertex_root.edges[1];
            vertex_root.edges[3] = vertex_root.id * 1000 + vertex_root.edges[2];
            vertex_root.edges[4] = vertex_root.id * 10000 + vertex_root.edges[3];
            vertex_root.edges[5] = vertex_root.id * 100000 + vertex_root.edges[4];

            std::optional<Address> vertex_root_addr =
                CCA_chip[cc_id]->create_object_in_memory<SimpleVertex>(vertex_root);

            if (!vertex_root_addr) {
                std::cout << "Memory not declared!\n";
                continue;
            }

            // std::cout << "vertex_root_addr = " << vertex_root_addr.value() << "\n";

            std::shared_ptr<int[]> args_x = std::make_shared<int[]>(2);
            args_x[0] = 1;
            args_x[1] = 7;

            CCA_chip[cc_id]->insert_action(
                std::make_shared<SSSPAction>(vertex_root_addr.value(),
                                             actionType::application_action,
                                             true,
                                             2,
                                             args_x,
                                             eventId::sssp_predicate,
                                             eventId::sssp_work,
                                             eventId::sssp_diffuse));

            // cc->execute_action();
        }
    }

    bool global_active_cc = true;
    u_long total_cycles = 0;
    std::cout << "Starting Execution on the CCA Chip: \n";
    auto start = chrono::steady_clock::now();

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
    auto end = chrono::steady_clock::now();

    std::cout << "Total Cycles: " << total_cycles << "\n";
    cout << "Elapsed time in nanoseconds: "
         << chrono::duration_cast<chrono::nanoseconds>(end - start).count() << " ns" << endl;

    cout << "Elapsed time in microseconds: "
         << chrono::duration_cast<chrono::microseconds>(end - start).count() << " µs" << endl;

    cout << "Elapsed time in milliseconds: "
         << chrono::duration_cast<chrono::milliseconds>(end - start).count() << " ms" << endl;

    cout << "Elapsed time in seconds: "
         << chrono::duration_cast<chrono::seconds>(end - start).count() << " sec\n";
    /*   args_x = nullptr;
      std::cout << "in main(): args_x.use_count() == " << args_x.use_count() << " (object @ "
                << args_x << ")\n"; */

    return 0;
}
