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
#include <queue>

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
        std::cout << "in Copy constructor Address" << std::endl;
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

// TODO: Maybe later convert these too `std::function`
//       With perhaps a std::map of the functions
//       that way we can add new functions
typedef int (*handler_func)(int nargs, const std::shared_ptr<int[]>& args);

enum class eventId : int
{
    sssp_predicate = 0,
    sssp_work,
    sssp_diffuse,
    eventId_max
};

int
sum(int nargs, void* args);
int
sub(int nargs, void* args);

int
sssp_predicate(int nargs, const std::shared_ptr<int[]>& args)
{
    std::cout << "in sssp_predicate" << std::endl;
    int x = args[0];
    int y = args[1];

    int result = x + y;
    return result;
}
int
sssp_work(int nargs, const std::shared_ptr<int[]>& args)
{
    std::cout << "in sssp_work" << std::endl;
    return 0;
}
int
sssp_diffuse(int nargs, const std::shared_ptr<int[]>& args)
{
    std::cout << "in sssp_diffuse" << std::endl;
    return 0;
}
// int mult(int num1, int num2);
// int div1(int num1, int num2);

handler_func event_handlers[] = { sssp_predicate, sssp_work, sssp_diffuse };

int
sum(int nargs, void* args)
{
    int x = static_cast<int>(static_cast<int*>(args)[0]);
    int y = static_cast<int>(static_cast<int*>(args)[1]);
    free(args);
    int result = x + y;

    return result;
}
int
sub(int nargs, void* args)
{
    int x = static_cast<int>(static_cast<int*>(args)[0]);
    int y = static_cast<int>(static_cast<int*>(args)[1]);
    free(args);
    int result = x - y;

    return result;
}

enum class actionType : int
{
    internal_action = 0,
    application_action,
    actionType_max
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
    ~Action() { std::cout << "In Action destructor" << std::endl; }
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
        std::cout << "sssp action constructor\n";

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

void
fun(std::shared_ptr<int> sp)
{
    std::cout << "in fun(): sp.use_count() == " << sp.use_count() << " (object @ " << sp << ")\n";
}

struct SimpleVertex
{
    u_int32_t id;
    u_int32_t edges[6];

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

// Note this class is not thread-safe, which is ok as we don't intend to use multithreading.
class ComputeCell
{
  public:
    // TODO: remove this later
    void print_SimpleVertex(const Address& vertex_addr)
    {
        if (vertex_addr.cc_id != this->id) {
            std::cout << "Invalid addr the vertex does not exist on this CC\n";
            return;
        }

        SimpleVertex* vertex = (SimpleVertex*)(this->memory.get() + vertex_addr.addr);
        std::cout << vertex->id << "\n";

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
    u_int32_t memory_available_in_bytes() { return this->memory_size - get_memory_used(); }

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

    void execute_action()
    {

        if (!this->action_queue.empty()) {
            std::shared_ptr<Action> action = this->action_queue.back();
            this->action_queue.pop();

            if constexpr (debug_code == true) {
                // The `*` before action dereferences the `shared_ptr action->vertex_addr`
                this->print_SimpleVertex(*action->vertex_addr);
            }
            // if predicate
            std::cout << event_handlers[get_underlying_enum_index(action->predicate)](action->nargs,
                                                                                      action->args)
                      << std::endl;

            // if work

            // if diffuse
            return;
        }
        std::cout << "Cannot execute action as the action_queue is empty!\n";
    }

    // Communication of the Compute Cell
    u_int32_t id;
    std::vector<u_int32_t> neighbor_compute_cells;

    // Memory of the Compute Cell
    static constexpr u_int32_t memory_size = 200; // * 1024 * 1024; // 2 MB
    std::unique_ptr<char[]> memory;
    char* memory_raw_ptr;
    char* memory_curr_ptr;

    // Actions queue of the Compute Cell
    std::queue<std::shared_ptr<Action>> action_queue;

    // Tasks for the Compute Cell. These tasks exist only for this simulator and are not part of the
    // actual internals of any Compute Cell.
    // TODO:
    // std::queue<std::shared_ptr<Task>> task_queue;

    // Constructor
    ComputeCell()
    {
        cout << "Inside constructor of ComputeCell\n";
        this->memory = std::make_unique<char[]>(this->memory_size);

        this->memory_raw_ptr = memory.get();
        this->memory_curr_ptr = memory_raw_ptr;
    }

    ~ComputeCell() { cout << "Inside destructor of ComputeCell\n"; }
};

int
main()
{
    // Create an Action queue and a *memory location
    // Populate the memory somehow
    // Insert actions in the queue that operate on objects in memory

    ComputeCell cc;

    // char *v =  root_vertex_addr + static_cast<char*>(memory_raw_ptr);

    // print_SimpleVertex(root_vertex_addr, memory);

    for (int i = 0; i < 2; i++) {

        // put a vertex in memory
        SimpleVertex vertex_root;
        vertex_root.id = 5;
        vertex_root.edges[0] = i;
        vertex_root.edges[1] = i * 10 + vertex_root.edges[0];
        vertex_root.edges[2] = i * 100 + vertex_root.edges[1];
        vertex_root.edges[3] = i * 1000 + vertex_root.edges[2];
        vertex_root.edges[4] = i * 10000 + vertex_root.edges[3];
        vertex_root.edges[5] = i * 100000 + vertex_root.edges[4];
        ;

        std::optional<Address> vertex_root_addr =
            cc.create_object_in_memory<SimpleVertex>(vertex_root);

        if (!vertex_root_addr) {
            std::cout << "Memory not declared!\n";
            continue;
        }

        std::cout << "vertex_root_addr = " << vertex_root_addr.value() << "\n";

        std::shared_ptr<int[]> args_x = std::make_shared<int[]>(2);
        args_x[0] = 1;
        args_x[1] = 7;

        cc.insert_action(std::make_shared<SSSPAction>(vertex_root_addr.value(),
                                                      actionType::application_action,
                                                      true,
                                                      2,
                                                      args_x,
                                                      eventId::sssp_predicate,
                                                      eventId::sssp_work,
                                                      eventId::sssp_diffuse));

        cc.execute_action();
    }

    /*   args_x = nullptr;
      std::cout << "in main(): args_x.use_count() == " << args_x.use_count() << " (object @ "
                << args_x << ")\n"; */

    return 0;
}
