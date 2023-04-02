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

class Address
{
  public:
    // Global ID of the compute cell where the address resides
    int cc_id;
    // The offset to the memory of the compute cell
    int addr;

    Address(int id, int address_in)
    {
        this->cc_id = id;
        this->addr = address_in;
    }
    friend ostream& operator<<(ostream& os, const Address& ad)
    {
        os << "cc_id: " << ad.cc_id << "addr: " << ad.addr;
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
    // int x = static_cast<int>(static_cast<int *>(args)[0]);
    int x = args[0];
    // int y = static_cast<int>(static_cast<int *>(args)[1]);
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
// int mult(int x, int y) { return (x * y); }
int
div1(int x, int y)
{
    if (y != 0)
        return (x / y);
    else
        return 0;
}

enum class actionType : int
{
    internal_action = 0,
    application_action,
    actionType_max
};

// TODO: Maybe make this an abstract class with virtual functions
//       and then use something like:
//       `std::queue<std::make_shared/unique<Action>> ActionsQueue`
//       Need to investigate this further.
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
    // TODO: maybe use the class `Address for` this?
    void* obj;

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
    SSSPAction(actionType type,
               bool ready,
               int nargs_in,
               const std::shared_ptr<int[]>& args_in,
               eventId predicate_in,
               eventId work_in,
               eventId diffuse_in)
    {
        this->action_type = type;
        this->is_ready = ready;

        this->nargs = nargs_in;
        this->args = args_in;

        this->predicate = predicate_in;
        this->work = work_in;
        this->diffuse = diffuse_in;

        std::cout << "sssp action constructor\n";
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
    // Return the memory used in bytes
    u_int32_t get_memory_used() { return this->memory_curr_ptr - this->memory_raw_ptr; }

    // In bytes
    u_int32_t get_memory_curr_ptr_offset() { return get_memory_used(); }

    // Returns the offset in memory for this newly created object
    template<typename T>
    u_int32_t create_object_in_memory(T obj)
    {
        u_int32_t obj_memory_addr_offset = get_memory_curr_ptr_offset();
        this->memory_curr_ptr = memcpy(this->memory_curr_ptr, &obj, sizeof(T));
        this->memory_curr_ptr = this->memory_curr_ptr + sizeof(T);

        return obj_memory_addr_offset;
    }

    void insert_action(const std::shared_ptr<Action>& action) { this->action_queue.push(action); }

    void execute_action()
    {
        if (!this->action_queue.empty()) {
            std::shared_ptr<Action> action = this->action_queue.back();
            this->action_queue.pop();
            std::cout << event_handlers[get_underlying_enum_index(action->predicate)](action->nargs,
                                                                                      action->args)
                      << std::endl;
            return;
        }
        std::cout << "Cannot execute action as the action_queue is empty!\n";
    }

    // Communication of the Compute Cell
    u_int32_t id;
    std::vector<u_int32_t> neighbor_compute_cells;

    // Memory of the Compute Cell
    static constexpr u_int32_t memory_size = 2 * 1024 * 1024; // 2 MB
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
    /*

    // std::cout << valInt << std::endl;
    // int *args = static_cast<int *>(malloc(2 * sizeof(int)));
    std::shared_ptr<int[]> args = std::make_shared<int[]>(2);
    args[0] = 1;
    args[1] = 2;

    SSSPAction sssp_action = SSSPAction(actionType::application_action, true, 2,
    args, eventId::sssp_predicate, eventId::sssp_work, eventId::sssp_diffuse);
    int valInt = get_underlying_enum_index(sssp_action.predicate);

    // eventId sum_id = ;
    std::cout << event_handlers[valInt](sssp_action.nargs, sssp_action.args) <<
    std::endl;

    */

    // Create an Action queue and a *memory location
    // Populate the memory somehow
    // Insert actions in the queue that operate on objects in memory

    std::queue<std::shared_ptr<Action>> actionQueue;

    constexpr u_int32_t memory_size = 2 * 1024 * 1024; // 2 MB
    std::unique_ptr<char[]> memory = std::make_unique<char[]>(memory_size);

    char* memory_raw_ptr = memory.get();
    char* memory_curr_ptr = memory_raw_ptr;

    // put a vertex in memory
    SimpleVertex vertex_root;
    vertex_root.id = 5;
    vertex_root.edges[0] = 1;
    vertex_root.edges[1] = 11;
    vertex_root.edges[2] = 111;
    vertex_root.edges[3] = 1111;
    vertex_root.edges[4] = 11111;
    vertex_root.edges[5] = 111111;

    u_int32_t root_vertex_addr = memory_curr_ptr - memory_raw_ptr;

    memcpy(memory_curr_ptr, &vertex_root, sizeof(vertex_root));
    memory_curr_ptr = memory_curr_ptr + sizeof(vertex_root);

    std::cout << "in main(): (root_vertex_addr= " << root_vertex_addr << "), memory_raw_ptr = ("
              << (int*)(memory_raw_ptr) << "), memory_curr_ptr = (" << (int*)(memory_curr_ptr)
              << "\n";

    // char *v =  root_vertex_addr + static_cast<char*>(memory_raw_ptr);

    print_SimpleVertex(root_vertex_addr, memory);

    std::shared_ptr<int[]> args_x = std::make_shared<int[]>(2);
    args_x[0] = 1;
    args_x[1] = 7;

    actionQueue.push(std::make_shared<SSSPAction>(actionType::application_action,
                                                  true,
                                                  2,
                                                  args_x,
                                                  eventId::sssp_predicate,
                                                  eventId::sssp_work,
                                                  eventId::sssp_diffuse));

    std::shared_ptr<Action> x = actionQueue.back();
    actionQueue.pop();
    std::cout << event_handlers[get_underlying_enum_index(x->predicate)](x->nargs, x->args)
              << std::endl;

    x = nullptr;
    args_x = nullptr;
    std::cout << "in main(): args_x.use_count() == " << args_x.use_count() << " (object @ "
              << args_x << ")\n";

    return 0;
}
