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

#include "ComputeCell.hpp"
#include "SimpleVertex.hpp"

Task
send_operon(std::string message)
{
    return Task([message](std::string xx) {
        std::cout << "Executed second task! message: " << message << "\n";
    });
}

// TODO: move this to application
int
sssp_predicate(ComputeCell& cc, const Address& addr, int nargs, const std::shared_ptr<int[]>& args)
{
    // std::cout << "in sssp_predicate" << std::endl;
    return 0;
}

// TODO: move this to application
int
sssp_work(ComputeCell& cc, const Address& addr, int nargs, const std::shared_ptr<int[]>& args)
{
    // std::cout << "in sssp_work" << std::endl;
    int x = args[0];
    int y = args[1];

    int result = x + y;
    return 0;
}

// TODO: move this to application
int
sssp_diffuse(ComputeCell& cc, const Address& addr, int nargs, const std::shared_ptr<int[]>& args)
{
    // std::cout << "in sssp_diffuse: " << std::endl;
    // std::cout << "(" << addr.cc_id << ", " << addr.addr << ")" << std::endl;
    // std::cout << addr  << std::endl;

    //cc.print_SimpleVertex(addr);
    SimpleVertex<Address>* v = static_cast<SimpleVertex<Address>*>(cc.get_object(addr));
    for (int i = 0; i < v->number_of_edges; i++) {
        std::string message = "Send from ";
        message += std::to_string(v->id) + " --> (" + std::to_string(v->edges[i].cc_id) + ", " +
                   std::to_string(v->edges[i].addr) + ")\n";

        cc.task_queue.push(send_operon(message));
    }

    return 0;
}

// TODO: Maybe later convert these too `std::function`
//       With perhaps a std::map of the functions
//       that way we can add new functions at runtime (if needed)
typedef int (*handler_func)(ComputeCell& cc,
                            const Address& addr,
                            int nargs,
                            const std::shared_ptr<int[]>& args);

// TODO: This really needs to be a map or something so as to no make it constant and be able to
// extend it
inline static handler_func event_handlers[] = { sssp_predicate, sssp_work, sssp_diffuse };

// TODO: move this to application
void
print_SimpleVertex(const ComputeCell& cc, const Address& vertex_addr)
{
    if (vertex_addr.cc_id != cc.id) {
        std::cout << "Invalid addr! The vertex does not exist on this CC\n";
        return;
    }

    SimpleVertex<Address>* vertex = (SimpleVertex<Address>*)cc.get_object(
        vertex_addr); // (SimpleVertex*)(this->memory.get() + vertex_addr.addr);
    std::cout << "Vertex ID: " << vertex->id << "\n";

    for (int i = 0; i < vertex->number_of_edges; i++) {
        std::cout << vertex->edges[i] << ", ";
    }
    std::cout << std::endl;
}

// Get the object memory location at address addr_in
void*
ComputeCell::get_object(Address addr_in) const
{
    return (this->memory.get() + addr_in.addr);
}

// Return the memory used in bytes
u_int32_t
ComputeCell::get_memory_used()
{
    return this->memory_curr_ptr - this->memory_raw_ptr;
}

// In bytes
u_int32_t
ComputeCell::get_memory_curr_ptr_offset()
{
    return get_memory_used();
}

// Get memory left in bytes
u_int32_t
ComputeCell::memory_available_in_bytes()
{
    return this->memory_size_in_bytes - get_memory_used();
}

// Returns the offset in memory for this newly created object
std::optional<Address>
ComputeCell::create_object_in_memory(void* obj, size_t size_of_obj)
{
    if (this->memory_available_in_bytes() < size_of_obj) {
        return std::nullopt;
    }

    u_int32_t obj_memory_addr_offset = get_memory_curr_ptr_offset();
    memcpy(this->memory_curr_ptr, obj, size_of_obj);
    this->memory_curr_ptr += size_of_obj;

    return Address(this->id, obj_memory_addr_offset);
}

void
ComputeCell::insert_action(const std::shared_ptr<Action>& action)
{
    this->action_queue.push(action);
}

// Checks if the compute cell is active or not
// TODO: when communication is added then update checks for the communication buffer too
bool
ComputeCell::is_compute_cell_active()
{
    return (!this->action_queue.empty() || !this->task_queue.empty());
}

void
ComputeCell::add_neighbor(
    std::pair<u_int32_t, std::pair<u_int32_t, u_int32_t>> neighbor_compute_cell)
{
    this->neighbor_compute_cells.push_back(neighbor_compute_cell);
}
void
ComputeCell::execute_action()
{

    if (!this->action_queue.empty()) {
        std::shared_ptr<Action> action = this->action_queue.front();
        this->action_queue.pop();

        if constexpr (debug_code == true) {
            print_SimpleVertex(*this, action->obj_addr);
        }

        // TODO: actually put the ifs

        // if predicate
        event_handlers[get_underlying_enum_index(action->predicate)](
            *this, action->obj_addr, action->nargs, action->args);

        // if work
        event_handlers[get_underlying_enum_index(action->work)](
            *this, action->obj_addr, action->nargs, action->args);

        // if diffuse
        event_handlers[get_underlying_enum_index(action->diffuse)](
            *this, action->obj_addr, action->nargs, action->args);
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

std::string
ComputeCell::get_compute_cell_shape_name(computeCellShape shape)
{
    switch (shape) {
        case (computeCellShape::block_1D):
            return std::string("block_1D");
            break;
        case (computeCellShape::triangular):
            return std::string("triangular");
            break;
        case (computeCellShape::square):
            return std::string("square");
            break;
        case (computeCellShape::hexagon):
            return std::string("hexagon");
            break;

        default:
            return std::string("Invalid Shape");
            break;
    }
}

computeCellShape
ComputeCell::get_compute_cell_shape_enum(std::string shape)
{
    if (shape == "block_1D") {
        return computeCellShape::block_1D;
    } else if (shape == "triangular") {
        return computeCellShape::triangular;
    } else if (shape == "sqaure") {
        return computeCellShape::square;
    } else if (shape == "hexagon") {
        return computeCellShape::hexagon;
    } else {
        return computeCellShape::computeCellShape_invalid;
    }
}

u_int32_t
ComputeCell::get_number_of_neighbors(computeCellShape shape_in)
{
    switch (shape_in) {
        case (computeCellShape::block_1D):
            return 2;
            break;
        case (computeCellShape::triangular):
            return 3;
            break;
        case (computeCellShape::square):
            return 4;
            break;
        case (computeCellShape::hexagon):
            return 6;
            break;

        default:
            return 0;
            break;
    }
}
