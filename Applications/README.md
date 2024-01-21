# Asynchronous Message-Driven Applications Using Diffusive Programming Model
Applications build using CCASimulator using the diffusive programming model

## Breadth_First_Search
The [Breadth_First_Search](/Applications/Breadth_First_Search/) implements asynchronous `breadth first search` using the CCASimulator.
## Iterative_Breadth_First_Search
The [Iterative_Breadth_First_Search](/Applications/Iterative_Breadth_First_Search/) application implements asynchronous `breadth first search` using the CCASimulator. In particular it uses Iterative Deepening to traverse the graph.
## Iterative_Single_Source_Shortest_Path
The [Iterative_Single_Source_Shortest_Path](/Applications/Iterative_Single_Source_Shortest_Path/) application implements asynchronous `single source shortest path` using the CCASimulator. In particular it uses Iterative Deepening to traverse the graph.
## Page_Rank_Fixed_Iterations
The [Page_Rank_Fixed_Iterations](/Applications/Page_Rank_Fixed_Iterations/) implements asynchronous `page rank` with fixed number of iterations using the CCASimulator. In this application there is no convergence implemented rather the program runs for a fixed number if page rank iterations provided by the user.
## Page_Rank_Nested_Fixed_Iterations
The [Page_Rank_Nested_Fixed_Iterations](/Applications/Page_Rank_Nested_Fixed_Iterations/) implements an asynchronous `page rank` with nested fixed number of iterations using the CCASimulator. In this application there is no convergence implemented rather the program runs for a fixed number if page rank iterations provided by the user. These host side iterations invoke the `nested page rank` where it asynchronously goes into the next iteration. Kind of an overlap.
## Single_Source_Shortest_Path
The [Single_Source_Shortest_Path](/Applications/Single_Source_Shortest_Path/) implements asynchronous `single source shortest path` using the CCASimulator.

# Notes

## Deprecated
Some of the above applications have been moved to the `Deprecated` folder due to constant internal development of the simulator they need to be modified and tested. It should not require much work other than fixing the predicate and diffuse functions. Previously there was only one queue in a compute cell, i.e. the `action_queue` but now since it has been split into `action_queue` and `diffuse_queue` the applications need to provide "diffuse predicate function".

## Using clang-tidy
Compile with: `CC=gcc-13 CXX=g++-13 cmake -S . -B build -D THROTTLE=true -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`

Inside an application use something like:
> `$ noglob /opt/homebrew/opt/llvm/bin/clang-tidy -p build ../../Source/ComputeCell.cpp -checks=cppcoreguidelines-* -header-filter=.*`

> `$ noglob python3 /opt/homebrew/opt/llvm/bin/run-clang-tidy -p build -header-filter='.*' -checks='-*,modernize-use-nullptr'`
