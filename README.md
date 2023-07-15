# CCA-Simulator
Event-level simulator for [Continuum Computer Architecture (CCA)](https://superfri.org/index.php/superfri/article/view/188) class of designs.

## Summary
The CCA Simulator enables exploring design space of the CCA class of non-Von Neumann intelligent memory systems. These systems are concieved to be highly fine-grain parallel and use event-driven mechanisms to perform computation.

## Graph Applications
The [Applications](/Applications/) directory contains asynchronous message-driven applications written using the CCASimulator. Please browser through each application for build and run instructions.

## Using clang-tidy
Compile with: `CC=gcc-13 CXX=g++-13 cmake -S . -B build -D THROTTLE=true -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`

Inside an application use something like:
> `$ noglob /opt/homebrew/opt/llvm/bin/clang-tidy -p build ../../Source/ComputeCell.cpp -checks=cppcoreguidelines-* -header-filter=.*`

> `$ noglob python3 /opt/homebrew/opt/llvm/bin/run-clang-tidy -p build -header-filter='.*' -checks='-*,modernize-use-nullptr'`
