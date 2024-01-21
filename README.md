# CCA-Simulator
Event-level simulator for [Continuum Computer Architecture (CCA)](https://superfri.org/index.php/superfri/article/view/188) class of designs.

## Summary
The CCA Simulator enables exploring design space of the CCA class of non-Von Neumann intelligent memory systems. These systems are concieved to be highly fine-grain parallel and use event-driven mechanisms to perform computation.

It can be used to design and deploy asynchronous message-driven computations and understand the runtime behavior of AM-CCA configurations. The simulator is high-level enough to be programmed using the diffusive programming model and yet low-level enough to simulate individual operon movements between CCs. In a single simulation cycle, an operon can traverse one hop from one CC to a neighboring CC. We make this assumption since AM-CCA channel links are $256$ bit wide and can easily send the small operons of our tested applications in a single flit cycle. Simultaneously, a single CC, can perform either of the two operations: 
1. a computing task, which is the predicate resolution and work in the user application action, or
2. the creation and staging of a new operon.

## Graph Applications
The [Applications](/Applications/) directory contains asynchronous message-driven applications written using the CCASimulator. Please browser through each application for build and run instructions.
