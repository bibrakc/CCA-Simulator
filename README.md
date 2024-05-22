# CCA-Simulator
Simulator for Continuum Computer Architecture (CCA) ([paper1](https://superfri.org/index.php/superfri/article/view/188), and [paper2](https://arxiv.org/abs/2402.02576)) class of designs.

## Summary
The CCA Simulator enables exploring design space of the CCA class of non-Von Neumann intelligent memory systems. These systems are concieved to be highly fine-grain parallel and use event-driven mechanisms to perform computation.

It can be used to design and deploy asynchronous message-driven computations and understand the runtime behavior of AM-CCA configurations. The simulator is high-level enough to be programmed using the diffusive programming model, see our [paper3](https://arxiv.org/abs/2402.06086), and yet low-level enough to simulate individual operon movements between CCs. In a single simulation cycle, a message can traverse one hop from one CC to a neighboring CC. We make this assumption since AM-CCA channel links are $256$ bit wide and can easily send the small operons (messages) of our tested applications in a single flit cycle. Simultaneously, a single CC, can perform either of the two operations: 
1. a computing instruction, which is contained in the predicate resolution and work in the user application action, or 
2. the creation and staging of a new operon.

It means that BFS and SSSP actions take $2$-$3$ cycles of compute, whereas Page Rank action takes anywhere from $3$-$70$ cycles of compute. When their diffusions are executed they in turn take cycles proportional to the amount of \textit{local edge-list size}.

## Demo of Message-Driven Execution
<img src="Analytics/Animations/BFS_32x32_v_1024_e_10240_th_ON_SH_ON.gif" alt="Animation" width="800"/>

<img src="Analytics/Animations/Legend_Animation.png" alt="Legend" width="800"/>

## Graph Applications
The [Applications](/Applications/) directory contains asynchronous message-driven applications written using the CCASimulator. Please browser through each application for build and run instructions.
