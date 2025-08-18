# AI & Robotics Glossary

An active glossary of terms, concepts, and acronyms used throughout my AI and robotics projects.

---

## A

### Ament
-   **Definition**: The build system used by ROS 2, which is an evolution of the `catkin` build system from ROS 1. It provides the tools and logic for processing packages. `ament_cmake` is a specific type of Ament package that uses CMake for its build logic, making it ideal for C++ and mixed-language projects.
-   **Context/Significance**: We used the `ament_cmake` build type for our `core_robotics_package` because it provides the flexibility to build both our C++ subscriber and Python publisher within the same package, a common practice in professional robotics.

---

## C

### Colcon
-   **Stands For**: **Co**llective **Con**struction
-   **Definition**: The standard command-line build tool for ROS 2. It's used to compile and build workspaces containing one or more ROS 2 packages, managing dependencies and generating the necessary setup files.
-   **Context/Significance**: We use `colcon build` to turn our Python and C++ source code into executable nodes that ROS 2 can run. Mastering colcon is essential for managing any non-trivial robotics project.

---

## D

### DDS
-   **Stands For**: Data Distribution Service
-   **Definition**: An industry-standard, peer-to-peer communication protocol for real-time systems, managed by the Object Management Group (OMG).
-   **Context/Significance**: DDS is the underlying middleware that makes ROS 2's communication system work. It handles the automatic discovery of nodes, the transmission of messages over the network, and the quality-of-service settings, enabling the robust, distributed environment we set up in Phase 1.

---

## G

### Git
-   **Definition**: A distributed version control system used for tracking changes in source code during software development. It allows developers to maintain a history of changes, create separate branches for new features, and collaborate effectively.
-   **Context/Significance**: Git is the fundamental tool we use to save our work, document changes via commits, and manage the history of each project in this portfolio. Proficiency in Git is a non-negotiable skill for any software engineering role.

### GitHub
-   **Definition**: A web-based platform that provides hosting for software development and version control using Git. It also includes project management, collaboration, and social coding features.
-   **Context/Significance**: GitHub is the public "showroom" for this portfolio. It's where we publish our Git repositories, making our work visible to recruiters and the broader engineering community.

---

## I

### Isaac Sim
-   **Definition**: A photorealistic, physics-accurate virtual environment and robotics simulation platform developed by NVIDIA. It leverages the NVIDIA Omniverse platform for high-fidelity rendering and simulation.
-   **Context/Significance**: Isaac Sim is our "digital twin" environment. It allows us to safely and rapidly develop, test, and validate our robotics algorithms on virtual robots before deploying them to physical hardware, which is a core tenet of modern robotics development.

---

## J

### Jetson (Orin Nano)
-   **Definition**: A series of embedded computing boards from NVIDIA. The Jetson Orin Nano is a compact, power-efficient AI computer designed to run modern AI workloads on edge devices like robots and drones.
-   **Context/Significance**: The Jetson is our "physical target." It's the hardware where we will deploy our "sim-to-real" projects, proving that our algorithms can run on a real, resource-constrained robotic brain.

### JetPack SDK
-   **Definition**: The comprehensive software development kit for NVIDIA Jetson modules. It bundles the Linux operating system (L4T), CUDA-X accelerated libraries, and other tools for developing and deploying AI software.
-   **Context/Significance**: We used the JetPack SDK to flash the operating system and all necessary NVIDIA drivers onto our Jetson Orin Nano, preparing it for robotics and AI tasks.

---

## N

### Node
-   **Definition**: The fundamental unit of computation in ROS 2. A node is a process that performs a specific task, such as controlling a motor, reading sensor data, or planning a path. Nodes communicate with each other by sending and receiving messages via topics, services, or actions.
-   **Context/Significance**: In Phase 2, we created two nodes: a Python publisher node (`simple_publisher`) and a C++ subscriber node (`simple_subscriber`), demonstrating the core building block of a ROS 2 system.

---

## P

### Package
-   **Definition**: The primary unit for organizing software in ROS 2. A package is a directory that contains everything related to a specific functionality, including its source code (nodes), launch files, configuration files, and a `package.xml` manifest file that defines its properties and dependencies.
-   **Context/Significance**: We created the `core_robotics_package` to contain our publisher and subscriber nodes, learning the standard structure for organizing and distributing robotics software.

---

## R

### rclcpp / rclpy
-   **Stands For**: ROS Client Library for C++ / ROS Client Library for Python
-   **Definition**: These are the official client libraries that provide the high-level APIs for interacting with the ROS 2 system in their respective languages. They provide the tools to create nodes, publishers, subscribers, services, and more.
-   **Context/Significance**: We used `rclpy` to write our publisher node and `rclcpp` to write our subscriber node, demonstrating proficiency in the two most common and important ROS 2 client libraries.

### ROS (ROS 2)
-   **Stands For**: Robot Operating System
-   **Definition**: A flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.
-   **Context/Significance**: ROS is the central nervous system for all the projects in this portfolio. It provides the standardized communication layer that allows our separate nodes (e.g., perception, planning, control) to work together as a cohesive system.

---

## S

### Sim-to-Real
-   **Definition**: A workflow in robotics where a policy or algorithm is first trained and tested in a simulation environment (like Isaac Sim) and then transferred to a physical robot (like a Jetson-powered device).
-   **Context/Significance**: The sim-to-real transfer is a critical challenge and a highly sought-after skill in modern robotics. The success of this portfolio hinges on demonstrating a successful sim-to-real pipeline.

### SME
-   **Stands For**: Subject Matter Expert
-   **Definition**: An individual who exhibits the highest level of expertise in a specialized job, task, or domain.
-   **Context/Significance**: The overarching goal of this entire portfolio is to build and demonstrate the skills, knowledge, and professional practices of an SME in the field of AI and Robotics.

---

## T

### Topic
-   **Definition**: A named bus over which ROS 2 nodes exchange messages. Topics have a many-to-many communication relationship: any number of nodes can publish (send) messages to a topic, and any number of nodes can subscribe (receive) messages from that same topic.
-   **Context/Significance**: Our nodes used the `/chatter` topic to communicate. The publisher sent messages to `/chatter`, and the subscriber received them from `/chatter`, demonstrating the most common communication pattern in ROS 2.

---

## V

### VRAM
-   **Stands For**: Video Random Access Memory
-   **Definition**: Specialized RAM used to store image and graphics data for a computer's display. In the context of AI and simulation, it's the dedicated memory on the GPU.
-   **Context/Significance**: VRAM is a critical resource for running Isaac Sim and training large AI models. The amount of available VRAM on a GPU determines the complexity of the simulations and models that can be run.

---

## W

### Workspace
-   **Definition**: A directory containing one or more ROS 2 packages, along with `build`, `install`, and `log` subdirectories created by `colcon`. A workspace allows you to develop and build multiple related packages simultaneously.
-   **Context/Significance**: We created the `ros2-ws` workspace to house our `core_robotics_package`. Sourcing the `install/setup.bash` file from this workspace makes our custom package's executables available in our terminal.