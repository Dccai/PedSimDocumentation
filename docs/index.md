# PedSim Documentation

This documentation provides an overview of the pedestrian simulation architecture, detailing how social forces are computed, how agent states are managed, and how the main simulation loop operates.

## Force Calculations Architecture

The simulation utilizes a hybrid approach for force calculations, featuring default C++ behaviors that can be dynamically overridden by Python-based models.

### C++ Default Forces (`ped_agent.cpp`)

The baseline functions to compute each social force are located in the `ped_agent.cpp` file. Each force is multiplied by a configured factor before being aggregated into a total force. The system uses this total force to calculate the new velocity and subsequent position of each pedestrian.

* **`desiredForce()`:** Pulls the pedestrian towards their current waypoint.
* **`socialForce()`:** Employs the Helbing social force model to repel the agent from other pedestrians.
* **`robotForce()`:** Influences pedestrian behavior based on robot proximity (disabled by default in the base class).
* **`myForce()`:** A placeholder for custom forces to be added to calculations (disabled by default in the base class).

While `ped_agent.cpp` handles the default velocity and position updates, the forces it relies on can be completely overridden by the Python integration.

### Python Force Overrides (`main.py`)

The Python force calculation system, located in `pedsim_agents\main.py`, allows you to swap out the default C++ forces for advanced behavioral models. The `main()` function initializes the specified force model, listens to the main robotics simulation for real-time input data, and feeds that data into a callback to compute new forces. When the `isForceOverridden` flag is set, these Python calculations bypass the C++ forces entirely.

The five available models are stored in the `pedsim_agents\pedsim_forces\forcemodels\` directory:

* **PySocialForce:** An advanced Helbing model incorporating group behavior, allowing agents to form and move in clusters.
* **Evacuation:** A physics-based model tailored for emergency scenarios to simulate panic and evacuation dynamics.
* **DeepSocialForce:** A machine-learning approach utilizing neural networks to predict agent forces.
* **Spinny:** A debugging and testing model that forces agents to rotate, providing an obvious visual confirmation that the Python override system is functioning.
* **Passthrough:** Passes the default C++ forces forward while applying a Python semantic overlay. This is highly useful for comparing base C++ behavior with Python models without losing semantic visual information (e.g., movement status, velocity components).

## Agent Implementation (`agent.cpp`)

The `agent.cpp` file contains the actual `Agent` class utilized by the simulator. It inherits from `Ped::Tagent` (found in `ped_agent.cpp`) and overrides several core behaviors to implement the active simulation logic. It also instantiates the state machine and provides helper functions used for state transitions.

* **`robotForce()` Override:** Implements inverse distance robot avoidance. It checks for robots within a 4.0-meter radius, calculates force magnitude using a `1.0/distance` formula, and returns a non-zero force.
* **`myForce()` Override:** Sums all registered custom `Force` objects (e.g., group coherence, group repulsion, random force, wall-following). Note that the default list of custom forces is initially empty.
* **`socialForce()` & `obstacleForce()` Overrides:** Adds runtime checks to dynamically disable specific forces based on the agent's current state. It also emits signals for RViz visualization.

## Agent State Machine (`agentstatemachine.cpp`)

Agent behaviors are driven by a state machine that evaluates conditions and handles transitions.

### Core State Functions

* **`doStateTransition()`:** Called every frame. It evaluates the current state, checks agent conditions via `agent.cpp` helper functions, and triggers `activateState()` if a change is required. It utilizes an early-return pattern to ensure only one state change occurs per frame.
* **`activateState(AgentState stateIn)`:** Deactivates the old state, resets forces via `agent->enableAllForces()`, and configures the new state (e.g., enabling/disabling specific forces, adjusting waypoints and velocity multipliers).
* **`deactivateState(AgentState state)`:** Handles cleanup upon exiting a state, such as resetting talking/listening IDs, force factors, and active waypoints.

### State Transition Logic

Transitions rely on `stateMaxDuration` (randomized around a base time) for timeout-based exits, alongside probability and condition-based checks (e.g., `agent->tellStory()`). Prerequisite conditions—such as ensuring enough people are nearby and no one else is currently talking—must be met before a transition occurs. Some states employ a 0.5s throttle to prevent rapid, erratic state switching.

### Pedestrian States

| State | Description |
| :--- | :--- |
| **StateNone** | Initial state with no active destination. |
| **StateWalking** | Normal movement; all forces enabled. |
| **StateRunning** | Fast movement; 2x speed multiplier. |
| **StateTalking** | Stopped for conversation; all forces disabled. |
| **StateTalkingAndWalking** | Chatting while moving; 0.3x speed, social force disabled. |
| **StateListening** | Receiving conversation; maintains distance to the talker. |
| **StateListeningAndWalking** | Following the talker while listening; forces disabled. |
| **StateTellStory** | Broadcasting a story (20s base duration); all forces disabled. |
| **StateGroupTalking** | Multi-person chat; KeepDistance force enabled, social force multiplier set to 15.0. |
| **StateRequestingService** | Looking for a service robot; 0.2x speed multiplier. |
| **StateReceivingService** | Being serviced by a robot; 0.01x speed (nearly stopped). |
| **StateRequestingGuide** | Waiting for a guide robot to arrive. |
| **StateFollowingGuide** | Following a robot; maintains a strict 3.1m distance. |
| **StateRequestingFollower** | Requesting a follower robot. |
| **StateGuideToGoal** | Being actively guided to a destination within the arena. |
| **StateClearingGoal** | Final movement sequence after reaching the arena goal. |
| **StateWaitForTimer** | Startup delay state; 0 speed. |
| **StateWaitForTrigger** | Startup trigger state; 0 speed. |

### Vehicle, Forklift, and Robot States

| Entity type | States |
| :--- | :--- |
| **Vehicles / Forklifts** | StateDriving, StateReachedShelf, StateLiftingForks (~3s), StateLoading (~3s), StateLoweringForks (~3s), StateBackUp. |
| **Service Robots** | StateDriving (patrolling waypoints), StateDrivingToInteraction (moving to requesting agent), StateProvidingService (servicing agent). |

## Simulation Orchestrator (`Simulator.cpp` / rosnav)

The `Simulator.cpp` acts as the primary orchestrator. It does not calculate forces or update states directly; instead, it runs the core 25Hz simulation loop and delegates physics updates to `SCENE.moveAllAgents()`.

### Topics vs. Services

* **Topic:** A continuous stream of signals or data.
* **Service:** A one-time operation or request.

### Data Flow

* **Input:** Robot odometry (`/odom`), dynamic reconfigure sliders (setting force factors from 0-10), and Python force overrides (`pedsim_agents_feedback`).
* **Processing:** Entirely handled via `SCENE.moveAllAgents()`, which triggers `updateState`, `computeForces`, and `move` for all entities.
* **Output:** Publishes results to `simulated_agents`, `pedsim_agents_data`, and `pedsim_agents_feedback`.

### Key Functions and Callbacks

| Function | Purpose |
| :--- | :--- |
| **initialize Simulation** | Handles complex setup and environment allocations; returns True/False. |
| **runSimulation** | Executes the 25Hz loop, calling `SCENE.moveAllAgents()` each frame. |
| **reconfigure CB** | Callback that monitors dynamic sliders and updates force factors (0-10). |
| **spawn Callback** | Manages the dynamic creation of new human agent clusters over time. |
| **update Robot Position** | Reads and processes incoming robot odometry. |
| **publish Robot Position** | Pushes out the standard ROS odometry data stream. |
| **get Agent States** | Packages all agent data into a comprehensive `agentStates` message (Note: This message contains pose, velocity, and semantics, which is distinct from the internal state machine states). |
| **publish Groups** | Publishes group membership information for visualization. |
| **Environment Getters** | `getWalls()`, `getObstacles()`, `getWaypoints()` retrieve geometry from the scene. |
| **publishPedSimAgents()** | Publishes all updated agent data to the `simulated_agents` topic. |
| **onPedsimAgents()** | Receives Python force overrides from `pedsim_agents_feedback` and applies them to agents. |
