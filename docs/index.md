# PedSim Documentation

An overview of where and how social forces are computed within the pedestrian simulation.

## C++ Force Calculations

The functions to compute each social force are located in the `ped_agent.cpp` file. 

*   **`desiredForce()`:** The force that pulls the pedestrian towards the waypoint.
*   **`socialForce()`:** Employs the Helbing social force model to repel from other pedestrians.
*   **`robotForce()`:** Causes pedestrian behavior to be influenced by the robot (currently not activated).
*   **`myForce()`:** Used to create any custom force to be added to calculations (currently not activated).

Each force is multiplied by a configured factor before being added to the total force. The `ped_agent.cpp` file calculates this total force, followed by the new velocity, and finally the new position for each pedestrian. This file is always used for updating velocity and position, although the forces it uses can be overridden by Python calculations.

## Python Force Calculations

While the C++ implementation provides the default force calculations, there are multiple Python-based force models available to choose from. 

The Python force calculation is handled in the `pedsim_agents\main.py` file, which chooses a force model based on the argument passed in. The `main()` function initializes the force model and listens in the main robotics simulation for real-time input data. This real-time data is then fed into a callback function to compute the forces for each pedestrian based on the chosen model. The calculated force is then sent to override the default force used in `ped_agent.cpp`.

### Available Force Models

The five available force models are located in the `pedsim_agents\pedsim_forces\forcemodels\` directory:

*   **PySocialForce:** An advanced Helbing model with group behavior where agents form groups and move together.
*   **Evacuation:** A physics-based model for emergency scenarios that helps simulate panic and evacuation behavior.
*   **DeepSocialForce:** A Machine Learning-based model that uses neural networks to predict forces.
*   **Spinny:** A debug and test model that makes agents rotate. This provides a visually obvious test to ensure forces are being applied and the override feature is being correctly implemented.
*   **Passthrough:** Passes the default C++ forces forward, but allows you to see the additional Python semantic overlay. This overlay includes information such as whether a pedestrian is moving and their velocity components. It is highly useful for comparing default C++ forces with Python force models without losing the Python-calculated semantic visual information.
