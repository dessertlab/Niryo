# Niryo Ned2 Control Project

This project provides a collection of utilities and libraries in C and Python to control **Niryo Ned2** robotic arms via their TCP JSON protocol.

## Python Utilization
The Python implementation centers around the `MyRobot` class (found in `myrobot_class.py`), which acts as a high-level wrapper for the `pyniryo` library.

*   **Key Features:** Automatic calibration, tool detection, and support for both joint-space and Cartesian coordinates. It also adds a management layer for "Saved Poses" and "Sequences" stored in JSON files.
*   **Quick Start:**
    ```python
    from myrobot_class import MyRobot
    robot = MyRobot("192.168.100.205", "Ned2_Robot")
    robot.load_from_json("sequences.json")
    robot.run_sequence("say-hi")
    robot.disconnect()
    ```

## C Utilization
The C implementation provides a lightweight way to control the robot without heavy dependencies, communicating over TCP port 40001 using length-prefixed JSON frames.

*   **Approaches:**
    *   **Minimal:** `run_sequence.c` uses internal string parsing to avoid external dependencies.
    *   **Library-based:** `run_sequence_jsonc.c` utilizes the `json-c` library for robust JSON handling.
*   **Compilation:** Typically requires linking with the math library and `json-c`:
    ```bash
    gcc -O2 run_sequence_jsonc.c -ljson-c -lm -o run_sequence
    ```

## Provided Examples
The project includes several examples to demonstrate robot interaction:

1.  **Choreography (`using_myRobot.py`):** Demonstrates how to load a sequence of movements (like waving or picking objects) from a JSON file and execute them using the Python wrapper.
2.  **Sequence Runner (`run_sequence_jsonc.c`):** A C program that reads robot poses from a JSON file, performs a handshake and calibration, and then executes a specific movement sequence.
3.  **TCP Demo (`niryo_tcp_demo.c`):** A lower-level example showing the raw "wire protocol" by manually sending commands like `HANDSHAKE`, `CALIBRATE`, and `MOVE_JOINTS` over a raw TCP socket.
4.  **Hardware Probing (`handshake_probe.c`):** A utility to test different network terminators and connection behaviors to ensure compatibility with the robot's server.
