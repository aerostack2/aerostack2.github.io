Pages:

00. Getting started
01. Aerostack2 Concepts
02. Examples
03. Aerial Platform
04. Robot Behaviors
05. AS2 Plugins
06. User Interfaces
08. ROS 2 Common interfaces
09. Development
10. Roadmaps
11. License Agreement: 3-Clause BSD License
12. Citations
13. About and Contact
14. Projects
15. User Reviews


Extended Table of Contents:

00. Getting started
    - License agreement
    - Previous dependencies
    - Ubuntu (Debian)
        - Binary Install
        - Source Install
            - Source Installation
            - Standard Build
            - Build with AS2 CLI
01. Aerostack2 Concepts
    - Architecture
        - Inter-process Communication
        - Platform and Sensors
        - Robotics Functions
        - Behaviors
        - Mission Control
        - Applications
    - Aerostack2 Inter-process Communication
    - Aerial Platform
    - Motion Controller
    - State Estimator
02. Examples
    - Prerequisites
    - Basics Examples
        - Keyboard Teleoperation Example
        - Swarm Keyboard Teleoperation Example
        - Python Example
        - Swarm Python Example
03. Aerial Platform
    - Gazebo
        - Introduction
        - Installation
            - Prerequisites
            - Install platform package
            - Install simulation assets
        - Aerostack2 Common Interface
            - Control Modes
            - Sensors
        - Config Simulation
        - Platform Launch
    - Pixhawk 4
        - Introduction
        - Installation
            - Prerequisites
                - Fast-DDS-Gen
                - pyros-genmsg
            - Install platform package
        - Aerostack2 Common Interface
            - Control Modes
            - Sensors
        - Platform Launch

    - Bitcraze Crazyflie
        - Introduction
        - Installation
            - Prerequisites
            - Install platform package
        - Aerostack2 Common Interface
            - Control Modes
            - Sensors
        - Platform Launch
    - Ryze Tello
    - DJI Matrice Series
04. Robot Behaviors
    - Motion Behaviors
        - Follow Reference Behavior
    - Perception Behaviors
    - Trajectory Generation Behaviors
        - Generate Polynomial Trajectory Behavior
    - Parameters Estimation Behaviors
    - Swarm Flocking Behaviors
    - Payload Behaviors
05. AS2 Plugins
    - Motion Controller Plugins
    - State Estimator Plugins
    - Behaviors Plugins
        - Motion Behaviors Plugins
        - Trajectory Generation Behavior Plugins
        - Follow Reference Behavior Plugins
06. User Interfaces
    - Behavior Tree
    - Keyboard Teleoperation
    - Alphanumeric Viewer
    - Web Gui
08. ROS 2 Common interfaces
    - Aerial Platform
        - Topics
        - Services
    - Motion Controller
        - Topics
        - Services
    - State Estimator
        - Topics
    - Behaviors
        - Actions
    - AS2 Msgs
    - Using AS2 Interfaces
09. Development
    - Command Line Interface (CLI)
    - Tutorials
        - Writing a New Aerial Platform
            - Overview
            - Requirements
            - Tutorial Steps
                - Abstract class, Aerial Platform
                - Overriden methods
                    - Class constuctor
                    - Sensor configuration
                    - Control mode
                    - Send command
                    - Arming
                    - Offboard
                    - Emergency kill switch
                    - Emergency stop
                    - Takeoff
                    - Land
                - Control modes configuration
        - Writing a New Motion Controller Plugin
            - Overview
            - Requirements
            - Architecture
                - Plugin Contract
            - Tutorial Steps
                - Plugin skeleton
                - Initialization
                - Essential parameters and parameter callbacks
                - State and reference hooks
                - Mode handling
                - Compute output
                - Reset and hover
                - Exporting the plugin
                - Configuration files
                - Launching
        - Writing a New State Estimator Plugin
            - Overview
            - Requirements
            - Tutorial Steps
                - Estimator Plugin Base
                - Overriden methods
                    - Setup
                - Exporting the plugin
                - Launching
        - Writing a New Behavior
            - Overview
            - Requirements
            - Behavior Server
            - Behavior Client
        - Writing a New Trajectory Generation Plugin
        - Writing a New Follow Reference Plugin
            - Overview
            - Requirements
            - Architecture
                - Plugin Contract
            - Tutorial Steps
                - Plugin skeleton
                - Initialization
                - Goal activation
                - Goal modification (optional)
                - Deactivation, pause and resume
                - Execution end
                - Per-tick run
                - Exporting the plugin
                - Configuration files
                - Launching
    - Develop Guide
        - Developing a New Package
            - Architecture of a New Package
            - Code Style
                - C++
                - Python
            - Test
                - Code Style Test
                - Code Functional Test
                - Code Coverage Test
    - API Documentation
        - AS2 Core
        - AS2 Msgs
        - AS2 Motion Controller
        - AS2 State Estimator
        - AS2 Behavior
10. Roadmaps
11. License Agreement: 3-Clause BSD License
12. Citations
13. About and Contact
14. Projects
15. User Reviews
