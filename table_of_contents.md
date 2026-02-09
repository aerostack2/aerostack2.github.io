Pages:

00. Getting started
01. Aerostack2 Concepts
02. Examples
03. Aerial Platform
04. Robot Behaviors
05. AS2 Plugins
06. User Interfaces
07. ROS 2 Common interfaces
08. Development
15. User Reviews
09. License Agreement: 3-Clause BSD License
10. Citations
11. About and Contact 


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
    - Perception Behaviors
    - Trajectory Generation Behaviors
    - Parameters Estimation Behaviors
05. AS2 Plugins
    - Motion Controller Plugins
        - State Estimator Plugins
        - Behaviors Plugins
        - Motion Behaviors Plugins
06. User Interfaces
    - Behavior Tree
    - Keyboard Teleoperation
    - Alphanumeric Viewer
    - Web Gui
07. ROS 2 Common interfaces
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
08. Development
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
            - Tutorial Steps
                - Controller Plugin Base
                - Overriden methods
                    - Initialization
                    - Update State
                    - Update Reference
                    - Compute Output
                    - Set Mode
                    - Parameters Update
                    - Reset
                - Exporting the plugin
                - Controller manager and configuration files
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
15. User Reviews
09. License Agreement: 3-Clause BSD License
10. Citations
11. About and Contact
