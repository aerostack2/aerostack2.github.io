## License agreement

Before the installation, you must [read and accept the conditions defined by the software license](https://github.com/aerostack/install/wiki/License)

## Previous dependencies
Make sure to have installed ROS2 via Debian Packages [[Galactic]](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html).

For simulation install Ignition Fortress -> https://gazebosim.org/docs/fortress/install_ubuntu 

## Installation

> For using and installing Aerostack2, your ROS2 environment must be settled up. If ROS2 is installed using debian packages you should run ```$ source /opt/ros/<your distro>/setup.bash``` before running these steps.

- Create a AS2 workspace:
```bash
mkdir -p ${HOME}/aerostack2_ws/src/ && cd ${HOME}/aerostack2_ws/src/
```

- Clone AS2 repository:
```bash
git clone https://github.com/aerostack2/aerostack2.git
```

- Install core dependencies:
```bash
sudo apt update
sudo apt install tmux python3-vcstool python3-rosdep python3-pip python3-colcon-common-extensions -y
sudo rosdep init
```

- Set environment vars (or add them to the ~/.bashrc file):
```bash
echo 'export AEROSTACK2_PATH=$HOME/aerostack2_ws/src/aerostack2' >> $HOME/.bashrc
echo 'source $AEROSTACK2_PATH/scripts/setup_env.bash' >> $HOME/.bashrc
```

## Release installation
Open a new terminal or update current shell with ```$ source ~/.bashrc```

- Download repositories (from project or source release):
    - From source release:
    ```bash
    cd $AEROSTACK2_PATH
    vcs import --recursive < ./installers/as2_beta_beetroot.repos
    ```

    - From project:
    ```bash
    # e.g. install mbzirc project ("as2 project -l" to list available projects)
    as2 project -n mbzirc
    ```

- Install dependencies:
```bash
cd ${HOME}/aerostack2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

- Build:
```bash
as2 build
```

More information on how to run each project can be found in the README.md located on each project folder.
