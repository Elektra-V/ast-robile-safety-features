# AST: Robile Safety Features

This repository contains source code to the Robile safety features task.

## Usage

Assuming you have Ubuntu installed on your system:

```bash
# IF YOU HAVEN'T ALREADY
# Install ROS2 ("Humble" distribution)
bash scripts/install-ros.sh

# Source ROS2
source /opt/ros/humble/local_setup.bash

# Create a new virtual environment & start it
virtualenv venv --system-site-packages
source ./venv/bin/activate
```

For development with VSCode, make sure you install the [ROS](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros) extension and configure it properly. This will include creating a `settings.json` file in `.vscode` folder (will be done automatically by the ROS extension):

```json
{
    "python.autoComplete.extraPaths": [
        "/opt/ros/humble/lib/python3.10/site-packages",
        "/opt/ros/humble/local/lib/python3.10/dist-packages"
    ],
    "python.analysis.extraPaths": [
        "/opt/ros/humble/lib/python3.10/site-packages",
        "/opt/ros/humble/local/lib/python3.10/dist-packages"
    ],
    "ros.distro": "humble"
}
```

Upon install and successful configuration, VSCode will be able to configure the workspace.

## License

This project is open-sourced under the MIT license.