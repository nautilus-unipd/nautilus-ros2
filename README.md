# Introduction
This project has the objective of providing a unified base framework using ROS2. It contains various modules and components that can be easily integrated and extended for different purposes.  \
The folder structure is the following:
```shell
ROS2_ws/ # ROS2 workspace folder structure
└── src/
    ├── camera_module/       # camera module node folder
    │   ├── camera_module/   # camera node code
    │   ├── config/          # camera module configuration files
    │   ├── launch/          # camera module launch files
    │   ├── resource/        
    │   └── test/            
    └── debug_server/        # debug server node folder
        ├── debug_server/    # debug server code
        ├── launch/          # debug server launch files
        ├── resource/        
        └── test/            
``` 
Future nodes must be placed accordingly to the ROS2 project structure in the `src` folder.  \
For the `camera_module` the general configurations such as resolution, framerate, quality, ecc., can be changed from the `config` folder:

```yaml
/**:
  ros__parameters:
    # Camera device settings
    camera_index: 0  # Will be overridden by launch file for each camera
    topic_name: 'image'

    # Image resolution
    width: 640
    height: 480

    # Frame rate
    fps: 30

    # Image quality (0-100, higher is better quality)
    quality: 80

    # Use compressed image transport (significantly reduces bag size)
    use_compression: true 

    # Publishing settings
    publish_rate: 30.0
    frame_id: "camera_frame"
    
    # System identification
    system_position: "front"  # front, back - will be overridden by launch file
    camera_side: "left"       # left, right - will be overridden by launch file
```

In this `stereo_camera.yaml` file you can also change the `topic_name` as you wish as it's generated dynamically. Don't worry about the `system_position` or `camera_side` as these attributes will be overwritten by the launch files.  \
The launch files are what generate the nodes. At this stage multiple launch files are needed for different positions of the camera (back, front, ecc.). In the future this may be made dynamic. Each launch file creates two node, one for each of the camera for a stereo pair. You can change the name or ID if they are not correct (eg. 1 - left, 0 - right) as sometimes the cameras IDs could be swapped at start.

```python
def generate_launch_description():
    # Get the path to the config file
    config_file = os.path.join(
        get_package_share_directory('camera_module'),
        'config',
        'stereo_camera.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='camera_module',
            executable='capture_node',
            name='capture_node_left',
            namespace='down/camera_left',
            parameters=[
                config_file,
                {
                    'camera_index': 0,  # Override camera_index for left camera
                    'system_position': 'down',
                    'camera_side': 'left'
                }
            ]
        ),
        Node(
            package='camera_module',
            executable='capture_node',
            name='capture_node_right',
            namespace='down/camera_right',
            parameters=[
                config_file,
                {
                    'camera_index': 1,  # Override camera_index for right camera
                    'system_position': 'down',
                    'camera_side': 'right'
                }
            ]
        )
    ])
```
> [!NOTE]
> In the future, if you need to create more node with different positions, create a new launch file following the existing patterns, and change all the references to the position (such as `down` in the previous example) to what you need.

# How to run


## 0. Install and build the ROS2 image

> [!WARNING]  
> This step is needed as this repository rewrites an old camera  acquisition implementation form C++ to python (`camera_module`) and some additional libraries are needed. When the docker image will be unified this step will not be needed anymore.  \
> For now, run these steps (do this after cloning this repo on the user home directory):

```shell
# go to the user home
cd ~
# clone the repository 
git clone git@github.com:nautilus-unipd/raspberry-setup.git
# switch to the branch with the added libraries
git checkout autodocking

# build the docker image (may need sudo)
docker build -t rasp-test:latest . # this step may take a while (~600s raspberry pi 5)
```

After doing this, continue with step 1.

## 1. Start the ROS2 docker container
To connect to an existing session or to start the container for the first time, use the `connect.sh` script. The first time the script will download the container from the `raspbery-setup` repo (this may take a while as it needs to download ~3.5Gb). After the first time, if not running, it will start the container or connect to an already running instance dynamically. 

```shell
CONTAINER_ID=$(docker ps -q --filter "rasp-test" | head -n 1)

if [ -n "$CONTAINER_ID" ]; then
    echo "exec"
    docker exec -it "$CONTAINER_ID" /bin/bash -c "cd /home/ubuntu/nautilus-ROS2"
else
    echo "compose"	
    docker compose run --rm raspberry
fi
```
## 2. Launch the camera script
Now you can choose to run the `launch-[back|down|front].sh` script that will start the respective camera node.\
Make sure to be in different raspberries for each node. As for now the code will not give any errors if multiple nodes are executed in the same system, just the image will be duplicated.
## 3. Launch debug server
The debug server is a webpage that allow the user to see the camera feed. This server can be run on one or multiple raspberries without any issues.\
To start the server connect to a new session with the `connect.sh` script (in a new terminal window) and after that run the `debug-server.sh`.\
You can now connect at `http://<raspberry-ip>:8081`.

## 4. Possible problems
If you encounter any problems, a useful first step is to execute the `clean-build.sh`. This script will delete the current `build`, `install` and `log` ROS2 folders of the project. Make sure to save any log files that may be useful to you before executing this script.

## TODO
- [ ] Implement the capture node as a Lifecycle node
- [ ] make the launch file dynamic `./launch.sh [up|down|back|custom]`, this need the
- [ ] Change the `docker-compose.yaml` file after merging the changes from the `autodoking` branch of the `raspberry-setup` repo
