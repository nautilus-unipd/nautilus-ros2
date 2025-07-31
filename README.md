# README under construction

# How to run
## 0. Install and build the ros2 image
This step will not be needed when the final ros2 image repo will be finalized.\
For now, to run this project these steps are needed (do this after cloning this repo on the user home directory):

```shell
# go to the user home
cd
# clone the repository 
git clone git@github.com:nautilus-unipd/raspberry-setup.git
git checkout autodocking

# build the docker image (my need sudo)
docker build -t rasp-test:latest .# this may take a while (~600s raspberry pi 5)
```
After doing this, return to this repo.

## 1. Start the ros2 docker container
Run the `connect.sh`, the first time this will download the container from the `raspbery-setup` repo, this may take a while (it needs to download ~3.5Gb). After the first time, it simply connects to it, no dowloads needed. 

```shell
CONTAINER_ID=$(docker ps -q --filter "rasp-test" | head -n 1)

if [ -n "$CONTAINER_ID" ]; then
    echo "exec"
    docker exec -it "$CONTAINER_ID" /bin/bash -c "cd /home/ubuntu/nautilus-ros2"
else
    echo "compose"	
    docker compose run --rm raspberry
fi
```
## 2. Launch the camera script
Now you can choose to run the `launch-back.sh` script or the `launch-front.sh` script that will start the respectively camera nodes.\
Make shure to be in different raspberries for the front and back, the code will not give any errors if both are executed in the same system, just the image will be duplicated.
## 3. Launch debug server
The debug server is a webpage that allow the user to see the camera feed. This server can be run on either the front or back systems, or both.\
To start the server execute, in a new window, the `connect.sh` script and after that the `debug-server.sh`.\
You can now connect view the server at the shown ip or at the system ip at the port `8081`.
## 4. Possible problems
If you encounter any problems, a first step could be to execute the `clean-build.sh` script that will delete the current `build`, `install` and `log` folders of the project. Make shure to save any log files that may interest you.

## TODO
- Implement the capture node as a Lyfecicle node
- Change the launch files accordingly
- Change the `docker-compose.yaml` file after merging the changes from the `autodoking` branch of the `raspberry-setup` repo
