
# Server-Client PyBullet

### To run this demo locally
1. Add the packages into your src folder in catkin_ws
2. Build the workspace and source it 
    ```catkin build
    . devel/setup.bash
    ```
3. Launch the server and the client
    ```roslaunch pybullet_server kuka_server_client.launch
    ```

### If you don't have ROS1 set up, use docker

1. Build the docker image
```docker build -t ros-noetic .
```
2. Run the Docker container 
```docker run --rm -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --name pybullet ros-noetic
```
3. Launch the server and the client
    ```. root/catkin_ws/devel/setup.bash
    roslaunch pybullet_server kuka_server_client.launch
    ```