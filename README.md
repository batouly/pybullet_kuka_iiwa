
# Server-Client PyBullet

### To run this demo locally
1. Add the packages into your src folder in catkin_ws
2. Build the workspace and source it as usual
    ```
    catkin build
    . devel/setup.bash
    ```
3. Launch the server and the client
    ```
    roslaunch pybullet_server kuka_server_client.launch
    ```