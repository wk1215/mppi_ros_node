# Setting up development environment on docker

## Prerequisites
- any os that supports docker (following settings was tested on ubuntu 20.04)
- [docker](https://docs.docker.com/engine/install/ubuntu/)

## First time setup
1. clone the project repository.
    ```
    cd <path to your workspace>
    git clone https://github.com/MizuhoAOKI/mppi_swerve_drive_ros
    ```
1. build the docker image.   
   make sure to connect to the internet because dependent packages will be downloaded.
    ```
    cd <path to your workspace>/mppi_swerve_drive_ros
    docker compose build
    ```
1. start the docker container.
    ```
    cd <path to your workspace>/mppi_swerve_drive_ros
    docker compose up -d
    ```
1. get into the bash inside the running container.
    ```
    cd <path to your workspace>/mppi_swerve_drive_ros
    docker compose exec noetic bash
    ```
1. [inside the docker container] clean the cache.
    ```
    cd ~/mppi_swerve_drive_ros
    rm -rf build devel logs .catkin_tools
    ```
1. [inside the docker container] build the project.
    ```
    cd ~/mppi_swerve_drive_ros
    source /opt/ros/noetic/setup.bash
    catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O2"
    source devel/setup.bash
    ```

## [After the first time setup] Use the running container
1. the command `docker compose up -d` above started a docker container named `noetic_container`.  
    check if noetic_container is running.   
   if it is running currently, you should see `noetic_container` in the list.  
   "STATUS" saying "Up" means the container is running now.
    ```
    $ docker ps -a
    CONTAINER ID   IMAGE          COMMAND                  CREATED         STATUS         PORTS     NAMES
    13f765669c57   655f16d731bd   "/ros_entrypoint.sh …"   4 seconds ago   Up 3 seconds             noetic_container
    ```
1. get into the bash inside the running container.
    ```
    docker compose exec noetic bash
    ```
1. [inside the docker container] exit the container.
    this command just exits from the bash inside the container and returns to the host terminal, but the container itself remains running.
    ```
    exit
    ```


## Other commands
-  stop the container.
    ```
    docker compose stop
    ```
-  restart the container.
    ```
    docker compose start
    ```
    use this command when you run `docker compose exec ...` but the output says the container is not running.
-  clean up.
    1. remove the container.
        ```
        docker compose down
        ```
    1. remove the image too.
        ```
        docker compose down --rmi local
        ```


## Note
On the docker environment above, you cannot access to GPU devices.
It is because this project does not heavily depend on GPU devices. 
If you need to use GPU devices, please tell us by creating an issue or a pull request.

