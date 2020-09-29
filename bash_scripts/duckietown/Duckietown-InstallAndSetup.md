# Install on Ubuntu

## Install helpful utilities

1. Download VSCode [Visual Studio Code](https://code.visualstudio.com/)
   1. Unpack and install: 
      ```
      sudo dpkg -i path_to_deb_file
      ```

2. Install alternative Linux terminal - ```Terminator```
   ```
   sudo apt-get install terminator
   ```
3. Installing ```Git```
   1. ```
      sudo apt-get install git
      ```
   2. ```
      sudo apt-get install git-extras
      ```
   3. Configure Git
      1. ```
         git config --global user.email "email"
         ```
   
      2. ```
         git config --global user.name "full name"
         ```

---

## Helpful (I'm stuck Links)
- [Docker Permissions Denied](https://www.digitalocean.com/community/questions/how-to-fix-docker-got-permission-denied-while-trying-to-connect-to-the-docker-daemon-socket)
- [Post Installation Instructions](https://docs.docker.com/engine/install/linux-postinstall/)
- [Docker Cheat Sheet!](https://dockerlabs.collabnix.com/docker/cheatsheet/)
- [Duckietown Op Manual](https://docs.duckietown.org/DT19/opmanual_duckiebot/out.pdf)

---

## Installing Docker

1. Install Docker using the ```Install using the repository```. This is the recommended way of installing Docker - [Docker Install](https://docs.docker.com/engine/install/ubuntu/)
2. Verify that Docker was successfully setup, run in terminal: 
   ```
   sudo docker run hello-world
   ```
3. Add your user to the ```docker``` group to use Docker as a non-root user (Warning about this from Docker: ```"Adding a user to the “docker” group grants them the ability to run containers which can be used to obtain root privileges on the Docker host"```)
   1. Run command in terminal: 
      ```
      sudo usermod -aG docker <your-user>
      ```
      1. You can use ```${USER}``` in place of the ```<your-user>``` to use the currently logged in user
4. **After, logout and then log back in for changes to take effect.**

---

## Installing Portainer

1. Above Docker install is required
2. Enter the following commands in terminal and execute:
   1. ```
      docker volume create portainer_data
      ```
   2. ```
      docker run -d -p 9000:9000 --name=portainer --restart=always -v /var/run/docker.sock:/var/run/docker.sock -v portainer_data:/data portainer/portainer-ce
      ```
   3. After the above two, Portainer should be up and running (Docker container is up and running)
   4. Now navigate to: [http:localhost:9000/](http:localhost:9000/)
   5. From there you'll be asked to enter a setup password for the admin account. After you'll be all set to go.

---

## Let's pull a sample Docker image for Ubuntu!
1. Run in terminal:
   1. ```
      docker pull library/ubuntu:18.04
      ```
      1. This gets the Ubuntu 18.04 image
   2. ```
      docker image list
      ```
      1. This will list all of the downloaded images
   3. ```
      docker run -it ubuntu:18.04
      ```
      1. This will start the Ubuntu 18.04 Docker image (will start in ```interactive``` and ```pseudo-tty``` [allows terminal access in container])
   4. Now you're inside the container!
   5. To detach from the container, press the following keys:
      1. ```ctrl``` + ```p```, then while still holding those two keys, hit ```q``` 
         1. This should drop you back into your terminal, and the Docker container will be running in the background
   6. To re-attach to the container, run the following in the terminal:
      1. ```
         docker attach <container-name>
         ```
         1. This will put you back into the container, to get out, do the 5th step!
   7. To stop a running container, run the following in terminal:
      1. ```
         docker stop <container-name>
         ```
   8. To remove a docker container, run the following in terminal:
      1. ```
         docker rm <container-name>
         ```
   9. To remove a docker image, run the following in terminal:
      1. ```
         docker rmi <image-name>
         ```

---

## Building and running Duckietown ROS Image
1. Clone the Git repo from official duckietown maintainers of Docker image - for now we'll use ```master19``` branch: [Repo](https://github.com/duckietown/rpi-ros-kinetic-base)
2. Open terminal in the root directory of the newly cloned repo
3. Copy this build script into the root directory of the git repo - [buildme.sh](https://github.com/CalebABG/CCSUTourbot-ROS/blob/master/bash_scripts/duckietown/buildme.sh) script to build the ```amd64``` arch version of the Docker image instead of the default ```arm32v7``` image which is designed to be used on the duckie-car/bot
   1. You will need to make the script executable. In the terminal with the root directory of the git repo, type in and execute this command:
      1. ```
         chmod +x buildme.sh
         ```
4. Now that the script is executable, last thing to do is start the build of the image!
5. In the same terminal, type in and execute this command:
   1. ```
      ./buildme.sh
      ```
6. Once the build is complete, now to test the built image!
7. In the same terminal, now run this command:
   1. ```
      docker run -it --privileged ccsu_duckietown/dt18/rpi-ros-kinetic-base:master19-amd64
      ```
8. If all goes well, you'll be dropped into the container in a linux shell!
9. From that shell, type in the following and then press enter: ```roscore``` 
10. And Boom! Roscore should have started and you should see info output!
11. To detach from the container refer to step 5 from ```Let's pull a sample Docker image for Ubuntu!``` from above!

---

## Building and running base Duckietown Docker image (NOT COMPLETE [experimental for now])

1. Enter in terminal and execute following command:
   1. ```
      docker pull duckietown/rpi-ros-kinetic-base:master19
      ```
      1. This will pull the master18 version of the Docker image for duckietown (this is the arm32v7 image)
2. Install QEMU:
   1. Enter in terminal and execute following commands:
      1. ```
         sudo apt-get install qemu-kvm qemu virt-manager virt-viewer libvirt-bin
         ```
      2. ```
         sudo apt-get install gcc-arm-linux-gnueabihf libc6-dev-armhf-cross qemu-user-static
         ```
3. Install Nvidia Docker stuff:
   1. ```
      distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
      ```
   2. ```
      curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
      ```
   3. ```
      curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker list
      ```
   4. ```
      sudo apt-get update
      ```
   5. ```
      sudo apt-get install -y nvidia-docker2
      ```
   6. ```
      sudo systemctl restart docker
      ```
   7. Test run image
      1. ```
         docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
         ```

---

## Steps for pulling and running Duckietown Gym-Duckietown Docker image
### Notes:
- ```
   docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY duckietown/gym-duckietown
  ```
### Helper links:
- [Gym-Duckietown Github](https://github.com/duckietown/gym-duckietown)
- [Using display in Docker container](https://blog.jessfraz.com/post/docker-containers-on-the-desktop/)
- [Installing FontConfig Ubuntu](https://linoxide.com/linux-how-to/install-fonts-on-ubuntu/)
- [Docker Hub - Gym-Duckietown](https://hub.docker.com/r/duckietown/gym-duckietown)
- [Mesa-Utils Install](https://askubuntu.com/questions/1067381/upgrade-mesa-utils-ubuntu-18-04)

1. Pull base docker image for gym:
   1. ```
      docker pull duckietown/gym-duckietown
      ```
2. Run image:
   1. ```
      docker run -it duckietown/gym-duckietown
      ```
3. Install needed dependencies:
   1. ```
      apt update && apt dist-upgrade
      ```
   2. ```
      apt-get install xvfb mesa-utils -y
      ```
   3. ```
      apt-get install fontconfig
      ```
4. Save current state of container as docker image so when running again, won't have to redo steps (as exiting current container will lose changes)
   - NOTE: Replace ```<author-name> with your name; and replace <container-id> with the ID of the currently running container for gym-duckietown```
   1. ```
      docker commit -a "<author-name>" -m "Add needed dependencies to Docker container" <container-id> ccsu_duckietown/gym-duckietown:base
      ```
   2. ...

---