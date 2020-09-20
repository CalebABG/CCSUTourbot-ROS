# Install on Ubuntu

## Install helpful utilitties

1. Download VSCode [Visual Studio Code](https://code.visualstudio.com/)
   1. Unpack and install: ```sudo dpkg -i path_to_deb_file```
2. Install alternative Linux terminal - ```Terminator```: ```sudo apt-get install terminator```
3. Duckietown Manual [Duckietown Op Manual](https://docs.duckietown.org/DT19/opmanual_duckiebot/out.pdf)

## Helpful (I'm stuck Links)
- [Docker Permissions Denied](https://www.digitalocean.com/community/questions/how-to-fix-docker-got-permission-denied-while-trying-to-connect-to-the-docker-daemon-socket)
- [Post Installation Instructions](https://docs.docker.com/engine/install/linux-postinstall/)
- [Docker Cheat Sheet!](https://dockerlabs.collabnix.com/docker/cheatsheet/)

---

## Installing Docker

1. Install Docker using the ```Install using the repository```. This is the recommended way of installing Docker - [Docker Install](https://docs.docker.com/engine/install/ubuntu/)
2. Verify that Docker was successfully setup, run in terminal: ```sudo docker run hello-world```
3. Add your user to the ```docker``` group to use Docker as a non-root user (Warning about this from Docker: ```"Adding a user to the “docker” group grants them the ability to run containers which can be used to obtain root privileges on the Docker host"```)
   1. Run command in terminal: ```sudo usermod -aG docker <your-user>```
      1. You can use ```${USER}``` in place of the ```<your-user>``` to use the currently logged in user
4. **After, logout and then log back in for changes to take effect.**

---

## Let's pull a sample Docker image for Ubuntu!
1. Run in terminal:
   1. ```docker pull library/ubuntu:18.04```
      1. This gets the Ubuntu 18.04 image
   2. ```docker image list```
      1. This will list all of the downloaded images
   3. ```docker run -it ubuntu:18.04```
      1. This will start the Ubuntu 18.04 Docker image (will start in ```interactive``` and ```pseudo-tty``` [allows terminal access in container])
   4. Now you're inside the container!
   5. To detach from the container, press the following keys:
      1. ```ctrl``` + ```p```, then while still holding those two keys, hit ```q``` -- this should drop you back into your terminal, and the Docker container will be running in the background
   6. To re-attach to the container, run the following in the terminal:
      1. ```docker attach <container-name>```
         1. This will put you back into the container, to get out, do the 5th step!
   7. To stop a running container, run the following in terminal:
      1. ```docker stop <container-name>```
   8. To remove a docker container, run the following in terminal:
      1. ```docker rm <container-name>```
   9. To remove a docker image, run the following in terminal:
      1.  ```docker rmi <image-name>```

---

## Building and running Duckietown ROS Image
1. Clone the Git repo from official duckietown maintainers of Docker image - for now we'll use ```master19``` branch: [Repo](https://github.com/duckietown/rpi-ros-kinetic-base)
2. Open terminal in the root directory of the newly cloned repo
3. Copy this build script [buildme.sh](https://github.com/CalebABG/CCSUTourbot-ROS/blob/master/bash_scripts/duckietown/buildme.sh) script to build the ```amd64``` arch version of the Docker image instead of the default ```arm32v7``` image which is designed to be used on the duckie-car/bot
   1. You will need to make the script executable. In the terminal with the root directory of the git repo, type in and execute this command:
      1. ```chmod +x buildme.sh```
4. Now that the script is executable, last thing to do is start the build of the image!
5. In the same terminal, type in and execute this command:
   1. ```./buildme.sh ```
6. Once the build is complete, now to test the built image!
7. In the same terminal, now run this command:
   1. ```docker run -it --privileged ccsu_duckietown/dt18/rpi-ros-kinetic-base:master19-amd64 ```
8. If all goes well, you'll be dropped into the container in a linux shell!
9. From that shell, type in the following and then press enter: ```roscore``` 
10. And Boom! Roscore should have started and you should see info output!
11. To detach from the container refer to step 5 from ```Let's pull a sample Docker image for Ubuntu!``` from above!

---

## Installing Git
1. Run in terminal:
   1. ```sudo apt-get install git```
   2. ```sudo apt-get install git-extras```
   3. Configure Git
      1. ```git config --global user.email "email"```
      2. ```git config --global user.name "full name"```

---