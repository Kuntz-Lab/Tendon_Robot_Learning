# Docker for Tendon Experiments

There are quite a few files here:

* `Dockerfile-ros2`: A Dockerfile for ROS 2 (Eloquent) in Ubuntu 18.04
* `Dockerfile-ros2-ompl`: Based on the ROS 2 image, add OMPL
* `Dockerfile-ros2-dev`: Based on the ROS 2 OMPL image, add developer tools
* `build.sh`: Build the Docker images:
  * `ros2:18.04-eloquent`
  * `ros2:18.04-eloquent-ompl`
  * `ros2-dev:18.04-eloquent-ompl`
* `run.sh`: Create an run a container based on the
  `ros2-dev:18.04-eloquent-ompl` image and call it `ros2-dev`
  * The container will launch a new terminal within the container


## Building

You can use the `build.sh` script directly, but it takes a few hours to
complete (mostly the OMPL image).  To do that, just call the script.

If you want to save some time, the first two docker images have been moved to
Dockerhub.  If you want to use these instead, then do the following:

Change the first line in `Dockerfile-ros2-dev` from

```
FROM ros2:18.04-eloquent-ompl
```

to

```
FROM mikebentley15/ros2:18.04-eloquent-ompl
```

Then you can comment out the first two `docker build` commands in `build.sh`
(to leave only the last `docker build` command left), and run `build.sh`.


## Running

There is a script called `run.sh`.  You are not required to run the image this
way.  You can look inside and use it as an example in order to have access to
your home directory and be able to launch X11 windows from inside the
container.

The `run.sh` script launches a separate terminal from within the container.
Also, every teriminal launched within the container will automatically source
the `/opt/ros-linux/setup.bash` setup file for convenience.


## Bash Logic

It may be helpful to distinguish host terminal shells from those terminal shells launched in the Docker container, there is a file created in the image called `/etc/docker-name`.

Here is a little excerpt from my bashrc file specifying my prompt:

```bash
# ANSI color codes
RS="\[\033[0m\]"    # reset
HC="\[\033[1m\]"    # hicolor
INV="\[\033[7m\]"   # inverse background and foreground
FBLK="\[\033[30m\]" # foreground black
FRED="\[\033[31m\]" # foreground red
FGRN="\[\033[32m\]" # foreground green
FYEL="\[\033[33m\]" # foreground yellow

alias in-docker="awk -F/ '\$2 == \"docker\"' /proc/self/cgroup | read"

# Set the prompt variable
PS1="${HC}${FGRN} \! \W \$${RS} "
if in-docker; then
  if [ -f "/etc/docker-name" ]; then
    _name="$(cat /etc/docker-name)"
  else
    _name="docker"
  fi
  PS1="${HC}${FYEL} (${_name})${RS}$PS1"
  unset _name
fi
```

When inside of a docker container, it will put the name of the docker container
at the beginning of the prompt in yellow (if there is a file caled
`/etc/docker-name`).  If there is not a file at `/etc/docker-name`, it will
have the word `(docker)` at the beginning of the prompt in yellow.
