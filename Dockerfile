# Use the official ROS Noetic base image
FROM px4io/px4-dev-ros-noetic

# HACK: needs to be removed once the ros noetic docker images is fixed
# RUN rm /etc/apt/sources.list.d/ros1-latest.list \
#   && rm /usr/share/keyrings/ros1-latest-archive-keyring.gpg

RUN apt-get update \
  && apt-get install -y ca-certificates curl

RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') ; \
    curl -L -s -o /tmp/ros-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && apt-get update \
    && apt-get install /tmp/ros-apt-source.deb \
    && rm -f /tmp/ros-apt-source.deb

# Set the environment variable for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WS=/opt/ros_workspace
ENV PX4_INSTALL=/opt/

# Installing cuda
ARG USING_GPU
ENV USING_GPU=${USING_GPU}
RUN echo "Should use GPU: $USING_GPU"
RUN if [ "$USING_GPU" = "true" ]; then \
      echo "USING_GPU=true so installing cuda..." \
        && wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.1-1_all.deb \
        && dpkg -i cuda-keyring_1.1-1_all.deb \
        && apt-get update \
        && apt-get -y install nvidia-container-runtime cuda-toolkit-12-8 \
        && pip install cupy-cuda12x; \
    else \
      echo "USING_GPU=false so cuda is not installed."; \
    fi
RUN 

# Install necessary packages for ROS, Gazebo, and building
COPY requirements.txt /tmp/requirements.txt
RUN apt-get update && \
    apt-get install -y $(cat /tmp/requirements.txt | cut -d'=' -f1) && \
    rm -rf /var/lib/apt/lists/*

# Hack to have ros-noetic-ros-numpy compiling
RUN pip install "numpy<1.24"

# Initialize rosdep if it hasn't been initialized
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; \
    fi && \
    rosdep update

# Create the ROS workspace
RUN mkdir -p $ROS_WS/src

# Set the working directory
WORKDIR $ROS_WS

# Clone the repository and get the commit to be checkout at from the docker-compose.yaml file
ARG BRANCH='dev/projectAVENUE'
ARG BRANCH_COMMIT=$BRANCH # Checkout the last commit per default
RUN git clone --single-branch --branch $BRANCH https://gitlab.inria.fr/chroma/drone-simulator.git $ROS_WS/src/drone-simulator \ 
    && cd $ROS_WS/src/drone-simulator/ \
    && echo "The commit to be check out is: $BRANCH_COMMIT" \
    && git checkout $BRANCH_COMMIT \
    && yes Y | ./init.sh \
    && chmod -R 777 $ROS_WS/src/drone-simulator/

ENV DIR_DRONE_SIMULATOR=$ROS_WS/src/drone-simulator
ENV DIR_DRONE_MODELS=$DIR_DRONE_SIMULATOR/packages/gazebo_models/models
ENV DIR_WORLDS=$DIR_DRONE_SIMULATOR/packages/gazebo_worlds/worlds

# RUN cd $ROS_WS/  \
#     && ln -s $DIR_DRONE_SIMULATOR/scripts .  \
#     && chmod +x scripts/*.sh

# # Build the workspace
RUN cd $ROS_WS;  \
    /bin/bash -c "source /opt/ros/noetic/setup.bash; \
    catkin_make"

RUN pip3 install --user jsonschema kconfiglib jinja2

RUN cd $PX4_INSTALL \
    && git clone --single-branch --branch dev/px4_multi_uav https://github.com/kbInria/PX4-Autopilot.git $PX4_INSTALL/PX4-Autopilot \
    && cd $PX4_INSTALL/PX4-Autopilot/ \
    && git submodule update --init --recursive \
    && DONT_RUN=1 make px4_sitl gazebo-classic_iris_velodyne

ENV DIR_PX4_FIRMWARE=$PX4_INSTALL/PX4-Autopilot

# Setup ROS environment
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source $ROS_WS/devel/setup.bash" >> /root/.bashrc
RUN echo "source /opt/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash /opt/PX4-Autopilot /opt/PX4-Autopilot/build/px4_sitl_default" >> /root/.bashrc
RUN echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:/opt/PX4-Autopilot" >> /root/.bashrc
RUN echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:/opt/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic" >> /root/.bashrc
RUN echo "export GAZEBO_PLUGIN_PATH=\$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:$ROS_WS/devel/lib" >> /root/.bashrc
RUN echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/opt/ros_workspace/src/drone-simulator/packages/gazebo_worlds/models" >> /root/.bashrc
RUN echo "export PATH=/usr/local/cuda-12.8/bin\${PATH:+:\${PATH}}" >> /root/.bashrc
RUN echo "export LD_LIBRARY_PATH=/usr/local/cuda-12.8/lib64\${LD_LIBRARY_PATH:+:\${LD_LIBRARY_PATH}}" >> /root/.bashrc

# Default command
CMD ["bash", "-c", "source devel/setup.bash && roslaunch chroma_gazebo_interfaces pointcloud_fusion_test.launch world:=kitchen"]

