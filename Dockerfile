FROM nvidia/cuda:12.6.0-runtime-ubuntu24.04

SHELL ["/bin/bash", "-lc"]

#Run the frontend first so it doesn't throw an error later
RUN apt-get update \
  && export TZ="America/New_York" \
  && DEBIAN_FRONTEND=noninteractive apt-get install -y keyboard-configuration \
  && DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata \
  && DEBIAN_FRONTEND=noninteractive apt-get install -y locales \
  && ln -fs "/usr/share/zoneinfo/$TZ" /etc/localtime \
  && dpkg-reconfigure --frontend noninteractive tzdata \
  && apt-get clean

# General dependencies for development
RUN apt-get update \
  && apt-get install -y --install-recommends\
  build-essential \
  cmake \
  coinor-libipopt-dev \
  cppcheck \
  curl \
  dbus-x11 \
  g++ \
  gcc \
  gdb \
  gfortran \
  git \
  libasio-dev \
  libbluetooth-dev \
  libcwiid-dev \
  libeigen3-dev \
  libgoogle-glog-dev \
  liblapack-dev \
  libpcl-dev \
  libspnav-dev \
  libusb-dev \
  lsb-release \
  mercurial \
  pkg-config \
  python3-dbg \
  python3-empy \
  python3-pip \
  python3-venv \
  python3-sklearn \
  software-properties-common \
  sudo \
  swig \
  wget \
  cmake-curses-gui \
  geany \
  tmux \
  iputils-ping \
  default-jre \
  iproute2 \
  zstd \
  tmux \
  && apt-get clean

RUN add-apt-repository -y ppa:neovim-ppa/stable \
  && apt-get update \
  && DEBIAN_FRONTEND=noninteractive apt-get install -y neovim

# Requires a docker build argument `user_id`
ARG user_id=1000
ENV USER=dcist
RUN deluser ubuntu && useradd -U --uid ${user_id} -ms /bin/bash $USER \
  && echo "$USER:$USER" | chpasswd \
  && adduser $USER sudo \
  && echo "$USER ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USER

# enable all nvidia capabilities
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

# Copy computer_installer as sudo and give permission to execute
COPY clearpath_computer_installer.sh /usr/local/bin/clearpath_computer_installer.sh
RUN chmod +x /usr/local/bin/clearpath_computer_installer.sh

# Computer installer must be run as a user, not sudo
USER $USER
ENV AUTO_YES=1
RUN bash /usr/local/bin/clearpath_computer_installer.sh

#Switch back to root for rest of setup
USER root

# Install ROS2-related packages
RUN apt-get update && apt-get install -y --no-install-recommends \
	ros-jazzy-clearpath-simulator \
	ros-jazzy-topic-tools \
	ros-jazzy-navigation2 \
	ros-jazzy-nav2-bringup \
	ros-jazzy-slam-toolbox \
	python3-vcstool && \
	apt-get clean && rm -rf /var/lib/apt/lists/*

WORKDIR /home/${USER}
COPY clearpath /etc/clearpath

# Copy the dcist_ws environment
COPY --chown=$USER:$USER ../ws /home/$USER/dcist_ws/src/

# Install the Ouster driver
RUN mkdir -p /home/$USER/dcist_ws/src \
  && git clone -b ros2 --recurse-submodules \
  	https://github.com/ouster-lidar/ouster-ros.git \
	/home/$USER/dcist_ws/src/ouster-ros

# Install the UBlox driver
RUN git clone -b ros2 https://github.com/KumarRobotics/ublox.git \
	/home/$USER/dcist_ws/src/ublox

# Install the Vectornav driver
RUN git clone -b ros2 https://github.com/dawonn/vectornav.git \
	/home/$USER/dcist_ws/src/vectornav

# build the dcist_ws
RUN cd /home/$USER/dcist_ws \
  && /bin/bash -c 'source /opt/ros/jazzy/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    colcon build --symlink-install'

RUN echo 'export PS1="\[$(tput setaf 2; tput bold)\]\u\[$(tput setaf 7)\]@\[$(tput setaf 3)\]\h\[$(tput setaf 7)\]:\[$(tput setaf 4)\]\W\[$(tput setaf 7)\]$ \[$(tput sgr0)\]"' >> ~/.bashrc
CMD ["/bin/bash"]
