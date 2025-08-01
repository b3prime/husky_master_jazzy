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

RUN apt-get update \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y ros-jazzy-ros-gz

COPY <<'EOF' /usr/local/bin/entrypoint.sh
#!/usr/bin/env bash
set -euo pipefail

USERNAME=${USER:-dcist}
HOME_DIR=/home/${USER}
export HOME=${HOME_DIR}
export ROS_HOME=${ROS_HOME:-${HOME_DIR}/.ros}
export ROS_LOG_DIR=${ROS_LOG_DIR:-${ROS_HOME}/log}

mkdir -p "${ROS_HOME}" "${ROS_LOG_DIR}" || true
chown -R "${USER}:${USER}" "${ROS_HOME}" || true

# hand off to CMD as the user
exec sudo -H -E -u ${USERNAME} \
  --preserve-env=ROS_HOME,ROS_LOG_DIR,ROS_DISTRO,BASH_ENV \
  "$@"
EOF

RUN chmod +x /usr/local/bin/entrypoint.sh

WORKDIR /home/${USER}
COPY clearpath /etc/clearpath

# Copy the dcist_ws environment
COPY --chown=$USER:$USER ../ws /home/$USER/dcist_ws

# build the dcist_ws
RUN cd /home/dcist/dcist_ws \
  && /bin/bash -c 'source /opt/ros/jazzy/setup.bash && \
    sudo apt update && \
    rosdep update' 

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["/bin/bash"]
