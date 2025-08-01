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

ARG USERNAME=dcist
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

RUN ( id -u ubuntu &>/dev/null && deluser --remove-home ubuntu || true ) && \
    ( getent group "${USER_GID}" >/dev/null || groupadd --gid "${USER_GID}" "${USERNAME}" ) && \
    ( getent passwd "${USER_UID}" >/dev/null || \
      useradd --uid "${USER_UID}" --gid "${USER_GID}" -m -s /bin/bash "${USERNAME}" ) && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/90-${USERNAME} && \
    chmod 0440 /etc/sudoers.d/90-${USERNAME}


# enable all nvidia capabilities
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

# Copy computer_installer as sudo and give permission to execute
COPY clearpath_computer_installer.sh /usr/local/bin/clearpath_computer_installer.sh
RUN chmod +x /usr/local/bin/clearpath_computer_installer.sh

# Computer installer must be run as a user, not sudo
USER $USERNAME
ENV AUTO_YES=1
RUN bash /usr/local/bin/clearpath_computer_installer.sh
#Switch back to root for rest of setup
USER root


COPY <<'EOF' /usr/local/bin/entrypoint.sh
#!/usr/bin/env bash
set -euo pipefail

USERNAME=${USERNAME:-dcist}
HOME_DIR=/home/${USERNAME}
export HOME=${HOME_DIR}
export ROS_HOME=${ROS_HOME:-${HOME_DIR}/.ros}
export ROS_LOG_DIR=${ROS_LOG_DIR:-${ROS_HOME}/log}

mkdir -p "${ROS_HOME}" "${ROS_LOG_DIR}" || true
chown -R "${USERNAME}:${USERNAME}" "${ROS_HOME}" || true

# hand off to CMD as the user
exec sudo -H -E -u ${USERNAME} \
  --preserve-env=ROS_HOME,ROS_LOG_DIR,ROS_DISTRO,BASH_ENV \
  "$@"
EOF

RUN chmod +x /usr/local/bin/entrypoint.sh

WORKDIR /home/${USERNAME}
COPY clearpath /etc/clearpath

# download and install the ZED SDK
RUN cd /home/dcist \
  && wget -O zed_sdk.run https://download.stereolabs.com/zedsdk/5.0/cu12/ubuntu24 \
  && chmod +x zed_sdk.run \
  && ./zed_sdk.run -- silent runtime_only\
  && rm zed_sdk.run

# Copy the dcist_ws environment
COPY --chown=$USERNAME:$USERNAME ../ws /home/$USERNAME/dcist_ws

# build the dcist_ws
RUN cd /home/dcist/dcist_ws \
  && /bin/bash -c 'source /opt/ros/jazzy/setup.bash && \
    sudo apt update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src --rosdistro=jazzy -y && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"'

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["/bin/bash"]
