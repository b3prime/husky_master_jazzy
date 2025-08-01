#!/bin/bash -e

platform="A200"
serial_number="0266"
ROS_DISTRO_MANUAL="jazzy"

#Log functions
log_space() {
  # Print a blank line
  echo ""
}

log_info() {
  # Print the information message
  echo -e "${INFO_BLUE}$1${RESET_TEXT}"
}

log_warn() {
  # Print the warning message
  echo -e "${WARN_YELLOW}$1${RESET_TEXT}"
}

log_error() {
  # Print the error message
  echo -e "${ERROR_RED}$1${RESET_TEXT}"
}

log_done() {
  # Print the error message
  echo -e "${DONE_GREEN}$1${RESET_TEXT}"
  log_space
}


# Determine the OS and ROS version
step_get_os_and_ros_version() {
  # Get the Ubuntu version
  UBUNTU_VERSION=$(. /etc/os-release && echo $UBUNTU_CODENAME)

  # Exit the script if the ROS version is unsupported
  if [[ "$ROS_DISTRO_MANUAL" == "unsupported" ]]; then
    log_error "Ubuntu version ($UBUNTU_VERSION) does not have a supported version of ROS 2, exiting"
    exit 0
  fi
}

# Setup Open Robotics package server to install ROS 2
step_setup_osrf_packge_server() {
  log_info "Setup Open Robotics package server to install ROS 2 $ROS_DISTRO_MANUAL"

  if [ -e /etc/apt/sources.list.d/ros2.list ]; then
    # Remove old ROS 2 installation if present
    # See https://discourse.ros.org/t/ros-signing-key-migration-guide/43937
    log_info "Detected old ROS 2 installation"
    sudo rm /etc/apt/sources.list.d/ros2.list
    sudo rm /usr/share/keyrings/ros-archive-keyring.gpg
  fi

  # Check if ROS 2 sources are already installed
  if dpkg -s ros2-apt-source &> /dev/null; then
    log_warn "ROS 2 sources exist, skipping"
  else
    sudo apt -y -qq install software-properties-common
    sudo add-apt-repository universe -y
    sudo apt -y -qq update && sudo apt -y -qq upgrade && sudo apt -y -qq install curl -y

    # See https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
    sudo apt install /tmp/ros2-apt-source.deb
    sudo rm /tmp/ros2-apt-source.deb

    # Check if sources were added
    if ! dpkg -s ros2-apt-source &> /dev/null; then
      log_error "Unable to add ROS 2 package server, exiting"
      exit 0
    fi
  fi

  log_done "Setup ROS 2 package server"
}

# Setup Clearpath Robotics package server
step_setup_cpr_packge_server() {
  log_info "Setting up Clearpath Robotics package server"

  # Check if Clearpath sources are already installed
  if [ -e /etc/apt/sources.list.d/clearpath-latest.list ]; then
    log_warn "Clearpath Robotics sources exist, skipping"
  else
    wget https://packages.clearpathrobotics.com/public.key -O - | sudo apt-key add -
    sudo bash -c 'echo "deb https://packages.clearpathrobotics.com/stable/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/clearpath-latest.list'
    # Check if sources were added
    if [ ! -e /etc/apt/sources.list.d/clearpath-latest.list ]; then
      log_error "Unable to add Clearpath Robotics package server, exiting"
      exit 0
    fi
  fi

  log_done "Setup Clearpath Robotics package server"
}

# Install the ROS 2 packages needed for the given ROS 2 distro
step_install_ros_packages() {
  log_info "Updating packages and installing ROS 2"
  sudo apt -y -qq update
  # All ROS distros
  sudo apt install -y -qq  iw ros-$ROS_DISTRO_MANUAL-ros-base ros-$ROS_DISTRO_MANUAL-clearpath-robot python3-argcomplete ros-dev-tools python3-vcstool python3-ds4drv python3-clearpath-computer-setup

  if [[ "$ROS_DISTRO_MANUAL" == "jazzy" ]]; then
    sudo apt -y -qq  install ros-jazzy-foxglove-bridge openssh-server
  fi
  log_done "Updating packages and installing ROS 2"
}

# Setup rosdep for OSRF and Clearpath packages
step_setup_rosdep() {
  log_info "Configuring rosdep"

  # Check if rosdep sources are already installed
  if [ -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    log_warn "rosdep was already initialized, skipping"
  else
    sudo rosdep -q init
    # Check if sources were added
    if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then
      log_error "rosdep failed to initialize, exiting"
      exit 0
    fi
  fi

  # Check if Clearpath rosdep sources are already installed
  if [ -e /etc/ros/rosdep/sources.list.d/50-clearpath.list ]; then
    log_warn "Clearpath Robotics rosdeps exist, skipping"
  else
    sudo wget -q https://raw.githubusercontent.com/clearpathrobotics/public-rosdistro/master/rosdep/50-clearpath.list -O \
      /etc/ros/rosdep/sources.list.d/50-clearpath.list
    # Check if sources were added
    if [ ! -e /etc/ros/rosdep/sources.list.d/50-clearpath.list ]; then
      log_error "Clearpath Robotics rosdeps, exiting"
      exit 0
    fi
  fi

  log_done "Configuring rosdep"
}

setup_change_net_timeout() {
  log_info "Configuring network service, if needed"
  # Check if the service file exists
  if [ -e "/lib/systemd/system/systemd-networkd-wait-online.service" ]; then
    # Check if timeout is present in the service file
    if grep -q "timeout=30" "/lib/systemd/system/systemd-networkd-wait-online.service"; then
      log_info "Timeout is already present in /lib/systemd/system/systemd-networkd-wait-online.service"
    else
      # Add --timeout=30 after ExecStart=/lib/systemd/systemd-networkd-wait-online
      sudo sed -i '/^ExecStart/ s/$/ --timeout=30/' "/lib/systemd/system/systemd-networkd-wait-online.service"
      log_info "Timeout added to /lib/systemd/system/systemd-networkd-wait-online.service"
    fi
  else
    log_warn "Service file /lib/systemd/system/systemd-networkd-wait-online.service not found."
  fi
  # Ensure the service is enabled
  sudo systemctl enable systemd-networkd-wait-online

  log_done "Configuring network service, if needed"
}


# Set front end to non-interactive to avoid prompts while installing packages
export DEBIAN_FRONTEND=noninteractive

# Check if the script is run as root
if [ "$EUID" -eq 0 ]; then
 log_warn "You are the root user, this needs to be run as a user to be completed."
fi

# Temporarily disable the blocking messages about restarting services in systems with needrestart installed
if [ -d /etc/needrestart/conf.d ]; then
  sudo bash -c "echo '\$nrconf{restart} = '\''a'\'';' > /etc/needrestart/conf.d/10-auto-cp.conf"
fi

step_setup_osrf_packge_server

step_setup_cpr_packge_server

step_install_ros_packages

step_setup_rosdep

setup_change_net_timeout

if [ ! "$EUID" -eq 0 ]; then

  log_info "Updating rosdep"
  rosdep -q update
  log_done "Updating rosdep"

  # Check if Clearpath folder exists
  if [ -d /etc/clearpath/ ]; then
    log_warn "Clearpath folder exist, skipping"
  else
    log_info "Creating setup folder"
    sudo mkdir -p -m 777 /etc/clearpath/
    # Check if directory was created
    if [ !  -d /etc/clearpath/ ]; then
      log_error "Clearpath folder setup, exiting"
      exit 0
    fi
  fi

  platform_serial_number="$platform-$serial_number"
  hostname_string="cpr-$platform-$serial_number"
  platform_namespace="$platform"_"$serial_number"

  # Check if the hostname is cpr-unassigned
  log_info "Checking hostname\e[0m"
  if [ "$(hostname)" = "clearpath-unassigned" ]; then
    log_info "Hostname is currently set to 'clearpath-unassigned'."
    sudo hostnamectl set-hostname "$hostname_string"
    # Display the new hostname
    log_info "Hostname changed to '$hostname_string'."
    sudo sed -i "s/clearpath-unassigned/$hostname_string/g" /etc/hosts
    # Notify the user to restart for changes to take effect
    log_info "Please restart your system for the changes to take effect."
  else
      log_info "Hostname is already set to '$(hostname)'. No changes needed."
  fi
  log_done "Checking hostname"

  source /opt/ros/$ROS_DISTRO_MANUAL/setup.bash

  wget -c https://raw.githubusercontent.com/clearpathrobotics/clearpath_computer_installer/main/cockpit_installer.sh && bash -e cockpit_installer.sh

  ros2 run clearpath_robot install
  sudo systemctl enable clearpath-robot

  log_info "Setting up Clearpath environment"
  grep -qxF "source /etc/clearpath/setup.bash" ~/.bashrc || echo "source /etc/clearpath/setup.bash" >> ~/.bashrc
  log_done "Setting up Clearpath environment"


  log_info "Setting up groups\e[0m"

  # Add user to dialout group for accessing Serial port
  sudo usermod -aG dialout $(whoami);
  if [ $(getent group video) ]; then
    log_info "video group already exists";
  else
    log_info "Adding video group";
    sudo addgroup video;
  fi
  if id -nGz "$(whoami)" | grep -qzxF "video"; then
    log_info "User:$(whoami) is already in video group";
  else
    log_info "Adding user:$(whoami) to video group";
    sudo usermod -a -G video $(whoami);
  fi

  if [ $(getent group flirimaging) ]; then
    log_info "flirimaging group already exists";
  else
    log_info "Adding flirimaging group";
    sudo addgroup flirimaging;
  fi
  if id -nGz "$(whoami)" | grep -qzxF "flirimaging"; then
    log_info "User:$(whoami) is already in flirimaging group";
  else
    log_info "Adding user:$(whoami) to flirimaging group";
    sudo usermod -a -G flirimaging $(whoami);
  fi

  val=$(< /sys/module/usbcore/parameters/usbfs_memory_mb)
  if [ "$val" -lt "2048" ]; then
    if [ -e /etc/default/grub ]; then
      if [ $(grep -c "usbcore.usbfs_memory_mb=" /etc/default/grub) -eq 0 ]; then # Memory Limit has not already been set
        sudo sed -i 's/GRUB_CMDLINE_LINUX_DEFAULT="[^"]*/& usbcore.usbfs_memory_mb=2048/' /etc/default/grub
        log_info "Increased the usbfs memory limits in the default grub configuration. Updating grub"
        sudo update-grub
      else
        log_warn "usbfs memory limit is already set in /etc/default/grub in the following line:\e[0m"
        echo "$(grep "usbcore.usbfs_memory_mb" /etc/default/grub)"
        log_warn "No changes made, verify that usbfs_memory_mb is set to a minimum of 2048 and then try rebooting the computer"
      fi

    else
      log_warn "/etc/default/grub configuration file not found, no changes made. usbfs_memory_mb must be set manually."
      log_warn "See https://github.com/ros-drivers/flir_camera_driver/tree/humble-release/spinnaker_camera_driver#setting-up-linux-without-spinnaker-sdk for instructions"
      exit 0
    fi
  else
    log_info "usbfs_memory_mb is already set to $val, no changes necessary."
  fi

  log_done "Setting up groups"
  log_done "Clearpath Computer Installer Complete"
  log_space
else
  log_warn "Clearpath Computer Installer needs to be ran as a user, change configuration in Dockerfile."
  log_space
fi

# Re-enable messages about restarting services in systems with needrestart installed
if [ -e /etc/needrestart/conf.d/10-auto-cp.conf ]; then
  sudo rm /etc/needrestart/conf.d/10-auto-cp.conf
fi

if ping -c1 gitlab.clearpathrobotics.com; then
  log_info "Downloading wireless configuration script for use later"
  wget https://gitlab.clearpathrobotics.com/research/lv426-netplan/-/raw/main/configure-lv426.sh -O /home/$USER/setup-lv426.sh
  chmod +x /home/$USER/setup-lv426.sh
fi
