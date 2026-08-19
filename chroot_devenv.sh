#!/usr/bin/env bash
#
# chroot_devenv.sh - replicate the docker devenv (ROS Noetic / Ubuntu 20.04)
# without requiring a container runtime. Builds a Focal rootfs with debootstrap,
# installs ROS Noetic + workspace dependencies inside it, and runs catkin_make
# via chroot(1).
#
# Usage:
#   ./chroot_devenv.sh setup   # create + configure the chroot environment
#   ./chroot_devenv.sh build   # run the compilation (catkin_make)
#   ./chroot_devenv.sh run     # launch openmower
#   ./chroot_devenv.sh shell   # open an interactive shell inside the chroot
#   ./chroot_devenv.sh cleanup # unmount (+ optionally delete) the rootfs
#   ./chroot_devenv.sh         # setup, build, then run
#
# Env overrides:
#   DEVENV_ROOTFS       path to the rootfs dir (default /var/ros-noetic-chroot)
#   UBUNTU_MIRROR       base mirror used for debootstrap (defaults to ports, because ARM64)
#   ROS_PACKAGES        space-separated ROS apt packages (default ros-noetic-desktop-full)
#   DEVENV_USE_HOST_ROS reuse a host ROS install at /opt/ros/${ROS_DISTRO} instead of
#                       installing ROS inside the chroot. auto|1|0 (default 0).
#
# Note: when DEVENV_USE_HOST_ROS is on, /opt/ros is bind-mounted from the host, so
# rosdep update is skipped (it would write into the host's mounted tree) and the
# host ROS was built for the host OS - its libs' transitive system deps are not in
# the Focal chroot. Install any missing libs manually (or via DEVENV_EXTRA_APT).
#
# Must be run as root (debootstrap and chroot require it).
set -euo pipefail

# ----------------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}" && pwd)"
WORKSPACE_IN_CHROOT="/workspace"

ROOTFS="${DEVENV_ROOTFS:-/var/ros-noetic-chroot}"
UBUNTU_MIRROR="${UBUNTU_MIRROR:-https://ports.ubuntu.com/ubuntu-ports}"
HOST_ARCH="${HOST_ARCH:-arm64}"
ROS_PACKAGES="${ROS_PACKAGES:-ros-noetic-desktop-full}"
ROS_DISTRO="noetic"
ROS_REPO_URL="http://packages.ros.org/ros/ubuntu"
ROS_KEY_URL="https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc"

# Reuse a host ROS installation at /opt/ros instead of installing ROS inside the
# chroot. auto -> use host ROS if /opt/ros/${ROS_DISTRO} exists, else install.
DEVENV_USE_HOST_ROS="${DEVENV_USE_HOST_ROS:-0}"
HOST_ROS_PATH="/opt/ros/${ROS_DISTRO}"

# ----------------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------------
info()  { printf '[devenv] %s\n' "$*"; }
warn()  { printf '[devenv][warn] %s\n' "$*" >&2; }
fatal() { printf '[devenv][error] %s\n' "$*" >&2; exit 1; }

require_root() {
    [[ "$(id -u)" -eq 0 ]] || fatal "must be run as root (needed for debootstrap and chroot)"
}

# Decide whether to reuse the host's /opt/ros tree inside the chroot.
host_ros_enabled() {
    case "${DEVENV_USE_HOST_ROS}" in
        auto)
            [[ -d "${HOST_ROS_PATH}" ]] && [[ -f "${HOST_ROS_PATH}/setup.bash" ]]
            ;;
        1|true|yes|on|y)
            if [[ -d "${HOST_ROS_PATH}" ]] && [[ -f "${HOST_ROS_PATH}/setup.bash" ]]; then
                true
            else
                warn "DEVENV_USE_HOST_ROS=1 but ${HOST_ROS_PATH} is missing; falling back to in-chroot install"
                false
            fi
            ;;
        0|false|no|off|n)
            false
            ;;
        *)
            warn "unknown DEVENV_USE_HOST_ROS value '${DEVENV_USE_HOST_ROS}'; treating as auto"
            [[ -d "${HOST_ROS_PATH}" ]] && [[ -f "${HOST_ROS_PATH}/setup.bash" ]]
            ;;
    esac
}

# Mount the virtual filesystems + workspace needed inside the chroot.
mount_all() {
    mkdir -p "${ROOTFS}/proc" "${ROOTFS}/sys" "${ROOTFS}/dev" "${ROOTFS}/dev/pts" "${ROOTFS}/workspace"

    mountpoint -q "${ROOTFS}/proc" || mount --bind /proc "${ROOTFS}/proc"
    mountpoint -q "${ROOTFS}/sys"  || mount --bind /sys  "${ROOTFS}/sys"
    mountpoint -q "${ROOTFS}/dev"  || mount --bind /dev  "${ROOTFS}/dev"
    mountpoint -q "${ROOTFS}/dev/pts" || mount --bind /dev/pts "${ROOTFS}/dev/pts" 2>/dev/null || true

    # The workspace (open_mower_ros) is bind-mounted at /workspace so build
    # artifacts (build/, devel/) persist on the host.
    mountpoint -q "${ROOTFS}/workspace" || mount --bind "${WORKSPACE}" "${ROOTFS}/workspace"

    # If reusing a host ROS install, bind-mount /opt/ros into the chroot so
    # setup/build/shell all see it.
    if host_ros_enabled; then
        mkdir -p "${ROOTFS}/opt/ros/${ROS_DISTRO}"
        mountpoint -q "${ROOTFS}/opt/ros/${ROS_DISTRO}" || mount --bind "${HOST_ROS_PATH}" "${ROOTFS}/opt/ros/${ROS_DISTRO}"
    fi

    # Networking inside the chroot (DNS + hosts).
    if [[ -f /etc/resolv.conf ]]; then
        cp --remove-destination /etc/resolv.conf "${ROOTFS}/etc/resolv.conf"
    fi
    if [[ -f /etc/hosts ]]; then
        cp --remove-destination /etc/hosts "${ROOTFS}/etc/hosts"
    fi
}

unmount_all() {
    for m in "${ROOTFS}/workspace" "${ROOTFS}/dev/pts" "${ROOTFS}/dev" "${ROOTFS}/sys" "${ROOTFS}/proc" "${ROOTFS}/opt/ros/${ROS_DISTRO}"; do
        mountpoint -q "$m" && umount "$m" || true
    done
}

# Run a command inside the chroot (mounts first, unmounts on exit).
run_chroot() {
    mount_all
    trap 'unmount_all' EXIT
    chroot "${ROOTFS}" /bin/bash -c "$1"
}

# ----------------------------------------------------------------------------
# setup
# ----------------------------------------------------------------------------
setup() {
    require_root

    if [[ -d "${ROOTFS}" && -f "${ROOTFS}/usr/bin/bash" ]]; then
        info "rootfs already present at ${ROOTFS}; reusing it"
    else
        info "creating Focal rootfs at ${ROOTFS}"
        if ! command -v debootstrap >/dev/null 2>&1; then
            info "installing debootstrap on the host"
            apt-get update && apt-get install -y debootstrap
        fi
        mkdir -p "${ROOTFS}"
        debootstrap --arch ${HOST_ARCH} --variant=minbase --include=ca-certificates,locales focal "${ROOTFS}" "${UBUNTU_MIRROR}"
    fi

    # Configure apt sources inside the chroot (Focal is EOL -> old-releases).
    info "configuring apt sources inside the chroot"
    cat > "${ROOTFS}/etc/apt/sources.list" <<EOF
deb ${UBUNTU_MIRROR} focal main restricted universe multiverse
deb ${UBUNTU_MIRROR} focal-updates main restricted universe multiverse
deb ${UBUNTU_MIRROR} focal-security main restricted universe multiverse
EOF

    # Base tooling the devenv Dockerfile installs.
    info "installing base tooling inside the chroot"
    run_chroot "
        export DEBIAN_FRONTEND=noninteractive
        apt-get update
        apt-get install -y --no-install-recommends \
            sudo git zsh gdb rsync ssh lsb-release gnupg ca-certificates libgtest-dev \
            libgmock-dev cmake 
    " # python3-empy python3-nose

    if host_ros_enabled; then
        info "reusing host ROS at ${HOST_ROS_PATH} (bind-mounted into the chroot)"
    else
        # ROS Noetic apt repository. Download the key into the rootfs on the host,
        # then add it inside the chroot.
        info "adding ROS ${ROS_DISTRO} apt repository inside the chroot"
        if ! command -v curl >/dev/null 2>&1 && ! command -v wget >/dev/null 2>&1; then
            info "installing curl on the host"
            apt-get update && apt-get install -y curl
        fi
        (command -v curl >/dev/null 2>&1 && curl -fsSL -o "${ROOTFS}/ros.asc" "${ROS_KEY_URL}") \
            || wget -qO "${ROOTFS}/ros.asc" "${ROS_KEY_URL}"
        run_chroot "
            apt-key add /ros.asc && rm -f /ros.asc
            echo 'deb ${ROS_REPO_URL} focal main' > /etc/apt/sources.list.d/ros-latest.list
        "

        info "installing ${ROS_PACKAGES} inside the chroot"
        run_chroot "
            export DEBIAN_FRONTEND=noninteractive
            apt-get update
            apt-get install -y --no-install-recommends ${ROS_PACKAGES}
        "

        info "running rosdep update inside the chroot"
        run_chroot "
            export ROS_DISTRO=${ROS_DISTRO}
            source /opt/ros/${ROS_DISTRO}/setup.bash
            apt install python3-rosdep
            rosdep init
            rosdep update --rosdistro ${ROS_DISTRO}
        "
    fi

    # Workspace rosdep dependencies (same approach as the Dockerfiles: build an
    # apt install list via --simulate, then install it plus libasio-dev for slic3r).
    info "installing workspace rosdep dependencies inside the chroot"
    run_chroot "
        export DEBIAN_FRONTEND=noninteractive
        export ROS_DISTRO=${ROS_DISTRO}
        source /opt/ros/${ROS_DISTRO}/setup.bash
        cd ${WORKSPACE_IN_CHROOT}
        rosdep install --from-paths src --ignore-src --simulate | \
            sed --expression '1d' | sort | tr -d '\n' | sed --expression 's/  apt-get install//g' > /apt-install-list
        apt-get update
        apt-get install -y --no-install-recommends \$(cat /apt-install-list) libasio-dev build-essential
        rm -f /apt-install-list
    "

    info "setting up nginx"
    run_chroot "
        apt install -y nginx 
        rm -rf /var/www /etc/nginx/sites-enabled/* 
        mkdir /opt/open_mower_ros
        ln -s /workspace/src/open_mower_ros/web /opt/open_mower_ros/web
    "
    info "copying nginx config into chroot"
    cp "./src/open_mower_ros/docker/assets/nginx.conf" "${ROOTFS}/etc/nginx/conf.d/default.conf"

    info "setting up mosquitto"
    run_chroot "
        apt install -y mosquitto
    "
    info "copying mosquitto config into chroot"
    cp "./src/open_mower_ros/docker/assets/mosquitto.conf" "${ROOTFS}/etc/mosquitto/mosquitto.conf"

    info "setup complete: ${ROOTFS}"
}

# ----------------------------------------------------------------------------
# build
# ----------------------------------------------------------------------------
build() {
    require_root
    [[ -d "${ROOTFS}" && -f "${ROOTFS}/usr/bin/bash" ]] || {
        warn "rootfs missing; running setup first"
        setup
    }

    info "building goat firmware..."
    run_chroot "
        cd ${WORKSPACE_IN_CHROOT}
        cmake open_goat_firmware/ -B build_fw/
        cd build_fw/
        make
    "

    info "running catkin_make inside the chroot"
    # catkin_init_workspace only creates src/CMakeLists.txt if it is absent
    # (it fails when the symlink already exists from a prior build).
    # -j1 because we run out of memory on the goat otherwise 
    run_chroot "
        export ROS_DISTRO=${ROS_DISTRO}
        source /opt/ros/${ROS_DISTRO}/setup.bash
        cd ${WORKSPACE_IN_CHROOT}
        [ -e src/CMakeLists.txt ] || (cd src && catkin_init_workspace)
        catkin_make -j1
    "
    info "build finished"
}

# ----------------------------------------------------------------------------
# shell / cleanup
# ----------------------------------------------------------------------------
shell() {
    require_root
    [[ -d "${ROOTFS}" && -f "${ROOTFS}/usr/bin/bash" ]] || fatal "rootfs missing - run setup first"
    mount_all
    trap 'unmount_all' EXIT
    info "entering chroot shell (workspace at /workspace)"
    # launch shell with openmower prefix so I stop trying to run things outside the goddamn chroot
    debian_chroot="openmower" chroot "${ROOTFS}" /bin/bash -l
}

# Delete the rootfs only after confirming it is a genuine debootstrap chroot and
# definitely not a critical or live host path. This always runs as root, so be
# paranoid: a single bad `rm -rf` could wipe the host.
safe_rm_rootfs() {
    local path="${ROOTFS}"

    # Must be a non-empty absolute path.
    [[ -n "${path}" && "${path}" == /* ]] || fatal "refusing to delete: ROOTFS is not an absolute path"

    # Resolve symlinks to the canonical path; it must exist.
    local real
    real="$(realpath -e "${path}" 2>/dev/null)" || fatal "refusing to delete: ROOTFS does not resolve to an existing path"

    # Never allow removing a critical system or host directory.
    case "${real}" in
        /|/home|/root|/usr|/etc|/bin|/sbin|/var|/opt|/opt/ros|/tmp|/dev|/proc|/sys|/workspace|/run|/boot|/lib|/lib64|/mnt|/media)
            fatal "refusing to delete: ${real} is a protected path" ;;
    esac

    # The path must look like a debootstrap rootfs (not a live host dir).
    [[ -d "${real}/etc" && -d "${real}/etc/apt" && -f "${real}/usr/bin/bash" ]] \
        || fatal "refusing to delete: ${real} does not look like a chroot rootfs"

    # Make sure nothing inside is still a bind-mounted host path before deleting;
    # `rm -rf` would otherwise recurse into a mounted dir and destroy host data.
    unmount_all
    for t in "${real}/workspace" "${real}/dev/pts" "${real}/dev" "${real}/sys" "${real}/proc" "${real}/opt/ros/${ROS_DISTRO}"; do
        mountpoint -q "$t" && fatal "refusing to delete: ${t} is still a mount point"
    done

    info "safely removing rootfs at ${real}"
    rm -rf -- "${real}"
}

cleanup() {
    require_root
    if [[ "${1:-}" == "--delete" ]]; then
        safe_rm_rootfs
    else
        unmount_all
        info "unmounted ${ROOTFS}; use 'cleanup --delete' to also remove it"
    fi
}

launchom() {
    run_chroot "
        source /workspace/devel/setup.bash
        source /workspace/mower_config.sh
        service nginx start
        service mosquitto start
        roslaunch open_mower open_mower.launch &
        roslaunch_pid=\$!
        /workspace/build_fw/open_goat.elf
        kill \$roslaunch_pid 2>/dev/null
        service nginx stop
        service mosquitto stop
        killall nginx
        killall mosquitto
        echo Goodbye!
    "
}

# ----------------------------------------------------------------------------
# main
# ----------------------------------------------------------------------------
case "${1:-}" in
    setup)   setup ;;
    build)   build ;;
    shell)   shell ;;
    run)     launchom   ;;
    cleanup) cleanup "${2:-}" ;;
    "" )     info "no command given; running setup, build, run"; setup; build; launchom ;;
    *)       fatal "unknown command '${1}'; expected setup|build|shell|cleanup" ;;
esac
