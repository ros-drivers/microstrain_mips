FROM ros:noetic-ros-base-focal

# Install MSCL (github deb install is kind of a pain)
ENV DEBIAN_FRONTEND="noninteractive"
RUN set -ex \
    && apt-get update && apt-get install -y \
        jq \
        curl \
    && install_github_deb () { \
        org="${1}"; \
        repo="${2}"; \
        tag="${3}"; \
        artifact="${4}"; \
        list_asset_url="https://api.github.com/repos/${org}/${repo}/releases/tags/${tag}"; \
        asset_url=$(curl -fsSl "${list_asset_url}" | jq ".assets[] | select(.name==\"${artifact}\") | .url" | sed 's/\"//g'); \
        curl -fsSLJ -H 'Accept: application/octet-stream' "${asset_url}" -o "/tmp/${artifact}"; \
        dpkg -i "/tmp/${artifact}" || apt-get install -f; \
        rm -f "/tmp/${artifact}"; \
    } \
    && install_github_deb "LORD-MicroStrain"  "MSCL" "v62.0.0" "c++-mscl_62.0.0_amd64.deb"

# Use rosdep to install the dependencies of the packages
COPY ros_mscl /tmp/src/ros_mscl
COPY mscl_msgs /tmp/src/mscl_msgs
RUN set -ex \
    && rosdep install --from-paths /tmp/src --ignore-src -r -y \
    && rm -rf /tmp/src
