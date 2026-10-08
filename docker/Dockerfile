# vsim 容器镜像：一个镜像给所有单元用（ROS 2 + rmw_zenoh + foxglove_bridge + CAN 工具 + 本工程）。
#
#   docker build -f docker/Dockerfile -t vsim:lyrical .                           # 在仓库根目录执行
#   docker build -f docker/Dockerfile --build-arg ROS_DISTRO=jazzy -t vsim:jazzy .
#   docker build -f docker/Dockerfile --build-arg RUN_TESTS=0 ...                 # 跳过构建期单元测试
# 官方 ros 镜像同时提供 amd64 与 arm64；arm64 镜像用于"ECU 指令集级"仿真（见 compose.yaml 中 ecu 的注释）。
ARG ROS_DISTRO=lyrical

# ---------------------------------------------------------------- 构建阶段
FROM ros:${ROS_DISTRO}-ros-base AS build
ARG ROS_DISTRO
ARG RUN_TESTS=1
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
WORKDIR /ws

# 先只拷 package.xml 装依赖，源码改动时可复用这一层缓存
COPY src/vsim_core/package.xml src/vsim_core/package.xml
COPY src/vsim_interfaces/package.xml src/vsim_interfaces/package.xml
COPY src/vsim_ros/package.xml src/vsim_ros/package.xml
RUN apt-get update \
 && rosdep update --rosdistro "${ROS_DISTRO}" \
 && rosdep install --from-paths src --ignore-src -y --rosdistro "${ROS_DISTRO}" \
 && rm -rf /var/lib/apt/lists/*

COPY src ./src
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
 && colcon build --merge-install --event-handlers console_direct+ \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DVSIM_FETCH_GTEST=OFF \
 && if [[ "${RUN_TESTS}" == "1" ]]; then \
      colcon test --merge-install --packages-select vsim_core --event-handlers console_direct+ \
      && colcon test-result --verbose; \
    fi

# ---------------------------------------------------------------- 运行阶段
FROM ros:${ROS_DISTRO}-ros-base
ARG ROS_DISTRO
RUN apt-get update && apt-get install -y --no-install-recommends \
      "ros-${ROS_DISTRO}-rmw-zenoh-cpp" \
      "ros-${ROS_DISTRO}-foxglove-bridge" \
      can-utils iproute2 kmod \
 && rm -rf /var/lib/apt/lists/*

COPY --from=build /ws/install /ws/install
COPY docker/entrypoint.sh /vsim_entrypoint.sh
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp \
    VSIM_PREFIX=/ws/install
WORKDIR /ws
ENTRYPOINT ["/vsim_entrypoint.sh"]
CMD ["bash"]
