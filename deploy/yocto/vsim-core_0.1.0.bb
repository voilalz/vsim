# Yocto / OpenEmbedded 配方骨架：把 vsim_core（纯 C++17、无 ROS 依赖）编进目标板镜像。
# 放到自己的 layer（如 meta-vsim/recipes-vsim/vsim-core/）下，按实际仓库地址修改 SRC_URI / SRCREV。
# ROS 2 部分（vsim_interfaces、vsim_ros）用 meta-ros 构建，见 docs/hardware_integration.md。
SUMMARY = "vsim core: plant models, ECU application/runtime and hardware abstraction layer"
LICENSE = "Apache-2.0"
# TODO：确定许可证后加入 LICENSE 文件，并把下面的 md5 换成 `md5sum LICENSE` 的实际值
LIC_FILES_CHKSUM = "file://${WORKDIR}/git/LICENSE;md5=CHANGE_ME"

SRC_URI = "git://git.example.com/vsim.git;protocol=https;branch=main"
SRCREV = "${AUTOREV}"
S = "${WORKDIR}/git/src/vsim_core"

inherit cmake

# 目标板上不跑单元测试；需要时打开并把 vsim_core_tests 装进 ${PN}-ptest
EXTRA_OECMAKE = " \
    -DVSIM_BUILD_TESTS=OFF \
    -DVSIM_BUILD_APPS=ON \
    -DVSIM_WARNINGS_AS_ERRORS=OFF \
"

# 接入真实外设时打开（需要补全对应实现，见 hardware_integration.md）
PACKAGECONFIG ??= ""
PACKAGECONFIG[gpiod] = "-DVSIM_WITH_GPIOD=ON,-DVSIM_WITH_GPIOD=OFF,libgpiod"
PACKAGECONFIG[soem] = "-DVSIM_WITH_SOEM=ON,-DVSIM_WITH_SOEM=OFF,soem"

# 运行期需要的内核模块（SocketCAN）；vcan 只在板上做回环测试时需要
RRECOMMENDS:${PN} += "kernel-module-can kernel-module-can-raw kernel-module-vcan can-utils"

FILES:${PN} += "${datadir}/ament_index ${datadir}/vsim_core"
FILES:${PN}-dev += "${libdir}/cmake/vsim_core"
