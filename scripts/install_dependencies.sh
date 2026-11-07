#!/usr/bin/env bash
# Install the common Ubuntu 20.04 / ROS Noetic apt dependencies only.
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE=install
DUFO=false
usage() {
  cat <<'HELP'
用法：bash scripts/install_dependencies.sh [--check] [--dufomap]
  默认       安装 Ubuntu/ROS 常用 apt 依赖（需要已配置 ROS Noetic 软件源）。
  --check    只检查系统及模拟 apt 安装，不使用 sudo，不修改系统。
  --dufomap  额外安装 dufomap==1.1.1 到项目内独立虚拟环境；已存在时只检查。
  -h, --help 显示此说明。
不安装 PX4、PedSim、GPU 驱动、CUDA、PyTorch/gsplat，不编译工作空间。
HELP
}
for arg in "$@"; do
  case "$arg" in
    --check) MODE=check ;;
    --dufomap) DUFO=true ;;
    -h|--help) usage; exit 0 ;;
    *) printf '未知参数: %s\n' "$arg" >&2; usage >&2; exit 2 ;;
  esac
done
if [[ ! -r /etc/os-release ]]; then echo '找不到 /etc/os-release。' >&2; exit 1; fi
# shellcheck disable=SC1091
source /etc/os-release
if [[ "${ID:-}" != ubuntu || "${VERSION_ID:-}" != 20.04 || "$(dpkg --print-architecture)" != amd64 ]]; then
  echo '该入口仅针对 Ubuntu 20.04 amd64；其他系统未验证。' >&2; exit 1
fi
PACKAGES=(
  build-essential cmake git curl python3-pip python3-venv python3-numpy python3-scipy
  python3-catkin-tools python3-rosdep libeigen3-dev libpcl-dev libopencv-dev
  libyaml-cpp-dev libgoogle-glog-dev libgflags-dev libceres-dev libgeographic-dev pcl-tools
  gazebo11 libgazebo11-dev ros-noetic-desktop-full ros-noetic-mavros ros-noetic-mavros-extras
  ros-noetic-gazebo-ros-pkgs ros-noetic-pcl-ros ros-noetic-tf2-sensor-msgs
)
if [[ "$MODE" == check ]]; then
  echo '检查 Ubuntu 20.04 amd64 及已配置软件源中的依赖；以下是 apt 模拟，不会安装。'
  apt-get -s --no-remove install "${PACKAGES[@]}"
  if [[ "$DUFO" == true ]]; then
    echo 'DUFOMap: 安装模式会使用 .venvs/dufomap；已有环境只检查，不覆盖。'
  fi
  echo '检查完成；未安装或构建，模拟通过不代表完整项目可运行。'
  exit 0
fi
if [[ "${EUID}" -eq 0 ]]; then SUDO=(); else SUDO=(sudo); fi
if ! command -v apt-get >/dev/null; then echo '缺少 apt-get。' >&2; exit 1; fi
if [[ ${#SUDO[@]} -gt 0 ]] && ! command -v sudo >/dev/null; then echo '请先安装 sudo，或使用管理员执行。' >&2; exit 1; fi
echo '将更新 apt 索引并安装常用依赖；不更改软件源或清除已有构建。'
"${SUDO[@]}" apt-get update
if ! apt-cache show ros-noetic-desktop-full >/dev/null 2>&1; then
  echo '未找到 ROS Noetic 软件包；请先按 docs/环境与依赖安装.md 配置 ROS Noetic 软件源。' >&2
  exit 1
fi
"${SUDO[@]}" apt-get --no-remove install -y "${PACKAGES[@]}"
if [[ "$DUFO" == true ]]; then
  DUFO_ENV="$ROOT_DIR/.venvs/dufomap"
  if [[ -e "$DUFO_ENV" ]]; then
    echo "已存在 $DUFO_ENV，只检查版本，不修改已有环境。"
    [[ -x "$DUFO_ENV/bin/python" ]] || { echo '已有路径不是有效虚拟环境。' >&2; exit 1; }
    "$DUFO_ENV/bin/python" -c 'from importlib.metadata import version; v=version("dufomap"); print("dufomap",v); assert v=="1.1.1", "需要 dufomap 1.1.1"'
  else
    echo '创建独立虚拟环境并安装 DUFOMap 1.1.1。'
    bash "$ROOT_DIR/scripts/run_sh/install_dufomap_demo.sh"
  fi
fi
echo '常用 apt 依赖安装完成；这不等于完成 PX4、插件、项目编译或飞行验证。'
echo '请继续阅读 docs/环境与依赖安装.md，检查 PX4、GeographicLib、PedSim 和可选 3DGS。'
