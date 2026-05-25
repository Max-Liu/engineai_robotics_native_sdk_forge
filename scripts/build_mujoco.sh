#!/bin/bash

# Exits on error
set -e

# Gets the source directory
root_dir="$(realpath -s $(cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)/..)"
mujoco_dir="$root_dir/simulation/mujoco"
deps_archive="$mujoco_dir/mujoco_deps_x86.tar.xz"
deps_dir="$mujoco_dir/_deps"

usage() {
  cat <<'EOF'
Usage: ./scripts/build_mujoco.sh [options]

Options:
  -m, --mirror-deps        Download MuJoCo dependencies from mirror repositories.
                           Use this only when GitHub is unreachable.
  -h, --help               Show this help message.
EOF
}

parse_args() {
  while (($# > 0)); do
    case "$1" in
      -m|--mirror-deps)
        use_mirror_deps=1
        ;;
      -h|--help)
        usage
        exit 0
        ;;
      *)
        echo "Unknown argument: $1"
        ;;
    esac
    shift
  done
}

download_mirrors_mujoco_deps() {
  if [ ! -f "$deps_dir/glfw3-src/CMakeLists.txt" ]; then
    git clone "https://gitee.com/mirrors/glfw.git" "$deps_dir/glfw3-src"
  fi

  if [ ! -f "$deps_dir/lodepng-src/lodepng.cpp" ]; then
    git clone "https://gitee.com/mirrors/LodePNG.git" "$deps_dir/lodepng-src"
  fi
}

has_local_mujoco_deps() {
  [ -f "$deps_dir/glfw3-src/CMakeLists.txt" ] && [ -f "$deps_dir/lodepng-src/lodepng.cpp" ]
}

prepare_build_dir() {
  local cache_file="$build_dir/CMakeCache.txt"

  if [ ! -f "$cache_file" ]; then
    return
  fi

  local cached_source_dir
  local cached_build_dir
  cached_source_dir="$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' "$cache_file" | tail -n 1)"
  cached_build_dir="$(sed -n 's/^CMAKE_CACHEFILE_DIR:INTERNAL=//p' "$cache_file" | tail -n 1)"

  if [ "$cached_source_dir" != "$mujoco_dir" ] || [ "$cached_build_dir" != "$build_dir" ]; then
    echo "Existing MuJoCo CMake cache belongs to a different checkout; recreating $build_dir"
    echo "  cached source: ${cached_source_dir:-unknown}"
    echo "  current source: $mujoco_dir"
    echo "  cached build:  ${cached_build_dir:-unknown}"
    echo "  current build:  $build_dir"
    cmake -E rm -rf "$build_dir"
  fi
}

parse_args "$@"

if [[ "${use_mirror_deps:-0}" == "1" ]]; then
  echo "Downloading MuJoCo deps from gitee"
  download_mirrors_mujoco_deps
fi

if ! has_local_mujoco_deps && [ -f "$deps_archive" ]; then
  echo "Extracting local MuJoCo deps from $deps_archive ..."
  tar -xf "$deps_archive" -C "$mujoco_dir"
fi

cmake_args=(-DBUILD_RELEASE=ON -DMUJOCO_ENABLE_LTO=OFF)
if has_local_mujoco_deps; then
  echo "Using local MuJoCo deps from $deps_dir"
  cmake_args+=(-DMUJOCO_USE_LOCAL_DEPS=ON)
fi

# Builds the project
build_dir="$mujoco_dir/build"
prepare_build_dir
mkdir -p "$build_dir" && cd "$build_dir"
cmake "${cmake_args[@]}" ..

# Compiles with 2 threads less than the number of cores
num_cores=$(expr $(nproc) - 2)
if [ $num_cores -lt 1 ]; then
  num_cores=$(nproc)
fi
cmake --build . --parallel $num_cores
