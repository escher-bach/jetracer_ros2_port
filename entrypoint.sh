#!/bin/bash
set -e

# Source ROS 2 Humble setup
source /opt/ros/humble/setup.bash

# Source the workspace setup
source /ros2_ws/install/setup.bash

# Export CycloneDDS implementation
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Ensure ROS_DOMAIN_ID is set (defaults to 0 if not provided)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# GPU (phase 1, mounts): make the compute-slice bind mounts indistinguishable
# from a normally installed CUDA/cuDNN/TensorRT box - real lib and header
# names in the standard multiarch paths, so plain `g++ -lnvinfer`,
# `#include <NvInfer.h>`, nvcc and cmake work with zero extra flags.
# No-op when the mounts are absent (building/running off the Jetson).
# We link ONLY real files from the pantries and synthesize the soname/dev
# names ourselves: the host's own dev symlinks (libcudnn.so, cudnn.h) route
# through /etc/alternatives, which does not exist in this container.
# Validated by jetson_cuda_experiment H0b/H2/H5.
if [ -d /usr/lib/aarch64-linux-gnu/tegra ] && [ -d /hostlib ]; then
    libdir=/usr/lib/aarch64-linux-gnu

    # libcuda soname links (the ro tegra mount lacks libcuda.so.1)
    cuda_drv=$(ls $libdir/tegra/libcuda.so* 2>/dev/null | sort | tail -1)
    if [ -n "$cuda_drv" ]; then
        ln -sf "$cuda_drv" $libdir/libcuda.so.1
        ln -sf "$cuda_drv" $libdir/libcuda.so
    fi

    # Scattered compute families: link each real file, then synthesize the
    # .so.N / .so chain its name implies (libcudnn.so.8.0.0 -> .so.8 -> .so)
    for fam in libcublas libcublasLt libcudnn libnvinfer libnvparsers \
               libnvonnxparser libmyelin; do
        for f in /hostlib/${fam}*.so*; do
            if [ -f "$f" ] && [ ! -L "$f" ]; then
                name=$(basename "$f")
                ln -sf "$f" "$libdir/$name"
                chain=$name
                while [ "${chain%.*}" != "$chain" ]; do
                    chain=${chain%.*}
                    case "$chain" in
                        *.so|*.so.*) [ -e "$libdir/$chain" ] || ln -sf "$f" "$libdir/$chain" ;;
                        *) break ;;
                    esac
                done
            fi
        done
    done
    ldconfig

    # Headers: TensorRT (Nv*.h) + cuDNN (cudnn*.h) into the standard multiarch
    # include dir; also provide the _v8-stripped cuDNN names normally supplied
    # by the host's alternatives system.
    if [ -d /hostinclude ]; then
        incdir=/usr/include/aarch64-linux-gnu
        for h in /hostinclude/aarch64-linux-gnu/Nv*.h \
                 /hostinclude/aarch64-linux-gnu/cudnn*.h \
                 /hostinclude/cudnn*.h; do
            if [ -f "$h" ] && [ ! -L "$h" ]; then
                name=$(basename "$h")
                ln -sf "$h" "$incdir/$name"
                if [ "${name/_v8/}" != "$name" ]; then
                    ln -sf "$h" "$incdir/${name/_v8/}"
                fi
            fi
        done
    fi
fi

# Execute the command passed into this entrypoint
exec "$@"
