#!/bin/bash
# Idempotent model provisioning: sha256-pinned download + extract + one-shot
# TensorRT engine build via trtexec (H5-proven path). Engines are device- and
# TRT-version-specific, so they are built on the robot and cached in the
# docker-compose ./models volume.
# Usage: ensure_model.sh <model-name> <model-dir>
set -euo pipefail

MODEL="${1:?usage: ensure_model.sh <model-name> <model-dir>}"
MODEL_DIR="${2:?usage: ensure_model.sh <model-name> <model-dir>}"
TRTEXEC=/usr/src/tensorrt/bin/trtexec

# Standard pretrained models from the dusty-nv jetson-inference zoo
# (data/networks/models.json); sha256 computed 2026-07-15 from these URLs.
case "$MODEL" in
  fcn-resnet18-voc-320x320)
    URL=https://nvidia.box.com/shared/static/p63pgrr6tm33tn23913gq6qvaiarydaj.gz
    SHA256=6481e373a37f3cb4f449d5643b5c930ee500af204840c8c80cfcb75e28c66acd
    ;;
  fcn-resnet18-cityscapes-512x256)
    URL=https://nvidia.box.com/shared/static/k7s7gdgi098309fndm2xbssj553vf71s.gz
    SHA256=f38299549cb22b158d1e6d8f5d60276bf8de2402393a05b16ab554fd6a211727
    ;;
  *)
    echo "ensure_model: unknown model '$MODEL'" >&2
    exit 1
    ;;
esac

DEST="$MODEL_DIR/$MODEL"
ONNX="$DEST/fcn_resnet18.onnx"
ENGINE="$DEST/fcn_resnet18.engine"

if [ -f "$ENGINE" ]; then
    echo "ensure_model: engine already cached: $ENGINE"
    exit 0
fi

mkdir -p "$DEST"

if [ ! -f "$ONNX" ]; then
    TARBALL="$DEST/model.tar.gz"
    echo "ensure_model: downloading $MODEL (~42 MB)..."
    curl -fL --retry 3 -o "$TARBALL" "$URL"
    echo "$SHA256  $TARBALL" | sha256sum -c -
    # Tarball layout: <Model-Name>/{fcn_resnet18.onnx,classes.txt,colors.txt}
    tar -xzf "$TARBALL" -C "$DEST" --strip-components=1
    rm -f "$TARBALL"
    if [ ! -f "$ONNX" ]; then
        echo "ensure_model: $ONNX missing after extraction; contents:" >&2
        ls -R "$DEST" >&2
        exit 1
    fi
fi

echo "ensure_model: building TensorRT engine (first time only; takes minutes on the Nano)..."
"$TRTEXEC" --onnx="$ONNX" --saveEngine="$ENGINE" --fp16 --workspace=256
echo "ensure_model: engine built: $ENGINE"
