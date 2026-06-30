#!/usr/bin/env bash
set -euo pipefail

echo "=== JetRacer Docker Setup ==="

# Check architecture
ARCH=$(dpkg --print-architecture)
if [ "$ARCH" != "arm64" ]; then
    echo "Error: This script supports arm64 only."
    exit 1
fi

# Check Ubuntu release
CODENAME=$(lsb_release -cs)
if [ "$CODENAME" != "bionic" ]; then
    echo "Warning: This script was tested on Ubuntu 18.04 (bionic)."
fi

echo "Installing Docker Engine (docker.io)..."
sudo apt-get update
sudo apt-get install -y docker.io curl gnupg lsb-release

echo "Adding Docker repository if needed..."
if ! grep -Rq "download.docker.com/linux/ubuntu" /etc/apt/sources.list /etc/apt/sources.list.d 2>/dev/null; then

    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -

    echo "deb [arch=arm64] https://download.docker.com/linux/ubuntu ${CODENAME} stable" | \
        sudo tee /etc/apt/sources.list.d/docker.list >/dev/null

    sudo apt-get update
fi

echo "Installing Docker Compose Plugin..."
sudo apt-get install -y docker-compose-plugin

echo "Adding current user to docker group..."
sudo usermod -aG docker "$USER"

echo
echo "===== Installation Complete ====="
docker --version

if docker compose version; then
    echo "Docker Compose plugin installed successfully."
else
    echo "ERROR: Docker Compose plugin installation failed."
    exit 1
fi

echo
echo "Please log out and back in (or reboot) for docker group changes to take effect."