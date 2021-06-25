#!/bin/sh

# Uninstall old versions
sudo apt-get remove docker docker-engine docker.io containerd runc

# Set up the repository
sudo apt-get update
sudo apt-get install \
    apt-transport-https \
    ca-certificates \
    curl \
    gnupg \
    lsb-release
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
echo \
  "deb [arch=amd64 signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install Docker Engine
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io

# Manage Docker as a non-root user
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker

# Verify that Docker Engine is installed correctly by running the hello-world image.
docker run hello-world

echo "docker installed successfully! Reboot so that your group membership is re-evaluated.(log out and log back in might not work.)"


# reference
## https://docs.docker.com/engine/install/ubuntu/
## https://docs.docker.com/engine/install/linux-postinstall/
