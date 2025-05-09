#!/bin/bash

# Starts a second bash terminal in the detected running container

# Find the ID of the running container
container_id=$(docker ps -q)

if [ -z "$container_id" ]; then
  echo "No running containers found."
  exit 1
fi

# Open a new terminal in the running Docker container
docker exec -it "$container_id" /bin/bash
