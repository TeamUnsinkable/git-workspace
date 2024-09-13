# Variables

COMPOSE_FILE = compose/hammerhead-compose.yaml
IMAGE_NAME = amra-development:dev  # Use the dev tag for the image
CONTAINER_NAME = amra-container
CPU_LIMIT = 4
MEMORY_LIMIT = 16g


# Detect OS and set variables for X11 forwarding
OS := $(shell uname -s)

ifeq ($(OS),Darwin)
    DISPLAY_VAR := host.docker.internal:0
    PRE_RUN_CMD := xhost + 127.0.0.1
else ifeq ($(OS),Linux)
    DISPLAY_VAR := $(DISPLAY)
    PRE_RUN_CMD := xhost +local:docker
else ifeq ($(OS),Windows_NT)
    DISPLAY_VAR := $(shell grep nameserver /etc/resolv.conf | awk '{print $$2}'):0
    PRE_RUN_CMD := echo "Set DISPLAY for Windows"
endif

# Build all Docker images using Docker Compose
image:
	@echo "Building Docker images..."
	docker compose -f $(COMPOSE_FILE) build

# Run the Docker Compose setup with CPU and memory limits
run:
	@echo "Running Docker Compose services..."
	$(PRE_RUN_CMD)
	docker compose -f $(COMPOSE_FILE) up -d --build
	@echo "Docker Compose services are running."

# Stop and remove all containers using Docker Compose
stop:
	@echo "Stopping all Docker containers..."
	docker compose -f $(COMPOSE_FILE) down

# Clean up Docker images
clean:
	@echo "Cleaning up Docker images..."
	docker rmi -f $(IMAGE_NAME) || true

# Debug: Enter a specific container in bash
debug:
	@echo "Entering Docker container in debug mode..."
	@if docker ps | grep -q $(CONTAINER_NAME); then \
		docker exec -it $(CONTAINER_NAME) /bin/bash; \
	else \
		echo "Container is not running. Starting container in debug mode..."; \
		$(PRE_RUN_CMD); \
		docker run -it --name $(CONTAINER_NAME) \
		--cpus="$(CPU_LIMIT)" --memory="$(MEMORY_LIMIT)" \
		-e DISPLAY=$(DISPLAY_VAR) \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v $(shell pwd):/app \
		$(IMAGE_NAME) bash; \
	fi

# Rebuild and run Docker containers
container: image run
