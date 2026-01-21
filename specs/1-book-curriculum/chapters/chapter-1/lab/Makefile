.PHONY: up down run clean lint test

# Start the Docker container
up:
	docker-compose up -d --build

# Stop the Docker container
down:
	docker-compose down

# Run the lab launch file inside the container
run:
	docker-compose exec ros2_lab bash -c "source /opt/ros/humble/setup.bash && ros2 launch hello_physical_ai hello_physical_ai.launch.py"

# Clean up build artifacts
clean:
	rm -rf build/ install/ log/

# Run linting check
lint:
	flake8 src/

# Run verification tests
test:
	docker-compose exec ros2_lab bash -c "source /opt/ros/humble/setup.bash && python3 -m unittest discover src/"
