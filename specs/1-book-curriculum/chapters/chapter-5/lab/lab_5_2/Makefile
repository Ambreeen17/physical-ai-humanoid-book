# Makefile for lab_5_2

.PHONY: build run stop clean test

build:
	docker-compose build

run: build
	docker-compose up -d
	docker-compose exec lab_5_2 bash

stop:
	docker-compose down

clean:
	docker-compose down -v
	rm -rf build/ install/ log/

test:
	docker-compose exec lab_5_2 bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && python3 -m pytest tests/"

logs:
	docker-compose logs -f lab_5_2
