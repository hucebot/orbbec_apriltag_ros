REGISTRY = registry.gitlab.inria.fr
IMAGE_NAME = hucebot/code/apriltag_pose
TAG = latest

.PHONY: login build-dev run build-dep deploy stop logs clean

login:
	docker login $(REGISTRY)

build-dev:
	docker compose -f docker-compose.dev.yaml build

run:
	xhost +local:docker
	docker compose -f docker-compose.dev.yaml up -d
	docker exec -it apriltag_dev /bin/bash -c "source /entrypoint.sh && /bin/bash"

build-dep:
	docker compose -f docker-compose.yaml build
	docker tag apriltag_pose:latest $(REGISTRY)/$(IMAGE_NAME):$(TAG)

deploy:
	docker compose -f docker-compose.yaml up -d

logs:
	docker compose -f docker-compose.yaml logs -f

stop:
	docker compose -f docker-compose.dev.yaml stop
	docker compose -f docker-compose.yaml stop

clean:
	docker compose -f docker-compose.dev.yaml down --remove-orphans
	docker compose -f docker-compose.yaml down --remove-orphans
	rm -rf build/ install/ log/