SERVICE=ros

build:
	docker-compose build

run:
	docker-compose up -d && docker ps | grep robot-arm-container && docker exec -it robot-arm-container bash || echo "Container is not running."

stop:
	docker-compose down

remove:
	docker-compose down -v

clean:
	docker system prune -af
