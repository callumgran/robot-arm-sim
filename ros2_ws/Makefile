.PHONY: build clean run

build:
	rm -rf build install log
	colcon build --symlink-install
	@echo "Build complete. Sourcing setup.bash..."
	bash -c "source install/setup.bash"

clean:
	rm -rf build install log

run:
	bash -c "source install/setup.bash && ros2 launch robot_arm gazebo.launch.py"

