.PHONY: clean build help source

help:
	@echo "Available commands:"
	@echo "  make clean  - Remove all build artifacts (build/, install/, log/)"
	@echo "  make build  - Build all packages"
	@echo "  make source - Show source commands"

clean:
	rm -rf build/ install/ log/

build:
	colcon build --symlink-install --packages-select teleop_interfaces
	colcon build --symlink-install --packages-select amazing_hand lucidgloves_ros2

source:
	@echo "source /opt/ros/humble/setup.bash"
	@echo "source install/setup.bash"
