.PHONY: clean build help

help:
	@echo "Available commands:"
	@echo "  make clean  - Remove all build artifacts (build/, install/, log/)"
	@echo "  make build  - Build all packages"

clean:
	@echo "Cleaning build artifacts..."
	rm -rf build/ install/ log/
	@echo "Clean complete!"

build:
	colcon build --symlink-install
