# ANSI color codes
BLACK := \033[0;31
RED := \033[0;31m
GREEN := \033[0;32m
YELLOW := \033[0;33m
BLUE := \033[0;34m
PURPLE := \033[0;35m
CYAN := \033[0;36m
WHITE := 033[0;37m
RESET := \033[0m

# Target directories
BUILD_DIR := src/drivers/build
INSTALL_DIR := src/include

# CMake and Make options
CMAKE_OPTIONS := -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$(abspath $(INSTALL_DIR))
MAKE_OPTIONS := --no-print-directory

# The file containing ROS2 packages
TEXT_FILE := packages.txt

# Command to read package names from the text file, ignoring lines starting with *
READ_PACKAGES_CMD := grep -v '^*' $(TEXT_FILE)

FAILED_PACKAGES_FILE = failed_packages.txt
SUCCESSFUL_PACKAGES_FILE = successful_packages.txt
VAR := $(shell date)

.PHONY: all
all: update_bashrc ${BUILD_DIR}/Makefile
	@echo "$(GREEN)Building the library of project  in src/drivers...$(RESET)"
	@$(MAKE) $(MAKE_OPTIONS) -C $(BUILD_DIR)
	@echo "$(GREEN)Installing the library...$(RESET)"
	@$(MAKE) $(MAKE_OPTIONS) -C $(BUILD_DIR) install

$(BUILD_DIR)/Makefile:
	@echo "$(BLUE)Creating build directory in src/drivers if not exists...$(RESET)"
	@mkdir -p ${BUILD_DIR}
	@echo "$(BLUE)Configuring the project with CMake in src/drivers...$(RESET)"
	@cd $(BUILD_DIR) &&	cmake $(CMAKE_OPTIONS) .. || { echo "$(RED)CMake configuration failed!$(RESET)"; exit 1; }

.PHONY: update_bashrc
update_bashrc:
	@CURRENT_DIR=$$(pwd); \
	MODEL_DIR=$$(pwd)/Tools/simulation/models; \
	if grep -q "export VEHICLE_CONTRAL_SOFTWARE=" ~/.bashrc; then \
		sed -i "s|^export VEHICLE_CONTRAL_SOFTWARE=.*|export VEHICLE_CONTRAL_SOFTWARE=$$CURRENT_DIR|" ~/.bashrc; \
	else \
		echo "export VEHICLE_CONTRAL_SOFTWARE=$$CURRENT_DIR" >> ~/.bashrc; \
	fi; \
	if grep -q "export GZ_SIM_RESOURCE_PATH=.*$$MODEL_DIR" ~/.bashrc; then \
		echo "$(YELLOW) GZ_SIM_RESOURCE_PATH already contains $$MODEL_DIR $(RESET)"; \
	elif grep -q "export GZ_SIM_RESOURCE_PATH=" ~/.bashrc; then \
		sed -i "s|^export GZ_SIM_RESOURCE_PATH=.*|export GZ_SIM_RESOURCE_PATH=$${GZ_SIM_RESOURCE_PATH}:$$MODEL_DIR|" ~/.bashrc; \
	else \
		echo "export GZ_SIM_RESOURCE_PATH=$${GZ_SIM_RESOURCE_PATH}:$$MODEL_DIR" >> ~/.bashrc; \
	fi
	@echo "$(GREEN)Updating .bashrc with current directory...$(RESET)"
	@. ~/.bashrc

.PHONY: all_clean
all_clean: clean ros_clean
	@echo "$(RED)Clean complete.$(RESET)"
.PHONY: clean
clean:
	@echo "$(RED)Cleaning up src/drivers...$(RESET)"
	@rm -rf $(BUILD_DIR)
	@echo "$(RED)Clean complete.$(RESET)"


.PHONY: rebuild
rebuild: clean all

.PHONY: build
build:
	@echo "$(GREEN)Building the project in src/drivers...$(RESET)"
	@$(MAKE) $(MAKE_OPTIONS) -C $(BUILD_DIR)

.PHONY: install
install:
	@echo "$(GREEN)Installing the project in src/drivers...$(RESET)"
	@$(MAKE) $(MAKE_OPTIONS) -C $(BUILD_DIR) install

.PHONY: ros_build
ros_build:
	@echo "$(PURPLE)ROS2 packages build process has started...$(RESET)"
	@rm -f $(FAILED_PACKAGES_FILE) $(SUCCESSFUL_PACKAGES_FILE)
	@$(READ_PACKAGES_CMD) | while read -r package; do \
		echo "$(CYAN)Package: $$package$(RESET)"; \
		if colcon build --packages-select $$package; then \
			echo $$package >> $(SUCCESSFUL_PACKAGES_FILE); \
		else \
			echo "$(RED)Error: Build failed for package $$package$(RESET)"; \
			echo $$package >> $(FAILED_PACKAGES_FILE); \
		fi \
	done
	@bash -c 'echo "Press Enter to continue..."; read'
	@clear
	@echo "$(BLUE)================BUILD ANALYSIS================$(RESET)"; \
	if [ -f $(FAILED_PACKAGES_FILE) ]; then \
		FAILED_COUNT=$$(wc -l < $(FAILED_PACKAGES_FILE)); \
		printf "$(RED)Number of failed packages: %d$(RESET)\n" $$FAILED_COUNT; \
		cat $(FAILED_PACKAGES_FILE); \
		printf "\n"; \
	fi; \
	if [ -f $(SUCCESSFUL_PACKAGES_FILE) ]; then \
		SUCCESSFUL_COUNT=$$(wc -l < $(SUCCESSFUL_PACKAGES_FILE)); \
		printf "$(GREEN)successful packages: %d$(RESET)\n" $$SUCCESSFUL_COUNT; \
		cat $(SUCCESSFUL_PACKAGES_FILE); \
		printf "\n"; \
	fi

.PHONY: ros_clean
ros_clean:
	@echo "$(RED)log, build, install directories deleted$(RESET)"
	@rm -rf ./log ./build ./install