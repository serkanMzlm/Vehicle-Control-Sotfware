# ANSI color codes
BLACK := \033[0;31
RED := \033[0;31m
GREEN := \033[0;32m
YELLOW := \033[0;33m
BLUE := \033[0;34m
PURPLE := 033[0;35m
CYAN := 033[0;36m
WHITE := 033[0;37m
RESET := \033[0m

# Target directories
BUILD_DIR := src/drivers/build
INSTALL_DIR := src/include

# CMake and Make options
CMAKE_OPTIONS := -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$(abspath $(INSTALL_DIR))
MAKE_OPTIONS := --no-print-directory

.PHONY: all
all: ${BUILD_DIR}/Makefile
	@echo "$(GREEN)Building the library of project  in src/drivers...$(RESET)"
	@$(MAKE) $(MAKE_OPTIONS) -C $(BUILD_DIR)
	@echo "$(GREEN)Installing the library...$(RESET)"
	@$(MAKE) $(MAKE_OPTIONS) -C $(BUILD_DIR) install

$(BUILD_DIR)/Makefile:
	@echo "$(BLUE)Creating build directory in src/drivers if not exists...$(RESET)"
	@mkdir -p ${BUILD_DIR}
	@echo "$(BLUE)Configuring the project with CMake in src/drivers...$(RESET)"
	@cd $(BUILD_DIR) &&	cmake $(CMAKE_OPTIONS) .. || { echo "$(RED)CMake configuration failed!$(RESET)"; exit 1; }


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
	@echo "Installing the project in src/drivers..."
	@$(MAKE) $(MAKE_OPTIONS) -C $(BUILD_DIR) install
