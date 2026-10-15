# emb-neurotech-prosthetic-arm
#
# `make demo` goes from a clean checkout to every number this repository
# quotes. It needs a C++ compiler, Python and ROS 2 Humble and about four
# minutes. `docker compose up` does the same thing in a container.
#
# Nothing here needs hardware, because there is no hardware. See STATUS.md.

PYTHON ?= python3
JOBS ?= $(shell (command -v nproc >/dev/null && nproc) || echo 4)

.DEFAULT_GOAL := help
.PHONY: help kinematics firmware firmware-image hardware model urdf ros test \
        sim sim-lossy demo check docs clean clean-all docker-build docker-up \
        docker-test docker-down

help:
	@echo "make kinematics      build and test the C++ kinematics library"
	@echo "make firmware        build and test the firmware's portable core"
	@echo "make firmware-image  cross-compile the STM32H743 image and verify it"
	@echo "make hardware        evaluate the front end and power models"
	@echo "make model           regenerate the model JSON and the URDF from the C++"
	@echo "make ros             build the ROS 2 workspace"
	@echo "make test            every test suite in the repository"
	@echo "make sim             the end-to-end simulation, acquisition to actuation"
	@echo "make sim-lossy       the same run with 15 percent of frames dropped"
	@echo "make check           the generated files and documented numbers still agree"
	@echo "make demo            everything above, in order, from nothing"
	@echo "make docker-up       the same thing in Docker"
	@echo "make clean           remove build directories"

# --- the pieces, each buildable on its own ---------------------------------

kinematics:
	./kinematics/run_tests.sh

firmware:
	./firmware/run_tests.sh

# Needs arm-none-eabi-gcc. The image has never been flashed.
firmware-image:
	./firmware/build_firmware.sh
	./firmware/verify_image.sh

hardware:
	$(PYTHON) hardware/semg-servo-board/analysis/report.py
	$(PYTHON) -m pytest hardware/semg-servo-board/analysis -q

# The URDF and the model JSON are generated from kinematics/src/model.cpp.
# Anything that changes the mechanism goes through here.
model: kinematics
	./kinematics/build/dump_model > ros2_ws/src/arm_description/config/a1_arm_model.json
	$(PYTHON) scripts/generate_urdf.py
	$(PYTHON) -m pytest ros2_ws/src/arm_description/test -q

urdf: model

ros:
	./scripts/build_ros.sh

test:
	./scripts/build_ros.sh test
	$(PYTHON) -m pytest hardware/semg-servo-board/analysis ros2_ws/src/arm_description/test -q

# --- the run that produces the numbers --------------------------------------

sim: ros
	@mkdir -p analysis/sim
	@bash -c 'set +u; . install/setup.bash; \
	  ros2 run arm_bringup sim_demo --output analysis/sim/control_loop_sim.json'
	$(PYTHON) scripts/check_sim_result.py analysis/sim/control_loop_sim.json

# The same run with a link that drops frames, which is what a marginal cable
# does and what the host timeout logic exists for.
sim-lossy: ros
	@mkdir -p analysis/sim
	@bash -c 'set +u; . install/setup.bash; \
	  ros2 run arm_bringup sim_demo --frame-loss 0.15 \
	    --output analysis/sim/control_loop_lossy.json'

# --- consistency -------------------------------------------------------------

# Everything that can drift: the description against the model it came from,
# the hardware documents against the models they quote and the shell scripts
# against bash.
check: model
	$(PYTHON) scripts/generate_urdf.py --check
	$(PYTHON) hardware/semg-servo-board/analysis/report.py --quiet
	$(PYTHON) scripts/check_docs.py
	@for script in scripts/*.sh firmware/*.sh kinematics/*.sh; do \
	  bash -n "$$script" && echo "ok: $$script"; \
	done

docs: check

# --- everything ---------------------------------------------------------------

demo: kinematics firmware hardware model ros sim check
	@echo
	@echo "Done. The numbers are in analysis/ and hardware/semg-servo-board/analysis/."
	@echo "Nothing in them was measured on hardware, because there is none."

# --- docker -------------------------------------------------------------------

# Compose reuses an image with a matching tag rather than noticing the
# Dockerfile has moved on, so every docker target builds first.
docker-build:
	docker compose build

docker-up: docker-build
	docker compose up

docker-test: docker-build
	docker compose run --rm test

docker-down:
	docker compose down -v

# --- tidying ------------------------------------------------------------------

clean:
	rm -rf build install log
	rm -rf kinematics/build firmware/build firmware/build-arm
	find . -name __pycache__ -type d -prune -exec rm -rf {} +
	find . -name .pytest_cache -type d -prune -exec rm -rf {} +

# Also removes the generated results. `make demo` regenerates them.
clean-all: clean
	rm -rf analysis/sim
