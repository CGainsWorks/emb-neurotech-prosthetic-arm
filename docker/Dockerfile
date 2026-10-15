# Everything this repository builds, in one image.
#
# Based on the official ROS 2 Humble image so the distribution does not have to
# be installed and with the ARM toolchain added so the firmware image can be
# cross-compiled in the same container. It is a big image for that reason;
# `make demo` on a machine that already has ROS is quicker.
#
# Nothing in here needs hardware. There is none.
FROM ros:humble-ros-base

# --no-install-recommends throughout: the recommended set for the ARM
# toolchain alone is several hundred megabytes of documentation.
RUN apt-get update -qq && apt-get install -y -qq --no-install-recommends \
      build-essential \
      cmake \
      gcc-arm-none-eabi \
      binutils-arm-none-eabi \
      libnewlib-arm-none-eabi \
      python3-pytest \
      make \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /arm

# The whole repository, not just the sources. The consistency checks compare
# the committed results and documents against what the code regenerates, so an
# image missing the documents would fail them for the wrong reason.
COPY . /arm

# Build everything once at image build time, so `docker compose up` runs rather
# than compiles.
RUN ./kinematics/run_tests.sh clean \
    && ./firmware/run_tests.sh clean \
    && ./firmware/build_firmware.sh clean \
    && ./scripts/build_ros.sh

# ROS needs its environment on every shell and the entry point is the only
# place to put it that survives `docker compose run`.
COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["make", "sim"]
