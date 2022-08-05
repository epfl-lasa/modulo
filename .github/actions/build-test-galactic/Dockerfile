# https://docs.github.com/en/actions/creating-actions/dockerfile-support-for-github-actions
FROM ghcr.io/aica-technology/ros2-control-libraries:galactic-devel

# Copy and set the entrypoint commands to execute when the container starts,
# explicilty invoking as ros2 user and with bash interpreter.
COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["sudo", "su", "-", "ros2", "-c", "/bin/bash", "-c", "/entrypoint.sh"]
