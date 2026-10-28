#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /opt/svea/jazzy/setup.bash
cd /svea_ws
colcon --log-base /tmp/pom-log build --symlink-install --build-base /tmp/pom-build --install-base /tmp/pom-install --packages-up-to peaceofmine_operator >/tmp/pom-build.log 2>&1 || { cat /tmp/pom-build.log; exit 1; }
source /tmp/pom-install/setup.bash
colcon --log-base /tmp/pom-test-log test --build-base /tmp/pom-build --install-base /tmp/pom-install --packages-select peaceofmine_operator peaceofmine_interfaces --event-handlers console_direct+
colcon --log-base /tmp/pom-result-log test-result --test-result-base /tmp/pom-build --verbose
for entry in operator.launch.xml operator_sim.launch.py; do
  set +e
  timeout --signal=INT --kill-after=8s 12s ros2 launch peaceofmine_operator "$entry" is_sim:=true use_cameras:=false use_joy:=false port:=18089 > /tmp/pom-launch.log 2>&1
  result=$?
  set -e
  cat /tmp/pom-launch.log
  if [ "$result" != 124 ] || grep -E 'Traceback|process has died|Caught exception' /tmp/pom-launch.log; then exit 1; fi
done
