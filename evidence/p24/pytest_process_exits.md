# python3 -m pytest test/test_a7_process_exits.py (bringup package, no ROS graph): launch_guard records before it asserts; a synthetic -11 in a sidecar is derived into a rate
# repository commit a918f01
platform linux -- Python 3.12.3, pytest-7.4.4, pluggy-1.4.0 -- /usr/bin/python3
rootdir: /home/slash/openamr_ws/src/openamr-upperbody-sw/ros2/src/openamrobot_upperbody_bringup
plugins: ament-pep257-0.17.5, ament-flake8-0.17.5, launch-testing-3.4.11, ament-copyright-0.17.5, ament-xmllint-0.17.5, ament-lint-0.17.5, launch-testing-ros-0.26.12, colcon-core-0.21.3, cov-4.1.0, typeguard-4.1.5
collecting ... collected 5 items

test/test_a7_process_exits.py::test_the_exit_codes_of_every_process_are_recorded_in_name_order PASSED [ 20%]
test/test_a7_process_exits.py::test_the_codes_are_recorded_before_the_check_fails_and_the_check_still_fails PASSED [ 40%]
test/test_a7_process_exits.py::test_a_launch_with_nothing_to_record_is_checked_as_before PASSED [ 60%]
test/test_a7_process_exits.py::test_the_a7_report_lists_the_exit_codes_per_launch_and_skips_a_launch_that_recorded_none PASSED [ 80%]
test/test_a7_process_exits.py::test_canary_a_synthetic_minus_11_in_a_sidecar_derives_a_rate_the_backing_check_backs PASSED [100%]

============================== 5 passed in 0.09s ===============================
