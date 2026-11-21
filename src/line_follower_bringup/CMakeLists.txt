cmake_minimum_required(VERSION 3.8)
project(line_follower_bringup)

find_package(ament_cmake REQUIRED)

# worlds/ and models/ are the provided originals, installed byte-for-byte (R-05).
# They live inside this package so the submitted package archive is self-contained.
# SHA256SUMS guards them (line_follower_tests/test_asset_integrity.py).
install(DIRECTORY launch config worlds models DESTINATION share/${PROJECT_NAME})

ament_package()
