cmake_minimum_required(VERSION 3.16)
# Build only main and its transitive requirements. Component-manager projects
# otherwise compile every ESP-IDF component, including unrelated networking,
# which makes a clean camera gate several times more expensive.
set(COMPONENTS main)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(s3_idf_camera)
