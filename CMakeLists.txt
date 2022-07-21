project(hello)
cmake_minimum_required(VERSION 3.13.4)

find_package(ZLIB)
message("ZLIB_FOUND ${ZLIB_FOUND}")
find_package(zstd)
message("zstd_FOUND ${zstd_FOUND}")
