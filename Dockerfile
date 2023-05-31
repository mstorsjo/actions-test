FROM ubuntu:18.04

RUN apt-get update -qq && apt-get install -qqy --no-install-recommends \
    git wget bzip2 file unzip libtool pkg-config cmake build-essential \
    automake yasm gettext autopoint vim-tiny python3 python3-distutils \
    ninja-build ca-certificates curl less zip && \
    apt-get clean -y && \
    rm -rf /var/lib/apt/lists/*

# Manually install a newer version of CMake; this is needed since building
# LLVM requires CMake 3.13.4, while Ubuntu 18.04 ships with 3.10.2. If
# updating to a newer distribution, this can be dropped.
RUN cd /opt && \
    curl -LO https://github.com/Kitware/CMake/releases/download/v3.20.1/cmake-3.20.1-Linux-$(uname -m).tar.gz && \
    tar -zxf cmake-*.tar.gz && \
    rm cmake-*.tar.gz && \
    mv cmake-* cmake
ENV PATH=/opt/cmake/bin:$PATH

# Install a newer version of Git; the version of Git in Ubuntu 18.04 is
# said to have issues with submodules, see e.g.
# https://github.com/mstorsjo/llvm-mingw/pull/210#issuecomment-870104971 and
# https://github.com/mstorsjo/llvm-mingw/pull/210#issuecomment-873486503.
# This isn't needed for building LLVM itself, but makes the built Docker
# image more useful for use as image for building other projects. If updating
# to a newer distribution, this can be dropped.
RUN apt-get update -qq && \
    apt-get install -qqy --no-install-recommends software-properties-common && \
    add-apt-repository ppa:git-core/ppa && \
    apt-get update -qq && \
    apt-get upgrade -qqy git && \
    apt-get clean -y && \
    rm -rf /var/lib/apt/lists/*


