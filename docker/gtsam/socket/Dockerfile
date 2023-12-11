FROM ubuntu:22.04
ENV DEBIAN_FRONTEND noninteractive

# install dependencies via apt
ENV DEBCONF_NOWARNINGS yes
RUN set -x && \
  apt-get update -y -qq && \
  apt-get upgrade -y -qq --no-install-recommends && \
  : "basic dependencies" && \
  apt-get install -y -qq \
    build-essential \
    pkg-config \
    cmake \
    git \
    wget \
    curl \
    tar \
    unzip && \
  : "g2o dependencies" && \
  apt-get install -y -qq \
    libatlas-base-dev \
    libsuitesparse-dev \
    libglew-dev && \
  : "gtsam dependencies" && \
  apt-get install -y -qq \
    libboost-all-dev \
    libtbb-dev && \
  : "OpenCV dependencies" && \
  apt-get install -y -qq \
    libjpeg-dev \
    libpng++-dev \
    libtiff-dev \
    libopenexr-dev \
    libwebp-dev \
    ffmpeg \
    libavcodec-dev \
    libavformat-dev \
    libavutil-dev \
    libswscale-dev && \
  : "backward-cpp dependencies" && \
  apt install -y -qq binutils-dev && \
  : "other dependencies" && \
  apt-get install -y -qq \
    libyaml-cpp-dev \
    sqlite3 \
    libsqlite3-dev && \
  : "remove cache" && \
  apt-get autoremove -y -qq && \
  rm -rf /var/lib/apt/lists/*

ARG CMAKE_INSTALL_PREFIX=/usr/local
ARG NUM_THREADS=1

ENV CPATH=${CMAKE_INSTALL_PREFIX}/include:${CPATH}
ENV C_INCLUDE_PATH=${CMAKE_INSTALL_PREFIX}/include:${C_INCLUDE_PATH}
ENV CPLUS_INCLUDE_PATH=${CMAKE_INSTALL_PREFIX}/include:${CPLUS_INCLUDE_PATH}
ENV LIBRARY_PATH=${CMAKE_INSTALL_PREFIX}/lib:${LIBRARY_PATH}
ENV LD_LIBRARY_PATH=${CMAKE_INSTALL_PREFIX}/lib:${LD_LIBRARY_PATH}

# Eigen
ARG EIGEN3_VERSION=3.3.7
WORKDIR /tmp
RUN set -x && \
  wget -q https://gitlab.com/libeigen/eigen/-/archive/${EIGEN3_VERSION}/eigen-${EIGEN3_VERSION}.tar.bz2 && \
  tar xf eigen-${EIGEN3_VERSION}.tar.bz2 && \
  rm -rf eigen-${EIGEN3_VERSION}.tar.bz2 && \
  cd eigen-${EIGEN3_VERSION} && \
  mkdir -p build && \
  cd build && \
  cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} \
    .. && \
  make -j${NUM_THREADS} && \
  make install && \
  cd /tmp && \
  rm -rf *
ENV Eigen3_DIR=${CMAKE_INSTALL_PREFIX}/share/eigen3/cmake

# g2o
ARG G2O_COMMIT=20230223_git
WORKDIR /tmp
RUN set -x && \
  git clone https://github.com/RainerKuemmerle/g2o.git && \
  cd g2o && \
  git checkout ${G2O_COMMIT} && \
  mkdir -p build && \
  cd build && \
  cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_UNITTESTS=OFF \
    -DG2O_USE_CHOLMOD=OFF \
    -DG2O_USE_CSPARSE=ON \
    -DG2O_USE_OPENGL=OFF \
    -DG2O_USE_OPENMP=OFF \
    -DG2O_BUILD_APPS=OFF \
    -DG2O_BUILD_EXAMPLES=OFF \
    -DG2O_BUILD_LINKED_APPS=OFF \
    .. && \
  make -j${NUM_THREADS} && \
  make install && \
  cd /tmp && \
  rm -rf *
ENV g2o_DIR=${CMAKE_INSTALL_PREFIX}/lib/cmake/g2o

# gtsam
ARG GTSAM_TAG=4.2a7
WORKDIR /tmp
RUN set -x && \
  git clone https://github.com/borglab/gtsam.git && \
  cd gtsam && \
  git checkout ${GTSAM_TAG} && \
  mkdir -p build && \
  cd build && \
  cmake \
    -DGTSAM_USE_SYSTEM_EIGEN=ON \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    -DGTSAM_BUILD_UNSTABLE=OFF \
    .. && \
  make -j${NUM_THREADS} && \
  make install && \
  cd /tmp && \
  rm -rf *
ENV gtsam_DIR=${CMAKE_INSTALL_PREFIX}/lib/cmake/gtsam

# OpenCV
ARG OPENCV_VERSION=4.7.0
WORKDIR /tmp
RUN set -x && \
  wget -q https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip && \
  unzip -q ${OPENCV_VERSION}.zip && \
  rm -rf ${OPENCV_VERSION}.zip && \
  cd opencv-${OPENCV_VERSION} && \
  mkdir -p build && \
  cd build && \
  cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} \
    -DBUILD_DOCS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_JASPER=OFF \
    -DBUILD_OPENEXR=OFF \
    -DBUILD_PERF_TESTS=OFF \
    -DBUILD_TESTS=OFF \
    -DBUILD_PROTOBUF=OFF \
    -DBUILD_opencv_apps=OFF \
    -DBUILD_opencv_dnn=OFF \
    -DBUILD_opencv_ml=OFF \
    -DBUILD_opencv_python_bindings_generator=OFF \
    -DENABLE_CXX11=ON \
    -DENABLE_FAST_MATH=ON \
    -DWITH_EIGEN=ON \
    -DWITH_FFMPEG=ON \
    -DWITH_TBB=ON \
    -DWITH_OPENMP=ON \
    .. && \
  make -j${NUM_THREADS} && \
  make install && \
  cd /tmp && \
  rm -rf *
ENV OpenCV_DIR=${CMAKE_INSTALL_PREFIX}/lib/cmake/opencv4

# socket.io-client-cpp
ARG SIOCLIENT_COMMIT=ff6ef08e45c594e33aa6bc19ebdd07954914efe0
WORKDIR /tmp
RUN set -x && \
  git clone https://github.com/shinsumicco/socket.io-client-cpp.git && \
  cd socket.io-client-cpp && \
  git checkout ${SIOCLIENT_COMMIT} && \
  git submodule init && \
  git submodule update && \
  mkdir -p build && \
  cd build && \
  cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} \
    -DBUILD_UNIT_TESTS=OFF \
    .. && \
  make -j${NUM_THREADS} && \
  make install && \
  cd /tmp && \
  rm -rf *
ENV sioclient_DIR=${CMAKE_INSTALL_PREFIX}/lib/cmake/sioclient

# protobuf
WORKDIR /tmp
RUN set -x && \
  apt-get update -y -qq && \
  apt-get install -y -qq autogen libtool && \
  wget -q https://github.com/google/protobuf/archive/v3.6.1.tar.gz && \
  tar xf v3.6.1.tar.gz && \
  cd protobuf-3.6.1 && \
  ./autogen.sh && \
  ./configure --prefix=${CMAKE_INSTALL_PREFIX} --enable-static=no && \
  make -j${NUM_THREADS} && \
  make install && \
  cd /tmp && \
  rm -rf * && \
  apt-get purge -y -qq autogen libtool && \
  apt-get autoremove -y -qq && \
  rm -rf /var/lib/apt/lists/*

# backward-cpp
ARG BACKWARD_CPP_COMMIT=5ffb2c879ebdbea3bdb8477c671e32b1c984beaa
WORKDIR /tmp
RUN set -x && \
  git clone https://github.com/bombela/backward-cpp.git && \
  cd backward-cpp && \
  git checkout ${BACKWARD_CPP_COMMIT} && \
  mkdir -p build && \
  cd build && \
  cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} \
    .. && \
  make -j${NUM_THREADS} && \
  make install && \
  cd /tmp && \
  rm -rf *

# stella_vslam
COPY . /stella_vslam/
WORKDIR /stella_vslam/
RUN set -x && \
  mkdir -p build && \
  cd build && \
  cmake \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    .. && \
  make -j${NUM_THREADS} && \
  make install && \
  rm -rf CMakeCache.txt CMakeFiles Makefile cmake_install.cmake example src && \
  chmod -R 777 ./*

ARG SOCKET_PUBLISHER_COMMIT=e2aaf66b27a6477b478bc8e86992907f09f29188
WORKDIR /tmp
RUN set -x && \
  git clone https://github.com/stella-cv/socket_publisher.git && \
  cd socket_publisher && \
  git checkout -q ${SOCKET_PUBLISHER_COMMIT} && \
  git submodule update --init --recursive && \
  mkdir -p build && \
  cd build && \
  cmake \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} \
    .. && \
  make -j${NUM_THREADS} && \
  make install && \
  cd /tmp && \
  rm -rf *

ARG EXAMPLES_COMMIT=b86009e2ed004f275c5bacd48e7c641a722914e1
WORKDIR /
RUN set -x && \
  git clone https://github.com/stella-cv/stella_vslam_examples.git && \
  cd stella_vslam_examples && \
  git checkout -q ${EXAMPLES_COMMIT} && \
  git submodule update --init --recursive && \
  mkdir -p build && \
  cd build && \
  cmake \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} \
    -DUSE_STACK_TRACE_LOGGER=ON \
    .. && \
  make -j${NUM_THREADS}

WORKDIR /stella_vslam_examples/build/
ENTRYPOINT ["/bin/bash"]
