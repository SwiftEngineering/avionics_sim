FROM ubuntu:18.04

ARG CMAKE_MAJOR=3
ARG CMAKE_MINOR=17
ARG CMAKE_PATCH=0

ENV TZ=Etc/UTC
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt update

# CLang Project Development Tools
RUN apt install -y git astyle gcovr lcov

# Versioned CMake
ENV CMAKE_MAJOR_MINOR=${CMAKE_MAJOR}.${CMAKE_MINOR}
ENV CMAKE_VERSION=${CMAKE_MAJOR}.${CMAKE_MINOR}.${CMAKE_PATCH}
RUN apt install -y curl
RUN curl "https://cmake.org/files/v${CMAKE_MAJOR_MINOR}/cmake-${CMAKE_VERSION}-Linux-x86_64.tar.gz" | tar --strip-components=1 -xz -C /usr/local

# Project Build Requirements
RUN apt install -y libboost-all-dev libignition-math4-dev 

# Project Documentation Build Requirements
RUN apt install -y python3 python3-pip
RUN apt install -y pandoc texlive-xetex texlive-fonts-recommended texlive-generic-recommended inkscape libcanberra-gtk-module
RUN python3 -m pip install -U numpy scipy matplotlib
RUN python3 -m pip install -U jupyter nbconvert jupyter_contrib_nbextensions jupyter_nbextensions_configurator

# Package Manager Cleanup
RUN apt-get -y autoremove \
	&& rm -rf /var/lib/apt/lists/* \
    && rm -rf /tmp/*