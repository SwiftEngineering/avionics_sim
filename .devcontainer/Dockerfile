# From Ubuntu 18.04: pinned by digest
FROM ubuntu@sha256:3b8692dc4474d4f6043fae285676699361792ce1828e22b1b57367b5c05457e3

ARG CMAKE_MAJOR=3
ARG CMAKE_MINOR=17
ARG CMAKE_PATCH=0

ARG TZ=Etc/UTC
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# CLang Project Development Tools
RUN apt update && apt install -y \
    git=1:2.17.1-1ubuntu0.8 \
    astyle=3.1-1ubuntu2 \
    gcovr=3.4-1 \
    lcov=1.13-3 \
    clang=1:6.0-41~exp5~ubuntu1 \
    clang-tidy=1:6.0-41~exp5~ubuntu1

# Versioned CMake
ENV CMAKE_MAJOR_MINOR=${CMAKE_MAJOR}.${CMAKE_MINOR}
ENV CMAKE_VERSION=${CMAKE_MAJOR}.${CMAKE_MINOR}.${CMAKE_PATCH}
RUN apt install -y curl
RUN curl "https://cmake.org/files/v${CMAKE_MAJOR_MINOR}/cmake-${CMAKE_VERSION}-Linux-x86_64.tar.gz" | tar --strip-components=1 -xz -C /usr/local

# Project Build Requirements
RUN apt install -y \
    libboost-all-dev=1.65.1.0ubuntu1 \
    libignition-math4-dev=4.0.0+dfsg1-4 \
# Project Documentation Build Requirements
    python3=3.6.7-1~18.04 \
    python3-pip=9.0.1-2.3~ubuntu1.18.04.5 \
    pandoc=1.19.2.4~dfsg-1build4 \
    texlive-xetex=2017.20180305-1 \
    texlive-fonts-recommended=2017.20180305-1  \
    texlive-generic-recommended=2017.20180305-1  \
    inkscape=0.92.3-1 \
    libcanberra-gtk-module=0.30-5ubuntu1

RUN python3 -m pip install -U \
    numpy==1.19.5 \
    scipy==1.5.4 \
    matplotlib==3.3.4 \
    jupyter==1.0.0 \
    nbconvert==6.0.7 \
    jupyter_contrib_nbextensions==0.5.1 \
    jupyter_nbextensions_configurator==0.4.1 \
    cpplint==1.5.5

# Add A Developer User
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID ${USERNAME} && \
    useradd --uid $USER_UID --gid $USER_GID -m ${USERNAME} && \
    apt install -y sudo && \
    echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Package Manager Cleanup
RUN apt-get -y autoremove \
	&& rm -rf /var/lib/apt/lists/* \
    && rm -rf /tmp/*