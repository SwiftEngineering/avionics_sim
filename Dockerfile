FROM swiftx/ubuntu-clang-base:v9

RUN apt update
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN apt install -y libignition-math4-dev

RUN apt install -y libboost-all-dev
