FROM swiftx/ubuntu-clang-base:v9

RUN apt update
RUN apt install -y libboost-all-dev libignition-math4-dev python3 python3-pip
RUN apt install -y pandoc texlive-xetex texlive-fonts-recommended texlive-generic-recommended inkscape libcanberra-gtk-module
RUN python3 -m pip install -U numpy scipy matplotlib
RUN python3 -m pip install -U jupyter nbconvert jupyter_contrib_nbextensions jupyter_nbextensions_configurator