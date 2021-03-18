# Jupyter Notebook Requirements

The following needs to be installed for jupyter notebook support in the px4 container, that is until the container is updated

```
sudo apt update

sudo apt install -y python3-pip pandoc texlive-xetex texlive-fonts-recommended texlive-generic-recommended inkscape libcanberra-gtk-module
```

## python3
```
python3 -m pip install -U jupyter nbconvert jupyter_contrib_nbextensions jupyter_nbextensions_configurator
```

## python2
```
sudo pip install -U jupyter nbconvert jupyter_contrib_nbextensions jupyter_nbextensions_configurator
```
