# To build:
  - Install:
```
sudo apt install python3-sphinx python3-sphinx-rtd-theme
```
  - If you are building on an NRP install, you will have to remove all references to python2 paths:
```
unset PYTHONPATH
```
  - Build html files:
```
make html
```
  - The files will be located in ./_build

