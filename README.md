#HackerCAD

HackerCAD is a library that I use to for generating gcode for my 5-axis PocketNC milling machine.

# Installing

## Dependencies

  - [pythonocc-core](https://github.com/tpaviot/pythonocc-core)
  - [FreeCAD](https://www.freecadweb.org/) (Optional.  Necessary for Adaptive pocketing.)
  - [jupyter lab](https://jupyter.org/) (Optional. Nice for running HackerCAD interactively.) and the following extensions:
    - jupyter-js-widgets/extension
    - jupyter-threejs/extension
  - [modelIKa](https://github.com/adamLange/modelIKa)

## Installing

After installing the above dependencies, clone HackerCAD.

    >git clone https://github.com/adamLange/HackerCAD

Make sure that the HackerCAD python3 library is on your python path.  On unix you can do that by adding the following line to your ~/.bashrc file:

    export PYTHONPATH=$PYTHONPATH:/home/path/to/where/you/cloned/the/repository/to/hackerCAD

You can check if your environment is set up properly by executing the following lines:

    cd ~
    python3
    >>import HackerCAD
    >>import area

That's it!

## Getting Started

Start up Jupyter Lab and open up the jupyter notebook hackerCAD/notebooks/example\_parts/nut/nut.ipynb

Follow along with the notebook and you should be able to generate some gcode that you can test out on the online [PocketNC Simulator](sim.pocketnc.com)
