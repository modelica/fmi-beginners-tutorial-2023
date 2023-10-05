# FMI Beginner's tutorial

This repository contains the agenda and materials for the [FMI Beginner's tutorial](https://www.conftool.com/modelica2023/index.php?page=browseSessions&form_session=5) presented at the [15th International Modelica Conference 2023](https://2023.international.conference.modelica.org/).

## Part 1: Introduction to the FMI (40 min)

by [Christian Bertsch](https://github.com/chrbertsch)

[PowerPoint presentation](part1/Introduction-to-FMI.pptx) covering

- the motivation and history of the FMI
- general technical concepts

## Part 2: Working with FMUs (45 min)

by [Torsten Sommer](https://github.com/t-sommer)

### Prerequisites

TODO

### Schedule

Live demo + Jupyter notebook

- FMU export (from Dymola)
  - open `MODELICA_Demo.Drive` in Dymola and export it as a source code FMU

- FMU export (from Simulink with FMIKit)
  - download and install FMI Kit for Simulink
  - open `Controller.slx` with grtfmi.tlc as a source code FMU

- Validation on Website

- work with FMUs in FMPy
  - set up FMPy, create file association, create desktop shortcut
  - view the model info
  - view the documentation
  - simulate the Drive FMU and plot the result
  - create an input CSV file
  - set the stop time, parameters, output interval (loadInertia1.J = 10)
  - validate the Drive FMU 
  - compile platform binary for the Drive FMU
  - log debug info and FMI calls + short discussion of FMI calling sequence
  - generate a Python notebook from the FMU and run it

- simulate an FMU with fmusim
  - download the Reference FMUs
  - simulate the Drive model with fmusim using the input file and set a parameter
  - plot the result CSV with Excel

## Break (30 min)

## Part 3: Connecting Multiple FMUs (45 min)

by [Maurizio Palmieri](https://github.com/mapalmieri) and [Cláudio Gomes](https://github.com/clagms)

### Prerequisites

1. Optional requisites for following along in the live demo:
   1. Java (recommended version 11)
   2. Install the into-cps application. A full guide can be found in this video: https://youtu.be/HkWh-PubYQo

### Schedule

1. Live demo using the into-cps application (use the [slides](./part3/into-cps_demo.pptx) to follow along):
   1. Setup
      1. Pre-requisites: show that java is installed.
      2. Download intocps application
      3. Download coe from download manager.
      4. Launch COE from UI to show that it works.
   2. Configure a multimodel
      1. Create new project (created project can be found in [part3/example_intocps_app](part3/example_intocps_app))
      2. Locate FMUs to be used.
      3. Move them to new project folder.
      4. Create multi model
   3. Configure a co-simulation
      1. Create cosim configuration.
      2. Explain the different options.
      3. Run it.
      4. Open the results folder
   4. Exploring alternative co-simulation configurations.
      1. Create new cosim config, with an increasing step size, and show instability creeping in.
   5. Summary

(Using Google Collab, needing a google account, using Jupyter notebooks)

IntoCPS App (GUI): Vary the stepsize (instable --> stable), Demo
Jupyter notebook with maestro: 
- connecting and co-simulating multiple FMUs
- solver step size: varying from the command line, convergence study
- include import of PI Controller from Simulink

## Part 4: Closing Session (10 min)

by [Cinzia Bernardeschi](https://github.com/cbernardeschi) and [Cláudio Gomes](https://github.com/clagms)

- (further) application catategories
- role of FMI and Digital twin
- FMI in simulation, FMI in Monitoring, 

## Q&A (10 min)






# Copyright and License

Code and documentation copyright (C) 2023 the Modelica Association Project FMI.
Code released under the https://opensource.org/licenses/BSD-2-Clause[2-Clause BSD License].
Docs released under https://creativecommons.org/licenses/by-sa/4.0/[Attribution-ShareAlike 4.0 International].

