# FMI Beginner's tutorial

This repository contains the agenda and materials for the [FMI Beginner's tutorial](https://www.conftool.com/modelica2023/index.php?page=browseSessions&form_session=5) presented at the [15th International Modelica Conference 2023](https://2023.international.conference.modelica.org/).

## Part 1: Introduction to the FMI (40 min)

by [Christian Bertsch](https://github.com/chrbertsch)

[PowerPoint presentation](part1/Introduction-to-FMI.pptx) covering

- the history of the FMI
- general concepts
- best practices (units, descriptions, naming)

## Part 2: Working with FMUs (45 min) (Torsten)

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

## Part 3: Interacting with multiple FMUs (45 min)

by [Maurizio Palmieri](https://github.com/mapalmieri) and [Cláudio Gomes](https://github.com/clagms)

### Prerequisites

TODO

### Schedule

(Using Google Collab, needing a google account, using Jupyter notebooks)

IntoCPS App (GUI): Vary the stepsize (instable --> stable), Demo
Jupyter notebook (with maestro: - connecting and co-simulating multiple FMUs
- solver step size: varying from the command line, convergence study
- include import of PI Controller from Simulink

## Part 4: Closing Session (Outlook, giving hints to additional material) (Cinzia / Claudio) 10 min

by [Cinzia Bernardeschi](https://github.com/cbernardeschi) and [Cláudio Gomes](https://github.com/clagms)

- (further) application catategories
- role of FMI and Digital twin
- FMI in simulation, FMI in Monitoring, 

## Q&A (10 min)
