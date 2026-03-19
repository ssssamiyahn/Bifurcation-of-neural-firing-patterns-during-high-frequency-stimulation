**NEURON Simulation of Extracellular Stimulation**

This repository contains the NEURON simulation code and extracellular electric field data used in our study of neuron responses to electrical stimulation.

**Requirements**

To run the simulations, you need to install the NEURON simulation environment:

NEURON official website:
https://neuron.yale.edu/neuron/

Please follow the installation instructions provided on the official website for your operating system.


**Before Running**

Before running the simulation for the first time, you need to compile the mechanism (.mod) files.

In the directory containing the .mod files, run:

`mknrndll`

This step is required to build the ion channel and membrane mechanism libraries used by NEURON.

For more details, please refer to the official NEURON documentation.

**Repository Structure**
```
├── COMSOL_eletric_field/
│   └── (extracellular electric field data)
│
├── Simulation code/
│   ├── init.hoc
│   ├── morphology.hoc
│   ├── membrane_dynamic.hoc
│   ├── calcrxc_stim.hoc
│   ├── Istim.hoc
│   ├── defsave.hoc
│   ├── defsave_node.hoc
│   ├── *.mod
```

**File Description**
1. Extracellular Electric Field

`COMSOL_eletric_field/`
Contains the precomputed extracellular electric field data (generated from finite element simulations).
These data are used as input for the NEURON simulations.

2. NEURON Simulation Code

All simulation scripts are located in the `Simulation code/ folder`.


**Core scripts**

`init.hoc`
Main entry point of the simulation.
Loads all required files and defines global simulation parameters such as total simulation time.

`morphology.hoc`
Defines the neuronal morphology.

`membrane_dynamic.hoc`
Defines membrane electrical properties, including ion channel parameters and initial extracellular potassium concentration etc.

`calcrxc_stim.hoc`
Loads and applies extracellular electric field data from the `COMSOL_eletric_field/` directory.

`Istim.hoc`
Defines the stimulation protocol.
A 100 Hz biphasic stimulation sequence (10 s duration) is provided as an example (`Istim100Hz10sbiphasicpulse.dat`).

**Output configuration**

`defsave.hoc`

`defsave_node.hoc`

These files define the simulation outputs (e.g., membrane potentials, node-specific variables).

**Mechanism files**

`*.mod`
Define ion channel dynamics and other membrane mechanisms.
These must be compiled using `mknrndll` before running the simulation.

**How to Run**

After compiling the `.mod` files, run the simulation with:

`nrniv init.hoc`

**Notes**

Ensure that the path to the `COMSOL_eletric_field/` directory is correctly set in `calcrxc_stim.hoc`.

The provided stimulation protocol and field data can be modified to explore different conditions.

The simulation assumes extracellular stimulation based on precomputed electric fields.

**Contact**

For questions or issues, please open an issue in this repository.
