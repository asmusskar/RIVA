# RIVA
Road-profile Inversion based on in-Vehicle Accelerations (RIVA) is a MATLAB-package for road profile inversion from in-vehicle accelerometer readings.

RIVA is based on matching measured vertical accelerations acquired over a specific road stretch with accelerations simulated by a calibrated quarter-car model. The matching is done with a PID control algorithm (see Figure 1), considering the road profile as an unknown 'input' and the measured accelerations as 'target' signal. In this approach, the physics of the quarter-car model governs the 'transfer function' relating input to output. A detailed description of the proposed methodology including model verification and validation of RIVA is presented in [Skar and Levenberg (2023)]. 

<div>
<img src="pics/RIVA_f2.png" width="90%">
</div>
<p>
 <b>Figure 1:</b> Block diagram of the closed feedback loop PID controller utilised in RIVA. 
</p>

The components of the RIVA software is briefly described below:
* `main.m` - main script for defining vehicle, importing accelerometer data, and post-processing of the results. 

* `qcar_acc_ms_tvar.m` - script for simulating a syntehtic acceleration trace based on a known vehicle and road profile.

* `PID_profile_inv_tvar.m` - script for road profile inversion based on measured accelerations.

*  `PID_profile_out_tvar.m` - script for plotting final optimal solution after the optimization process.

*  `pid_control.mat` - road profile data.

## Synthetic example and verification
The example script (i.e., 'main.m' script) provided is named 'RIVA_insilico.m'. This script provides a syntetic verification of the RIVA package. The purpose is to ensure that the proposed method can offer favorable convergence conditions for an optimization algorithm. Such a verification effort is a necessary first step before applying the profile inversion method to real field-collected data, that are noisy and may include errors. Hence, the script focuses on an ideal scenario where synthetically manufactured data is used as input

## Installation
* Download the package on your PC. 
* Open MATLAB
* Go to the directory 'RIVA'
* add the different directories of the RIVA on your MATLAB path — Now you are ready to run the validation examples provided and generate your own analysis. 
* RIVA is compatible with [OCTAVE](https://www.gnu.org/software/octave/index)

For example validation of the library can be launched with

``` matlab
addpath("basic")
addpath("examples")
RIVA_insilico
```

## How to contribute
*	To make changes or add a new function: <i>(i)</i> For the repository (make your own separate copy), <i>(ii)</i> make changes, and <i>(iii)</i> open a 'pull request'. Once approved, it can be merged into the master branch. If you wish to chat beforehand about your contribution, open an issue or email to asmusskar@gmail.com.
*	If you find a bug in the code: open an 'issue' to notify contributors and create an official record.

Before contributing, please consider how your function fits into RIVA. At a minimum, functions must be well-documented and compatible with [OCTAVE](https://www.gnu.org/software/octave/index), not using any third party components. 

# References
Skar, Asmus and Levenberg, Eyal (2023). Road Profile Inversion from In-Vehicle Accelerometers. Transp. Eng. Part B Pavements (in press)

