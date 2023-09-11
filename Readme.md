## RIVA
Road-profile Inversion based on in-Vehicle Accelerations (RIVA) is a MATLAB-package for road profile inversion from in-vehicle accelerometer readings.

RIVA is based on matching measured vertical accelerations acquired over a specific road stretch with accelerations simulated by a calibrated quarter-car model. The matching is done with a PID control algorithm, considering the road profile as an unknown 'input' and the measured accelerations as 'target' signal. In this approach, the physics of the quarter-car model governs the 'transfer function' relating input to output. The control loop, graphically visualised in Figure 1.

<div>
<img src="pics/RIVA_f2.png" width="90%">
  <b>Figure 1:</b> Block diagram of the closed feedback loop PID controller utilised in RIVA. 
</div>

The components of the RIVA software is briefly described below:
* `main.m` - main script for defining vehicle, import (in this case synthetic) accelerometer data, and post-processing of the results. 

* `qcar_acc_ms_tvar.m` - script for simulating a syntehtic acceleration trace based on a known vehicle and road profile

RIVA is based on matching measured vertical accelerations acquired over a specific road stretch with accelerations simulated by a calibrated mechanical quarter-car model (representing the vehicle). The matching is done after data collection with a PID control algorithm, considering the road profile as an unknown 'input' and the measured accelerations as 'target' signal. In this approach, the physics of the quarter-car model governs the 'transfer function' relating input to output.
