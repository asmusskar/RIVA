## RIVA
RIVA is a MATLAB-package for advanced pavement modeling that is highly adaptive.

<div>
<img src="pics/pic1.png" width="90%">
</div>

The components of the RIVA software is briefly described below:
* `main.m` - main script for defining vehicle, import (in this case synthetic) accelerometer data, and post-processing of the results. 

* `qcar_acc_ms_tvar.m` - script for simulating a syntehtic acceleration trace based on a known vehicle and road profile

RIVA is based on matching measured vertical accelerations acquired over a specific road stretch with accelerations simulated by a calibrated mechanical quarter-car model (representing the vehicle). The matching is done after data collection with a PID control algorithm, considering the road profile as an unknown 'input' and the measured accelerations as 'target' signal. In this approach, the physics of the quarter-car model governs the 'transfer function' relating input to output.
