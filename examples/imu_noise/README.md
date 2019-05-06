# Noise Generation Example

This example creates a sequence of samples for an accelerometer and gyroscope based
on the https://www.analog.com/media/en/technical-documentation/data-sheets/adis16448.pdf, a commonly-used MEMS IMU.

The sequence of samples can then be evaluated with Allan Variance to determine that
the noise generation in ign-sensors is working as expected.

To evaluate noise, first build and run.

```
mkdir build
cd build
cmake ..
make
./sensor_noise
```

This will generate two series of samples, one for accelerometer and another for gyroscope.

To use the analysis script, first install Allan Tools:

```
pip install --user allantools
```

Then execute the script:

```
../plot_samples.py
```

This will produce two graphs: the Allan Deviation plots for both the simulated accelerometer and gyroscope.  The values on these graphs should correspond closely to the inputs:

* Gyroscope Allan Plot:
  * sigma_N: Gyroscope Noise Density
  * sigma_K: Gyroscope Random Walk
* Accelerometer Allan Plot:
  * sigma_N: Accelerometer Noise Density
  * sigma_K: Accelerometer Random Walk


While this technique is used here to validate the operation of the algorithm, it could also be used
to estimate the noise parameters for a real IMU.


## References:

* Python Allan Tools: https://github.com/aewallin/allantools
* Inertial Sensor Noise Analysis with Allan Variance: https://www.mathworks.com/help/fusion/examples/inertial-sensor-noise-analysis-using-allan-variance.html
* https://en.wikipedia.org/wiki/Allan_variance
* http://cache.freescale.com/files/sensors/doc/app_note/AN5087.pdf
* Kalibr IMU Noise Model: https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model
