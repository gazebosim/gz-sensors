#!/usr/bin/env python3

import allantools
import numpy as np
import matplotlib.pyplot as plt

def estimate_random_walk(tau, adev):
    logtau = np.log10(tau)
    logadev = np.log10(adev)
    dlogadev = np.diff(logadev) / np.diff(logtau)

    slope = -0.5
    minidx = np.argmin(np.abs(dlogadev - slope))

    b = logadev[minidx] - slope * logtau[minidx]
    log_n = slope * np.log10(1) + b

    tau_n = 1
    n = 10**log_n
    line_n = n / np.sqrt(tau)
    return (tau_n, n, line_n)


def estimate_rate_random_walk(tau, adev):
    logtau = np.log10(tau)
    logadev = np.log10(adev)
    dlogadev = np.diff(logadev) / np.diff(logtau)

    slope = 0.5
    minidx = np.argmin(np.abs(dlogadev - slope))

    b = logadev[minidx] - slope * logtau[minidx]
    log_k = slope * np.log10(3) + b

    tau_k = 3
    k = 10**log_k
    line_k = k * np.sqrt(tau/3)
    return (tau_k, k, line_k)


def estimate_bias_instability(tau, adev):
    logtau = np.log10(tau)
    logadev = np.log10(adev)
    dlogadev = np.diff(logadev) / np.diff(logtau)

    slope = 0.0
    dlogadev = np.diff(logadev) / np.diff(logtau)
    minidx = np.argmin(np.abs(dlogadev - slope))
    b = logadev[minidx] - slope * logtau[minidx]

    scf_b = np.sqrt(2 * np.log10(2)/np.pi)
    log_b = b - np.log10(scf_b)

    tau_b = tau[minidx]
    b = 10**log_b
    line_b = b * scf_b * np.ones(tau.shape)
    return (tau_b, b * scf_b, line_b)


def plot_allan(samples):
    taus = np.logspace(-3, 3, 100)
    (t2, ad, ade, adn) = allantools.oadev(samples, rate=100, data_type='freq',
                                          taus=taus)

    (tau_n, n, line_n) = estimate_random_walk(t2, ad)
    (tau_k, k, line_k) = estimate_rate_random_walk(t2, ad)
    (tau_b, b, line_b) = estimate_bias_instability(t2, ad)

    plt.subplot(111, xscale='log', yscale='log')
    plt.loglog(t2, ad)
    plt.grid(True)

    plt.text(tau_n, n, 'N: ' + str(n))
    plt.plot(t2, line_n)
    plt.text(tau_k, k, 'K: ' + str(k))
    plt.plot(t2, line_k)
    plt.text(tau_b, b, 'B: ' + str(b))
    plt.plot(t2, line_b)
    plt.legend(['sigma', 'sigma_N', 'sigma_K', 'sigma_B'])


if __name__ == '__main__':
    accelSamples = np.fromfile('./accel_samples.bin', dtype=np.float64)
    gyroSamples = np.fromfile('./gyro_samples.bin', dtype=np.float64)

    plt.figure()
    plot_allan(accelSamples)
    plt.title('Accelerometer Allan Variance')
    plt.xlabel('tau')
    plt.ylabel('sigma(tau)')

    plt.figure()
    plot_allan(gyroSamples)
    plt.title('Gyroscope Allan Variance')
    plt.xlabel('tau')
    plt.ylabel('sigma(tau)')

    plt.show()

