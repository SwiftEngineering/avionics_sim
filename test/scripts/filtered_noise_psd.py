import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import scipy as sp
from scipy.signal.windows import hann


CWD_DIR = os.path.abspath(os.getcwd())
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BUILD_DIR = os.path.join(SCRIPT_DIR,'../../build')
BUILD_UNIT_TEST_DIR = os.path.join(BUILD_DIR,'test/unit')

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Not enough arguments provided, using default: ')
        input_file = 'dryden_sim_linear_rates.csv'
    else:
        input_file = sys.argv[1]

    #-----------------------------------------

    if os.path.exists(os.path.join(CWD_DIR, input_file)):
        input_file = os.path.join(CWD_DIR, input_file)
    elif os.path.exists(os.path.join(BUILD_DIR, input_file)):
        input_file = os.path.join(os.path.join(BUILD_DIR, input_file))
    elif os.path.exists(os.path.join(BUILD_UNIT_TEST_DIR, input_file)):
        input_file = os.path.join(os.path.join(BUILD_UNIT_TEST_DIR, input_file))
    else:
        print ('Path not found for file: ', input_file)

    print('Using File: ', input_file)
    #-----------------------------------------

    #-----------------------------------------
    my_data = np.genfromtxt(input_file, delimiter=',')

    time_s = my_data[:,0]
    rate_u = my_data[:,1]
    rate_v = my_data[:,2]
    rate_w = my_data[:,3]
    #-----------------------------------------

    plt.plot(time_s, rate_u)
    plt.ylabel('linear rate - longitude (m/s)')
    plt.xlabel('time(s)')
    plt.show()

    #-----------------------------------------
    dt_s = time_s[1] - time_s[0]
    #-----------------------------------------


    #-----------------------------------------
    freq_sample = 1 / dt_s # sample rate, number of samples per unit of time

    nblock = int(len(time_s) / 50)
    overlap = int(nblock / 2)
    win = hann(nblock, False)

    SF = np.linalg.norm(win)**2 / np.sum(win)**2
    #-----------------------------------------


    freq_sim_u, psd_sim_u = sp.signal.welch(rate_u, freq_sample, window=win, noverlap=overlap, nfft=nblock, scaling='spectrum', detrend=False)
    psd_sim_u = 10*np.log10(psd_sim_u)

    plt.semilogx(freq_sim_u, psd_sim_u, '.', label='Simulated')
    # plt.semilogx(freq_theo, psd_theo_u, label='Theoretical Dryden')
    plt.legend()
    plt.grid()
    plt.xlabel('frequency (Hz)')
    plt.ylabel('PSD (dB)')
    plt.xlim([0.01, 100])
    plt.ylim([-80, 20])

    plt.show()



