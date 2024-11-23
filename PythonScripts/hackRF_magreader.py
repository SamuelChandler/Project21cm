from hackrf import *
from scipy import signal
from time import sleep
import numpy as np
import matplotlib.pyplot as plt


center_mag = []
rollingData = []
fullBandAvg = []
finalBandAvgpxx = []
values_to_avg = 1
average_mag = 0


with HackRF() as hrf:
    
    # Desired hackRF settings
    hrf.sample_rate = 20e6
    hrf.center_freq = 1420e6
    hrf.lna_gain = 40
    hrf.vga_gain = 8
    num_samples = 2e4

    # loop and add power arrays to an array of arrays

    for count in range(values_to_avg):    
        samples = hrf.read_samples(num_samples)
        f, pxx = signal.welch(samples, hrf.sample_rate/1e6, 'blackman', nperseg=20e3,nfft=40e3)

        rollingData.append(pxx)
    
    # create an fft corrected frequency array
    sortedFreq = np.fft.fftshift(f)+1420
    
    # Average the power values point by point. (average per index)
    pointAvgpxx = np.average(np.array(rollingData), axis=0)      

    # acquire the indicies for desired frequency range MHz
    firstFreqIndex = np.where(sortedFreq == 1419)
    lastFreqIndex = np.where(sortedFreq == 1421)

    # create a new array with desired averaged values
    for index,power in enumerate(pointAvgpxx):
        if index >= firstFreqIndex[0]: 
            if index > lastFreqIndex[0]:
                break
            else: 
                finalBandAvgpxx.append(power)
    
    # Collecting and printing the average across the desired band
    average_mag = np.average(finalBandAvgpxx)
    print(average_mag)



    




    
