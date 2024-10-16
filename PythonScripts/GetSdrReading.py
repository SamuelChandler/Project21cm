from hackrf import *
from scipy import signal
from time import sleep
import matplotlib.pyplot as plt


#script Version of the hackRF_magreader.py for pyinstaller

print("Getting SDR Reading")

center_mag = []
values_to_avg = 100

with HackRF() as hrf:
    
    hrf.sample_rate = 20e6
    hrf.center_freq = 2400e6
    hrf.lna_gain = 8
    hrf.vga_gain = 22
    num_samples = 2e4

    # loop to gather multiple magnitude values at center frequency and average them
    for number in range(values_to_avg): 
        samples = hrf.read_samples(num_samples)
        
        f, pxx = signal.welch(samples, hrf.sample_rate/1e6, 'blackman',1024)

        center_sample_index = len(pxx)/2 - 1 

        center_value = pxx[int(center_sample_index)]
    
        center_mag.append(10*np.log10(center_value))

        average_mag = sum(center_mag) / len(center_mag)

average_mag