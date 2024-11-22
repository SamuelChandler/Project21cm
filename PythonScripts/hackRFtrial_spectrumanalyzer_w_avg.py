from hackrf import *
from scipy import signal
from time import sleep
import matplotlib.pyplot as plt
import numpy as np
import sys


 


def main():

    rollingData = []
    averages = 4

    with HackRF() as hrf:
        
        arguments = sys.argv
        averages = 32

        hrf.sample_rate = 8e6          
        hrf.center_freq = int(arguments[1])*1e6
        hrf.lna_gain = int(arguments[2])
        hrf.vga_gain = 32   
        num_samples = 2e5

        sleep(.1)

        for count in range(averages):    
            samples = hrf.read_samples(num_samples)
            f, pxx = signal.welch(samples, hrf.sample_rate/1e6, 'blackman', nperseg=20e3,nfft=40e3)

            rollingData.append(pxx)

        avg_pxx = np.average(np.array(rollingData), axis=0)      
        y_center = hrf.center_freq/1e6

        plt.ion()

        fig = plt.figure()
        ax = fig.add_subplot(111)
        line1, = ax.plot(np.fft.fftshift(f)+y_center, np.fft.fftshift(10*np.log10(avg_pxx)))
        plt.ylabel('Power Density (dB/Hz)')
        plt.xlabel('Frequency (MHz)')
        plt.title('Spectrum Analyzer')
        plt.ylim(-80,10)


        while plt.fignum_exists(fig.number): 
            sleep(1)
            samples = hrf.read_samples(num_samples)
            f, pxx = signal.welch(samples, hrf.sample_rate/1e6, 'blackman', nperseg=20e3,nfft=40e3)

            rollingData.pop(0)
            rollingData.append(pxx)

            avg_pxx = np.average(np.array(rollingData), axis=0)      

            line1.set_ydata(np.fft.fftshift(10*np.log10(avg_pxx)))
            fig.canvas.draw()
            fig.canvas.flush_events()
    return

       

if __name__ == "__main__":
    main()

    
