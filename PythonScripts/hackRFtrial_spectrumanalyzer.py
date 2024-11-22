from hackrf import *
from scipy import signal
from time import sleep
import matplotlib.pyplot as plt
import numpy as np
import sys


def main():
    with HackRF() as hrf:
        
        hrf.sample_rate = 20e6

        arguments = sys.argv
        print(arguments[1])
        
        hrf.center_freq = int(arguments[1])*1e6
        hrf.lna_gain = int(arguments[2])
        hrf.vga_gain = 16
        num_samples = 2e5

        sleep(.1)
        samples = hrf.read_samples(num_samples)
        
        f, pxx = signal.welch(samples, hrf.sample_rate/1e6, 'blackman', nperseg=20e3,nfft=40e3)
    
        y_center = hrf.center_freq/1e6

        plt.ion()

        fig = plt.figure()
        ax = fig.add_subplot(111)
        line1, = ax.plot(np.fft.fftshift(f)+y_center, np.fft.fftshift(10*np.log10(pxx)))
        plt.ylabel('Power Density (dB/Hz)')
        plt.xlabel('Frequency (MHz)')
        plt.title('Spectrum Analyzer')
        plt.ylim(-80,10)


        while plt.fignum_exists(fig.number): 
            sleep(.5)
            
            samples = hrf.read_samples(num_samples)
            f,pxx = signal.welch(samples, hrf.sample_rate/1e6, 'blackman', nperseg=20e3,nfft=40e3)
            line1.set_ydata(np.fft.fftshift(10*np.log10(pxx)))
            fig.canvas.draw()
            fig.canvas.flush_events()
    return

       

if __name__ == "__main__":
    main()

    
