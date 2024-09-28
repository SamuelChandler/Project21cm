from hackrf import *
from scipy import signal
from time import sleep
import matplotlib.pyplot as plt
import numpy as np

def main():
    with HackRF() as hrf:
        
        hrf.sample_rate = 20e6
        hrf.center_freq = np.int16(input('Please enter center frequncy in MHz:'))*1e6
        hrf.lna_gain = 8
        hrf.vga_gain = 22
        num_samples = 2e4

        samples = hrf.read_samples(num_samples)
        
        f, pxx = signal.welch(samples, hrf.sample_rate/1e6, 'blackman',1024)
    
        y_center = hrf.center_freq/1e6

        plt.ion()

        fig = plt.figure()
        ax = fig.add_subplot(111)
        line1, = ax.plot(np.fft.fftshift(f)+y_center, np.fft.fftshift(10*np.log10(pxx)))
        plt.ylabel('Power Density (dB/Hz)')
        plt.xlabel('Frequency (MHz)')
        plt.title('Spectrum Analyzer')
        plt.ylim(-70,10)


        while plt.fignum_exists(fig.number): 
            sleep(.2)
            samples = hrf.read_samples(2e4)[500:]
            f,pxx = signal.welch(samples, hrf.sample_rate/1e6, 'blackman', 1024)
            line1.set_ydata(np.fft.fftshift(10*np.log10(pxx)))
            fig.canvas.draw()
            fig.canvas.flush_events()
    return

       

if __name__ == "__main__":
    main()

    
