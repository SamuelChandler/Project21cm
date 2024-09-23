from hackrf import *
from scipy import signal
from time import sleep
import matplotlib.pyplot as plt

def main():
    with HackRF() as hrf:
        
        hrf.sample_rate = 20e6
        hrf.center_freq = 2400e6
        hrf.lna_gain = 8
        hrf.vga_gain = 22
        num_samples = 2e4

        samples = hrf.read_samples(num_samples)
        
        f, pxx = signal.welch(samples, hrf.sample_rate/1e6, 'blackman',1024)
    
        

        plt.ion()

        fig = plt.figure()
        ax = fig.add_subplot(111)
        line1, = ax.plot(np.fft.fftshift(f)+1420, np.fft.fftshift(10*np.log10(pxx)))
        plt.ylabel('Power Density (dB/Hz)')
        plt.xlabel('Frequency (MHz)')
        plt.title('Spectrum Analyzer')
        plt.ylim(-70,10)


        while 1: 
            sleep(.2)
            samples = hrf.read_samples(2e4)[500:]
            f,pxx = signal.welch(samples, hrf.sample_rate/1e6, 'blackman', 1024)
            line1.set_ydata(np.fft.fftshift(10*np.log10(pxx)))
            fig.canvas.draw()
            fig.canvas.flush_events()

       

if __name__ == "__main__":
    main()

    
