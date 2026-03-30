# Logbook
The purpose of this logbook is to register toughts and tentative in this way I can remember what I've done.

## History:
### Commits
#### Naive version
The first thing I've done is the trivial implementation of both the signal generator and the sampler and run the FFT code.
The majorpeak not always corresponds to the max freqeuncy, since noise affect the signal a treshold maybe a solution but finiding a suitable treshold is hard.
Next steps: find a way to identify peaks, probably outlier detection is a good solution.

#### Improving signal generator
To avoid confusion I need to create a better generator which can generate very high frequency signal, and after get better sampler.
Unfortunately the sampler (heltec lora) cannot use the i2s to sample (at least for what I discovered). So I need to keep trying to stabilize the output of the sampler as it is. I tried the double buffer strategy but it didnt work. 

#### plotthe FFT
Since it is hard to work without seeing the output of the FFT graphically I vibe-coded a serial plotter in python specifically designed to plot the FFT result. The FFT is noisy, it was really unstable, I moved one task to the other core and the result is slighly better but it varies too much, I suspect it may be a concurrency problem (reader is much faster than FFT)

#### Going back from teh siganl generator
The signal generator with i2s showed to be too noisy so at the moment is not the prorioty to use this kind of generator since we need to use lower freqeuncies.

#### Measuring time 
I tried to determine the maximum sampling frequency by measuring time to accomplish the sampling task without considering the delay and addind to the calculation after the measurements.
The time measured are show to have a median of 60us but with peaks to 120us. 

#### hard schduling
The 120us peak made me switch to a dynamic waiting strategy with `vTaskDelayUntil`, this should allow to have less noisy measuraments. This is confirmed by the FFT which is cleaner now.

#### Peak detection
To detect peaks I set up a threshold dynamically: 3*mean and to avoid noise near the important frequency I detected peak, so a freqeuncy is taken into account only if major than both its neighbour.
This is useless because we need just the max frequency. I've seen that sometimes 1'Hz is detected (up to now I have two frequencies 1 and 5 Hz) so there's still noise. The visual plotter I made in python show a substantial difference between 10Hz height and 1 and 5, so maybe it's enough to high up the treshold. To check if the peak detection worked properly I tried to increase the precision increasing the samples to 2048, the noise reappeared so I tried to create a link between the precision and the treshold: th = 5 * SAMPLES/FREQ. Because having more frequency the noise in the background splits so the average value decrease and some pykes of noise may be counted, in this way the magnitude needed increase. Probably there's a limit I have to test yet.

#### Window and WiFi
I implemented a simple mean calculator over a sample based thumbling window and sent the result via wifi. I chose a thumbling window because it is more intresting for the quantity of data when the sample rate decreases. I also tried to increase the total sample to increase precision a stress the treshold methob but it resisted. 
