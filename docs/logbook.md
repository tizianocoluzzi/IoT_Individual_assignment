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
