# IoT_Individual_assignment
Individual assigment of the IoT Algorithm and Services course in Spienza Engineering in Computer Science and artificial Intelligence 
## Goal
Using Arduino Framework write a signal generator and a sampler with the following tasks:
- oversampling
- identify optimal sampling frequency
- compute aggregation function over a window
- communicate the aggregated value using MQTT over WiFi
- communicate the aggregated value over LoRaWAN
- performance measurement:
    - energy saving
    - per-window execution time
    - data volume transmitted
    - end-to-end latency of the system
- bonus:
    - consider noise and spikes in the signal
    - use anomaly aware filters: Z-score, Hamper and evaluate TPR FPR Mean Error Reduction
    - execution time and energy impact of the filters
    - impact of the anomalies in the FFT
    - impact with the filtering ??
    - measure the effect on the window size, in particular energy consumption, end-to-end delay, memory usage.
