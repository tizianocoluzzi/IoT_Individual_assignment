import serial
import numpy as np
import matplotlib.pyplot as plt

# ==== CONFIG ====

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
MAX_BINS = 25

FREQ = 1000
BINS = 1024

# ==== SERIAL SETUP ====

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# ==== PLOT SETUP ====

plt.ion()
fig, ax = plt.subplots()

# Frequency axis using (i * FREQ) / BINS
x = (np.arange(MAX_BINS) * FREQ) / BINS
y = np.zeros(MAX_BINS)

bars = ax.bar(x, y, width=(FREQ / BINS) * 0.8)

ax.set_ylim(0, 1)
ax.set_xlim(0, x[-1] if len(x) > 0 else 1)

ax.set_title("Real-Time FFT Spectrum")
ax.set_xlabel("Frequency (Hz)")
ax.set_ylabel("Magnitude")

# ==== MAIN LOOP ====

while True:
    try:
        line_raw = ser.readline().decode('utf-8').strip()

        if not line_raw:
            continue

        parts = line_raw.split()

        if len(parts) < 10:
            continue

        values = np.array([float(p) for p in parts[:MAX_BINS]])

        # normalize
        if np.max(values) > 0:
            values = values / np.max(values)

        # Update bar heights
        for bar, h in zip(bars, values):
            bar.set_height(h)

        fig.canvas.draw()
        fig.canvas.flush_events()

    except KeyboardInterrupt:
        break
    except Exception as e:
        print("Error:", e)

ser.close()
