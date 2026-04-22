import json
import time
import csv
import os
import paho.mqtt.client as mqtt

# =========================
# CONFIG
# =========================
BROKER = "broker.hivemq.com"
PORT = 1883
SUB_TOPIC = "tzn/data"
PUB_TOPIC = "tzn/time"
CLIENT_ID = "delay_reflector"
CSV_FILE = "mqtt_log.csv"

# =========================
# CSV SETUP
# =========================
CSV_HEADERS = [
    "cnt", "t1", "mean", "window_exec_us", "sampling_freq_hz",
    "adaptive_sampling", "noise_enabled", "spike_probability",
    "filter_window_size", "auto_profile", "filter_applied",
    "filter_mean_exec_us", "tp", "tn", "fp", "fn",
    "previous_latency_us",
    "fpr", "tpr",
    "total_received", "total_missing", "loss_rate",
    "missing_packets", "out_of_order"
]

def init_csv():
    file_exists = os.path.isfile(CSV_FILE)
    f = open(CSV_FILE, "a", newline="")
    writer = csv.DictWriter(f, fieldnames=CSV_HEADERS)
    if not file_exists:
        writer.writeheader()
    return f, writer

# =========================
# STATE
# =========================
last_counter = None
total_received = 0
total_missing = 0

csv_file, csv_writer = init_csv()

def now_us():
    return int(time.monotonic_ns() // 1000)

# =========================
# CALLBACKS
# =========================
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to broker")
        client.subscribe(SUB_TOPIC)
    else:
        print(f"Connection failed with code {rc}")

def on_message(client, userdata, msg):
    global last_counter, total_received, total_missing

    t2 = now_us()

    try:
        payload = json.loads(msg.payload.decode())
    except Exception as e:
        print("Invalid JSON:", e)
        return

    if "cnt" not in payload:
        print("Missing counter field")
        return

    # =========================
    # EXTRACT FIELDS
    # =========================
    counter             = payload["cnt"]
    t1                  = payload["t1"]
    mean                = payload["mean"]
    window_exec_us      = payload["window_exec_us"]
    sampling_freq_hz    = payload["sampling_freq_hz"]
    adaptive_sampling   = payload["adaptive_sampling"]
    noise_enabled       = payload["noise_enabled"]
    spike_probability   = payload["spike_probability"]
    filter_window_size  = payload["filter_window_size"]
    auto_profile        = payload["auto_profile"]
    filter_applied      = payload["filter_applied"]
    filter_mean_exec_us = payload["filter_mean_exec_us"]
    tp                  = int(payload["tp"])
    tn                  = int(payload["tn"])
    fp                  = int(payload["fp"])
    fn                  = int(payload["fn"])
    previous_latency_us = payload["previous_latency_us"]

    total_received += 1

    # =========================
    # LOSS DETECTION
    # =========================
    missing_packets = 0
    out_of_order = False

    if last_counter is not None:
        expected = last_counter + 1
        if counter > expected:
            missing_packets = counter - expected
            total_missing += missing_packets
            print(f"⚠️  Missing packets: {missing_packets} (from {expected} to {counter - 1})")
        elif counter < last_counter:
            out_of_order = True
            print(f"⚠️  Out-of-order packet: {counter}")

    last_counter = counter

    # =========================
    # ECHO BACK (LOOPBACK)
    # =========================
    response = {
        "cnt": counter,
        "t1": t1,
        "t2": t2,
        "t3": now_us()
    }
    client.publish(PUB_TOPIC, json.dumps(response))

    # =========================
    # METRICS
    # =========================
    loss_rate = (total_missing / (total_received + total_missing)) if (total_received + total_missing) > 0 else 0
    fpr = fp / (fp + tn) if (fp + tn) != 0 else 0
    tpr = tp / (tp + fn) if (tp + fn) != 0 else 0

    # =========================
    # DEBUG OUTPUT
    # =========================
    print(
        f"cnt: {counter} | "
        f"t1: {t1} | "
        f"mean: {mean} | "
        f"window_exec_us: {window_exec_us} | "
        f"sampling_freq_hz: {sampling_freq_hz} | "
        f"adaptive_sampling: {adaptive_sampling} | "
        f"noise_enabled: {noise_enabled} | "
        f"spike_prob: {spike_probability:.3f} | "
        f"filter_window_size: {filter_window_size} | "
        f"auto_profile: {auto_profile} | "
        f"filter_applied: {filter_applied} | "
        f"filter_mean_exec_us: {filter_mean_exec_us} | "
        f"tp: {tp} tn: {tn} fp: {fp} fn: {fn} | "
        f"prev_latency_us: {previous_latency_us} | "
        f"fpr: {fpr:.4f} | tpr: {tpr:.4f} | "
        f"loss_rate: {loss_rate:.4f}"
    )

    # =========================
    # CSV LOGGING
    # =========================
    row = {
        "cnt":                  counter,
        "t1":                   t1,
        "mean":                 mean,
        "window_exec_us":       window_exec_us,
        "sampling_freq_hz":     sampling_freq_hz,
        "adaptive_sampling":    adaptive_sampling,
        "noise_enabled":        noise_enabled,
        "spike_probability":    round(spike_probability, 6),
        "filter_window_size":   filter_window_size,
        "auto_profile":         auto_profile,
        "filter_applied":       filter_applied,
        "filter_mean_exec_us":  filter_mean_exec_us,
        "tp":                   tp,
        "tn":                   tn,
        "fp":                   fp,
        "fn":                   fn,
        "previous_latency_us":  previous_latency_us,
        "fpr":                  round(fpr, 6),
        "tpr":                  round(tpr, 6),
        "total_received":       total_received,
        "total_missing":        total_missing,
        "loss_rate":            round(loss_rate, 6),
        "missing_packets":      missing_packets,
        "out_of_order":         out_of_order,
    }
    csv_writer.writerow(row)
    csv_file.flush()

# =========================
# MAIN
# =========================
client = mqtt.Client(client_id=CLIENT_ID)
client.on_connect = on_connect
client.on_message = on_message

try:
    client.connect(BROKER, PORT, keepalive=60)
    client.loop_forever()
except KeyboardInterrupt:
    print("\nShutting down...")
finally:
    csv_file.close()
    print(f"CSV saved to: {CSV_FILE}")
