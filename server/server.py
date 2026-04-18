import json
import time
import paho.mqtt.client as mqtt

# =========================
# CONFIG (PLACEHOLDERS)
# =========================
BROKER = "broker.hivemq.com"
PORT = 1883
SUB_TOPIC = "tzn/data"
PUB_TOPIC = "tzn/time"
CLIENT_ID = "delay_reflector"

# =========================
# STATE
# =========================
last_counter = None
total_received = 0
total_missing = 0

def now_us():
    return int(time.monotonic_ns() // 1000)  # microseconds
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
    t2 = now_us();
    try:
        payload = json.loads(msg.payload.decode())
    except Exception as e:
        print("Invalid JSON:", e)
        return

    if "cnt" not in payload:
        print("Missing counter field")
        return

    counter = payload["cnt"]
    total_received += 1

    # =========================
    # LOSS DETECTION
    # =========================
    if last_counter is not None:
        expected = last_counter + 1

        if counter > expected:
            missing = counter - expected
            total_missing += missing
            print(f"⚠️ Missing packets: {missing} (from {expected} to {counter-1})")

        elif counter < last_counter:
            print(f"⚠️ Out-of-order packet: {counter}")

    last_counter = counter

    # =========================
    # ECHO BACK (LOOPBACK)
    # =========================
    # Add reflector timestamp for RTT measurement
    response = {
            "cnt": counter,
            "t1": payload["t1"],
            "t2": t2,
            "t3": now_us()
    }

    client.publish(PUB_TOPIC, json.dumps(response))

    # =========================
    # DEBUG OUTPUT
    # =========================
    loss_rate = (total_missing / (total_received + total_missing)) if (total_received + total_missing) > 0 else 0

    print(
        f"id: {payload["cnt"]} |"
        f"mean: {payload["mean"]} |"
        f"exec_time: {payload["window_exec_us"]} |"
        f"sampling freq: {payload["sampling_freq_hz"]} |"
        f"Loss rate: {loss_rate:.4f}|"
    )

# =========================
# MAIN
# =========================
client = mqtt.Client(client_id=CLIENT_ID)

client.on_connect = on_connect
client.on_message = on_message

client.connect(BROKER, PORT, keepalive=60)

client.loop_forever()
