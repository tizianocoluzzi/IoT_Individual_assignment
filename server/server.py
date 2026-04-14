import json
import time
import paho.mqtt.client as mqtt

# =========================
# CONFIG (PLACEHOLDERS)
# =========================
BROKER = "broker.hivemq.com"
PORT = 1883
SUB_TOPIC = "tzn/data"
PUB_TOPIC = "tzn/res"
CLIENT_ID = "delay_reflector"

# =========================
# STATE
# =========================
last_counter = None
total_received = 0
total_missing = 0

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
    payload["reflected_timestamp"] = time.time()

    client.publish(PUB_TOPIC, json.dumps(payload))

    # =========================
    # DEBUG OUTPUT
    # =========================
    loss_rate = (total_missing / (total_received + total_missing)) if (total_received + total_missing) > 0 else 0

    print(
        f"Received: {counter} | "
        f"Total recv: {total_received} | "
        f"Missing: {total_missing} | "
        f"Loss rate: {loss_rate:.4f}"
    )

# =========================
# MAIN
# =========================
client = mqtt.Client(client_id=CLIENT_ID)

client.on_connect = on_connect
client.on_message = on_message

client.connect(BROKER, PORT, keepalive=60)

client.loop_forever()
