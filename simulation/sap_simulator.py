#!/usr/bin/env python3

import json
import random
import threading
import time

import paho.mqtt.client as mqtt

BROKER_HOST = "localhost"
BROKER_PORT = 1883

ORDER_INTERVAL_MIN_SEC = 8
ORDER_INTERVAL_MAX_SEC = 20

ROUTES = ["red", "green"]
PRIORITY_WEIGHTS = [1, 2, 2, 2, 3]

fleet_status = {}
known_agv_ids = set()
next_order_id = 1000
lock = threading.Lock()


def extract_agv_id(topic: str) -> str:
    parts = topic.split("/")
    return parts[1] if len(parts) >= 2 else "1"


def on_connect(client, userdata, flags, reason_code, properties=None):
    print(f"[SAP] Povezano na MQTT broker ({BROKER_HOST}:{BROKER_PORT}).")
    client.subscribe("agv/+/tasks")
    client.subscribe("agv/+/telemetry")


def on_message(client, userdata, msg):
    agv_id = extract_agv_id(msg.topic)

    with lock:
        known_agv_ids.add(agv_id)

    if msg.topic.endswith("/tasks"):
        try:
            data = json.loads(msg.payload.decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError):
            return

        pending_count = len(data.get("pending", []))
        is_busy = data.get("current") is not None

        with lock:
            fleet_status[agv_id] = {"pending": pending_count, "busy": is_busy}


def choose_best_agv() -> str | None:
    with lock:
        if not known_agv_ids:
            return None
        best_id = None
        best_load = float("inf")
        for agv_id in sorted(known_agv_ids):
            status = fleet_status.get(agv_id, {"pending": 0, "busy": False})
            load = status["pending"] + (1 if status["busy"] else 0)
            if load < best_load:
                best_load = load
                best_id = agv_id
        return best_id


def dispatch_order(client, route: str, priority: int, order_id: int):
    target_agv = choose_best_agv()
    if target_agv is None:
        print(f"[SAP] Nalog #{order_id} ceka - jos nijedno vozilo nije poznato "
              f"(pokreni bar jedan AGV simulator).")
        return

    task_cmd = {"action": "add", "route": route, "priority": priority}
    topic = f"agv/{target_agv}/cmd/task"
    client.publish(topic, json.dumps(task_cmd))

    priority_label = {1: "nizak", 2: "normalan", 3: "visok"}.get(priority, str(priority))
    print(f"[SAP] Nalog #{order_id} (ruta={route}, prioritet={priority_label}) "
          f"-> dodeljen vozilu #{target_agv} (bilo najslobodnije)")


def generate_and_dispatch(client):
    global next_order_id
    with lock:
        order_id = next_order_id
        next_order_id += 1

    route = random.choice(ROUTES)
    priority = random.choice(PRIORITY_WEIGHTS)
    dispatch_order(client, route, priority, order_id)


def order_generator_loop(client):
    while True:
        time.sleep(random.uniform(ORDER_INTERVAL_MIN_SEC, ORDER_INTERVAL_MAX_SEC))
        generate_and_dispatch(client)


def console_input_loop(client):
    print("\n[SAP] Rucno ubacivanje naloga - pritisni ENTER da odmah posaljes nalog,")
    print("      ili 'q' + ENTER za izlaz.\n")
    while True:
        try:
            cmd = input()
        except EOFError:
            break
        if cmd.strip().lower() == "q":
            print("[SAP] Gasim se.")
            break
        generate_and_dispatch(client)


def main():
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect(BROKER_HOST, BROKER_PORT, keepalive=60)
    client.loop_start()

    threading.Thread(target=order_generator_loop, args=(client,), daemon=True).start()

    print("[SAP] Simulator naloga pokrenut - cekam da se AGV vozila jave...")
    console_input_loop(client)

    client.loop_stop()
    client.disconnect()


if __name__ == "__main__":
    main()