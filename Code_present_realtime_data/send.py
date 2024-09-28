import time
import paho.mqtt.client as mqtt
import json
import random

mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

mqttc.connect("localhost")
mqttc.loop_start()

while True:
    x = random.sample(range(1, 255), 100)
    y = random.sample(range(1, 255), 100)
    z = random.sample(range(1, 255), 100)
    v = random.sample(range(1, 255), 100)

    data = {"x": x, "y": y, "z": z, "v": v}

    mqttc.publish("paho/test/topic/wallnut0611", json.dumps(data), qos=1)

    time.sleep(1)

mqttc.disconnect()
mqttc.loop_stop()