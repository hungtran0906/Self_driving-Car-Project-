import paho.mqtt.client as mqtt
import json
import csv

filename = "records.csv"
fields = ['crash', 'mass', 'y', 'v']
def on_subscribe(client, userdata, mid, reason_code_list, properties):
    # Since we subscribed only for a single channel, reason_code_list contains
    # a single entry
    if reason_code_list[0].is_failure:
        print(f"Broker rejected you subscription: {reason_code_list[0]}")
    else:
        print(f"Broker granted the following QoS: {reason_code_list[0].value}")

def on_unsubscribe(client, userdata, mid, reason_code_list, properties):
    # Be careful, the reason_code_list is only present in MQTTv5.
    # In MQTTv3 it will always be empty
    if len(reason_code_list) == 0 or not reason_code_list[0].is_failure:
        print("unsubscribe succeeded (if SUBACK is received in MQTTv3 it success)")
    else:
        print(f"Broker replied with failure: {reason_code_list[0]}")
    client.disconnect()

def on_message(client, userdata, message):
    j_msg = json.loads(message.payload.decode('utf-8'))
    crash = j_msg['crash']
    mass = j_msg['mass']
    # x = j_msg['x']
    y = j_msg['y']
    # z = j_msg['z']
    v = j_msg['v']

    with open(filename, 'a', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        for i in range(len(crash)):
            csvwriter.writerow([crash[i], mass[i], y[i], v[i]])

def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code.is_failure:
        print(f"Failed to connect: {reason_code}. loop_forever() will retry connection")
    else:
        # we should always subscribe from on_connect callback to be sure
        # our subscribed is persisted across reconnections.
        # client.subscribe("paho/test/topic/wallnut0611")
        client.subscribe("data/realtime")


mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqttc.on_connect = on_connect
mqttc.on_message = on_message
mqttc.on_subscribe = on_subscribe
mqttc.on_unsubscribe = on_unsubscribe
f = open(filename, "w+", newline='')
csv.DictWriter(f, fieldnames=fields).writeheader()
f.close()
mqttc.connect("localhost")
mqttc.loop_forever()
