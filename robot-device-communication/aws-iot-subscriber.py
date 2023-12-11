import time
import paho.mqtt.client as mqtt
import ssl
import json
import _thread

def on_connect(client, userdata, flags, rc):
    print("Connected to AWS IoT: " + str(rc))
    client.subscribe("wheelChair/position")  # Subscribe to the "raspi/data" topic when connected

def on_message(client, userdata, msg):
    payload = json.loads(msg.payload)
    print(f"Received message: {payload['msg']}")

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message  # Set the on_message callback

# Set up TLS configuration (replace with your actual certificate and key files)
client.tls_set(ca_certs='./rootCA.pem', certfile='./certificate.pem.crt', keyfile='./private.pem.key', tls_version=ssl.PROTOCOL_SSLv23)
client.tls_insecure_set(True)

# Connect to the AWS IoT broker
client.connect("END POINT", 8883, 60)


# Start the MQTT client loop to handle communication
client.loop_forever()
