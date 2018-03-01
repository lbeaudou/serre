import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
import Adafruit_DHT
import time
import sys
import json
import smbus
import time
import picamera
import base64

GPIO.setmode(GPIO.BCM)
i2c = smbus.SMBus(1)
sensor = Adafruit_DHT.DHT22
#def des gpio
GPIO.setup(20, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(21, GPIO.OUT, initial=GPIO.HIGH)  
GPIO.setup(16, GPIO.IN) 
camera = picamera.PiCamera()

camera.vflip = True
client = mqtt.Client("omega") 
event = json.dumps([{"type":"camera","value_checked":"false","value_hour":"12:00"},{"type":"km_pompe","value_checked":"false","value_hum_pour":"35"},{"type":"km_lum","value_checked":"false","value_lum_lux":"27"},{"type":"km_ventil","value_checked":"false","value_temp_pour":"31","value_speed_pour":"56"}])
km_1 = km_2 = km_3 = km_4 = 0
def on_message(client, userdata, msg):
    global km_1
    global km_2
    global event
    print(msg.topic+" "+str(msg.payload))
    #if msg.topic == "serre/event/omega":
      #event = json.load(msg.payload)
    if msg.topic == "serre/km/omega/km_1":
      if msg.payload == "1":
          GPIO.output(20, GPIO.LOW)
          km_1 = 1
      else:
          GPIO.output(20, GPIO.HIGH)
          km_1 = 0
        
      if msg.topic == "serre/km/omega/km_2":
        if msg.payload == "1":
          GPIO.output(21, GPIO.LOW)
          km_2 = 1
      else:
          GPIO.output(21, GPIO.HIGH)
          km_2 = 0
    j = {"km_1": km_1, "km_2": km_2,"km_3": km_3, "km_4": km_4, "device":"omega"}
    client.publish("serre/status/", json.dumps(j))
      

#client = mqtt.Client()
#client.on_connect = on_connect
client.on_message = on_message

client.connect("broker.mqttdashboard.com", 1883, 60)
 

#client.loop_forever()

client.subscribe("serre/km/omega/#", 2)

client.subscribe("serre/event/omega", 2)
client.loop_start()
val = 1
i2c.write_byte(0x23, 0x00)

i2c.write_byte(0x23, 0x01)


i2c.write_byte(0x23, 0x12)

t = time.time()
t2 = time.time()
while True:
  try:
    if time.time() -t > 300:
      t = time.time()
      humidity, temperature = Adafruit_DHT.read_retry(sensor, '4')
      camera.capture('image1.jpg')
      with open("image1.jpg", "rb") as image_file: 
        encoded_string = base64.b64encode(image_file.read())
        client.publish("serre/img/omega", encoded_string)
        print('send')
    
      #client.publish("serre/cp/omega/temp", temperature)
      #client.publish("serre/cp/omega/humity", humidity)
    
      lvleau = GPIO.input(16)
      print(lvleau)
      #client.publish("serre/cp/omega/lvleau", lvleau)
    
  
    # i2c.write_i2c_block_data(0x23, 0x04, [0x10, 0x20])
      
      data = [0,0]
      data = i2c.read_i2c_block_data(0x23, 0x12, 2)
      lum = ((data[0]) << 8) + (data[1])
      #client.publish("serre/cp/omega/lum", lum)
      
      ardu= i2c.read_i2c_block_data(0x24, 0x01, 4)
      print(ardu)
      ht = ardu[0] + (ardu[1] <<8)
      ht100 = ht*100/950;
      #client.publish("serre/cp/omega/ht", ht100)
      
      tension_efficace = 230.0
      ACS712_RAPPORT = 100.0
      arduc =  ardu[2] + (ardu[3] <<8)*1.0
      courant = (arduc)/1024*5/ACS712_RAPPORT*100000
      print(arduc)
      print(courant)
      courant_efficace = courant / 1.414
      pw = (courant_efficace * tension_efficace/1000)
      print(pw)
      
      #client.publish("serre/cp/omega/amp", pw)
      j = {"amp": pw, "ht": ht100,"lum": lum, "lvleau": lvleau, "humidity":humidity, "temp": temperature, "device":"omega"}
      client.publish("serre/sensor/omega", json.dumps(j))
    if time.time() - t2 > 1:
      t2 = time.time()
      print(json.loads(event)[0])
      
  except IOError as e:
    print "Oops!  That was no valid number.  Try again..."
    print(e)
