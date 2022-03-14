import datetime
from multiprocessing.connection import wait
from flask import Flask, render_template, request
import time
import csv
import matplotlib.pyplot as plt
from gps_class import GPSVis
from flask_serial import Serial
import pandas as pd

app = Flask(__name__)
app.config['SERIAL_TIMEOUT'] = 0.2
app.config['SERIAL_PORT'] = '/dev/ttyUSB0'
app.config['SERIAL_BAUDRATE'] = 115200
app.config['SERIAL_BYTESIZE'] = 8
app.config['SERIAL_PARITY'] = 'N'
app.config['SERIAL_STOPBITS'] = 1

ser =Serial(app)


@app.route('/')
def hello():
    now = datetime.datetime.now()
    timestring = now.strftime("%Y-%m-%d %H:%M:")
   
    vis = GPSVis(data_path='data/gps_data.csv',
             map_path='map.png',  # Path to map downloaded from the OSM.
        points=(46.06571, 14.50795, 46.05392,14.52232)) # Two c
    vis.create_image(color=(0, 0, 255), width=3)  # Set the color and the width of the GNSS tracks.     
    vis.plot_map(output='save')
    print()
    data = pd.read_csv('data/data.csv', names=['LATITUDE', 'LONGITUDE',"rssi","time"], sep=',')
        
    # copy dataframe structure only, no data
    c_data = pd.DataFrame(columns=data.columns)

        # copy valid values only
    c_data['LATITUDE'] = pd.to_numeric(data['LATITUDE'], errors='coerce')
    c_data['LONGITUDE'] = pd.to_numeric(data['LONGITUDE'], errors='coerce')
    c_data['rssi'] = pd.to_numeric(data['rssi'])

    rssi = c_data['rssi'].values[-1]    
    lat = c_data['LATITUDE'].values[-1]
    lon = c_data['LONGITUDE'].values[-1]
    lat = "{:.6f}".format(lat)
    lon = "{:.6f}".format(lon)

    
    templateData = {
        'title' : 'LoRa base station',
        'time'  :  timestring,
        'lat'   :  lat,
        'lon'   :  lon,
        'rssi'  :  rssi
    }
    return render_template('index.html', **templateData)
    
    
@ser.on_message()
def handle_message(msg):
    now = datetime.datetime.now()
    timestring = now.strftime("%Y-%m-%d %H:%M:%S")
    packet = msg.decode().strip()
    packet =packet + "," + timestring
    print(packet)
    with open("data/data.csv","a") as f:
          f.write(packet + "\n")
    with open("data/data.csv", mode="r") as org:
        obj =  csv.reader(org)
        wait(1)

        with open("data/gps_data.csv", mode="w") as new:
            w_obj = csv.writer(new)
            for column in obj:
                w_obj.writerow((column[0], column[1]))


if __name__ == '__main__':
    app.run(debug=True, port=5000, host='0.0.0.0')