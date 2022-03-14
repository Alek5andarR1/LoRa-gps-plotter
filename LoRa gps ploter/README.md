This is LoRa peer to peer real time gps plotter. It is using TTGO T-Beam V1.1 as LoRa radio. One node transmits it's gps position, second is a reciver for computer base station. Nodes are programmed using Arduino libraries and are PlatformIO projects. Base station is build by using https://github.com/tisljaricleo/GPS-visualization-Python. And it is python flask server. Raspberry pi is set as soft acces point. Seting it's own network and hosting aplication server. Comunication with nodes and raspberry pi is done by serial.  

Problems with TTGO T-Beam V1.1 gps were solved by shorting gps batery for five seconds and using https://github.com/eriktheV-king/TTGO_T-beam_GPS-reset as part of the transmiter sketch.

Efective range in city center is 200 to 400 meters.
