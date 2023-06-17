# esp-idf-gy85
A demo showing the pose of the GY-85 9DoF IMU sensor in 3D using esp-idf.   

GY-85 consists of the following three chips.   
- ADXL345 Accelermter   
- ITG3205 Gyroscope   
- QMC5883L Electronic Compass   

You can use the Kalman filter to estimate the Euler angle.   
Euler angles are roll, pitch and yaw.   
It's very intuitive and easy to understand.   
![a-Pitch-yaw-and-roll-angles-of-an-aircraft-with-body-orientation-O-u-v-original](https://user-images.githubusercontent.com/6020549/226072914-a7f923fc-eb6e-4d19-b2ff-8c9f2749ee6f.jpg)

First, calibrate the compass and find the offset value for each axis.   
As you can see, the X, Y and Z axes are quite off-center.   
![gy85-calib-1](https://user-images.githubusercontent.com/6020549/233240180-22a5bbc6-f25b-4d07-9910-1655c9a3964a.jpg)

Then use the sensor values to find the Euler angles.   
![gy85-euler](https://user-images.githubusercontent.com/6020549/233240284-2d3d56aa-b1a7-46df-b336-f1d295435601.JPG)

I used [this](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino) library.   


# Software requiment
ESP-IDF V4.4/V5.0.   
ESP-IDF V5.0 is required when using ESP32-C2.   

# Hardware requirements
GY-85 9DoF IMU Sensors.   

# Wireing
|GY-85||ESP32|ESP32-S2/S3|ESP32-C2/C3||
|:-:|:-:|:-:|:-:|:-:|:-:|
|VCC_IN|--|N/C|N/C|N/C||
|3V3|--|3.3V|3.3V|3.3V||
|GND|--|GND|GND|GND||
|SCL|--|GPIO22|GPIO12|GPIO5|(*1)|
|SDA|--|GPIO21|GPIO11|GPIO4|(*1)|
|M_DRDY|--|N/C|N/C|N/C||
|A_INT1|--|N/C|N/C|N/C||
|G_INT|--|N/C|N/C|N/C||

(*1)You can change it to any pin using menuconfig.   


# Caribrate compass
```
git clone https://github.com/nopnop2002/esp-idf-gy85
cd esp-idf-gy85/calibrate
idf.py set-target {esp32/esp32s2/esp32s3/esp32c2/esp32c3}
idf.py menuconfig
idf.py flash
```

### Configuration   
To find the offset value, set the compass offset to 0.   
![config-top](https://user-images.githubusercontent.com/6020549/233240375-7304bd5c-d232-439c-b667-1b0e3f7acfc2.jpg)
![config-app](https://user-images.githubusercontent.com/6020549/233240379-9c56a6ab-788d-4840-963e-8691718739e1.jpg)

### Execute calibration   
ESP32 acts as a web server.   
I used [this](https://github.com/Molorius/esp32-websocket) component.   
This component can communicate directly with the browser.   
It's a great job.   
Enter the following in the address bar of your web browser.   
```
http:://{IP of ESP32}/
or
http://esp32.local/
```

As you rotate the IMU it plots the X, Y and Z values.   
X, Y, Z offset are displayed.   

![gy85-calib-1](https://user-images.githubusercontent.com/6020549/233240180-22a5bbc6-f25b-4d07-9910-1655c9a3964a.jpg)

### Execute calibration again   
If you set the offset you got from the calibration and run it again, the circle position will change.   

![gy85-calib-2](https://user-images.githubusercontent.com/6020549/233240176-b656f16a-75ec-4546-9af3-f263a1574fbb.jpg)

# Get Euler angles from IMU
```
git clone https://github.com/nopnop2002/esp-idf-gy85
cd esp-idf-gy85/euler
idf.py set-target {esp32/esp32s2/esp32s3/esp32c2/esp32c3}
idf.py menuconfig
idf.py flash
```

### Configuration   
Sets the compass offset obtained by calibration.   

![config-top](https://user-images.githubusercontent.com/6020549/233240375-7304bd5c-d232-439c-b667-1b0e3f7acfc2.jpg)
![config-app](https://user-images.githubusercontent.com/6020549/233240379-9c56a6ab-788d-4840-963e-8691718739e1.jpg)

### View Euler angles with built-in web server   
ESP32 acts as a web server.   
I used [this](https://github.com/Molorius/esp32-websocket) component.   
This component can communicate directly with the browser.   
It's a great job.   
Enter the following in the address bar of your web browser.   
```
http:://{IP of ESP32}/
or
http://esp32.local/
```

![gy85-euler](https://user-images.githubusercontent.com/6020549/233240284-2d3d56aa-b1a7-46df-b336-f1d295435601.JPG)


WEB pages are stored in the html folder.   
I used [this](https://canvas-gauges.com/) for gauge display.   
I used [this](https://threejs.org/) for 3D display.   
You can change it as you like.   

### View Euler angles using PyTeapot   
You can view Euler angles using [this](https://github.com/thecountoftuscany/PyTeapot-Quaternion-Euler-cube-rotation) tool.   
It works as a UDP display server.   
This is a great application.   

```
+-------------+     +-------------+     +-------------+
|     IMU     | i2c |    ESP32    | UDP | pyteapot.py |
|             |---->|             |---->|             |
|             |     |             |     |             |
+-------------+     +-------------+     +-------------+
```

```
$ sudo apt install python3-pip python3-setuptools
$ python3 -m pip install -U pip
$ python3 -m pip install pygame
$ python3 -m pip install PyOpenGL PyOpenGL_accelerate
$ git clone https://github.com/thecountoftuscany/PyTeapot-Quaternion-Euler-cube-rotation
$ cd PyTeapot-Quaternion-Euler-cube-rotation
$ python3 pyteapot.py
```
The posture of your sensor is displayed.   
![gy-85_2023-04-20_11-17-50](https://user-images.githubusercontent.com/6020549/233240823-d094ca48-8025-4108-bae5-6bf87430a55c.png)

