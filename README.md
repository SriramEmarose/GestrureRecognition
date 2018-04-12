# GestrureRecognition

Demo at: https://www.youtube.com/watch?v=W4Afc9qcj4I

Implements a simple Convex Hull based gesture recognition and Raspberry Pi GPIO toggling.

Segmentation for convex hull is basic thresholding, hence susceptible to noises.
Take care to run in a controlled environment or feel free to add a better segmentation and use it

# Raspberry Pi connection details,
For 5 fingers, Pin number 38 in board will be high
For 3 fingers, Pin number 40 in board will be high
Both pin Low when fingers closed

# Dependencies
OpenCV     [Linux/Rpi Install: sudo apt-get libopencv-dev ]  </br>
WiringPi   [Installation: http://wiringpi.com/download-and-install/ ]

# Compiling the code
g++ GestureRecognition_Rpi.cpp -o gesture `pkg-config --libs opencv` -std=c++11

# Running the code
./gesture
