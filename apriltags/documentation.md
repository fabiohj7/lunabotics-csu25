<h1> April Tag Detection and Calibration </h1>

I reworked the original april tag code into a few different files for easier use.
I also included an object oriented version where you can easily import a DistanceDetector into your code 

-----------

<h2> Setting up your environment </h2>

if you dont have python3.11, download it:\
`brew install python@3.11 #for mac users`  

<h3> create a virtual environment for this project and source it: </h3>
    mkdir -p ~/.pyenvs/apriltags  
    python3.11 -m venv ~/.pyenvs/apriltags  
    source ~/.pyenvs/apriltags/bin/activate  

<h3> Install the proper modules </h3>
`pip install opencv-python`\
`pip install pupil-apriltags`

------------

<H2> Using the Code </h2>

To calibrate for a specific camera, run\
`python calibrate_camera.py`\
and let the code take 15 pictures of your printed grid\\

To include the code in a script, import the DistaceDetector class at the top of your file\
`from distance_detector import DistanceDetector`\
then, when you want to scan for an image you just need to create a distance detector object and call detect_tags:\
For example, to create a distance detector for camera 0, tags of size 125mm and camera parameters at the path ./camera_params.json, you would call:\
`dd = DistanceDetector(0, .125, 'camera_params.json')` \
Then simply call (with the optional boolean argument to show the image taken):\
`dd.detect_tags(visualize=True)`




