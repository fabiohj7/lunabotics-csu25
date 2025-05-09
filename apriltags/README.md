<h1> April Tag Detection and Calibration </h1>

I reworked the original april tag code into a few different files for easier use.
I also included an object oriented version where you can easily import a DistanceDetector into your code 

-----------

<h2> Setting up your environment </h2>

if you dont have python3.11, download it:

    brew install python@3.11 #for mac users

<h3> create a virtual environment for this project and source it: </h3>

    mkdir -p ~/.pyenvs/apriltags
    python3.11 -m venv ~/.pyenvs/apriltags
    source ~/.pyenvs/apriltags/bin/activate  

<h3> Install the proper modules </h3>

    pip install opencv-python
    pip install pupil-apriltags

------------

<H2> Using the Code </h2>

To calibrate for a specific camera, run
    
    python calibrate_camera.py
and let the code take 15 pictures of your printed grid

To include the code in a script, First, import the DistaceDetector class at the top of your file.
Next, create an instance of a `DistanceDetector` object.
Finally, to scan for an image, simply call the `detect_tags` method. You will first need to get a frame either by using a camera attached to a distancedetector
or from some other source  
To find the distance between tags, call `find_distance_to_tags`   
The final result will look something like this:  

    from distance_detector import DistanceDetector

    frame = dd.take_picture()
    tags = dd.detect_tags(frame)
    id, distance = dd.find_distance_to_tags(tags)
    dd.visualize_tags(tags, frame) # optional to show what the camera sees


