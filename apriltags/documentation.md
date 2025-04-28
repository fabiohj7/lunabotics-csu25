I reworked the original april tag code into a few different files for easier use.

-----------

For starters, to use this code make a python virtual environment in version 3.11 and download the required modules (Described below) 

if you dont have python3.11, download it:
brew install python@3.11 #for mac users

<h3> create a virtual environment for this project and source it: </h3>
mkdir -p ~/.pyenvs/apriltags
python3.11 -m venv ~/.pyenvs/apriltags
source ~/.pyenvs/apriltags/bin/activate

<h3> Install the proper modules </h3>
pip install opencv-python
pip install pupil-apriltags

