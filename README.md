# robotic_arm

# Install
First create a virtual environment in this repo using
`python3 -m venv venv`
Activate the environment:
`source venv/bin/activate`
Update pip:
`pip install --upgrade pip`
`sudo apt-get install build-essential libgtk-3-dev`
Install requirements
`pip install -r requirements.txt`

## Installing kinect
Need libusb: sudo apt-get install libusb-1.0-0-dev
Install libfreenect:
```
git clone https://github.com/OpenKinect/libfreenect
cd libfreenect
mkdir build
cd build
cmake .. -DBUILD_PYTHON3=ON
make
sudo make install
sudo ldconfig /usr/local/lib64/
```
```
cd ../../
cd libfreenect/wrappers/python
python setup.py install
```
```
git clone https://github.com/amiller/libfreenect-goodies.git
cd libfreenect-goodies/
```