# robotic_arm

# Run RL
`python RL/train.py --domain_name FetchPickAndPlace-v1 \
  --reward_type sparse --cameras 8 10 --frame_stack 1 --num_updates 1 \
  --observation_type pixel --encoder_type pixel --work_dir ./data/FetchPickAndPlace-v1 \
  --pre_transform_image_size 100 --image_size 84 --agent rad_sac \
  --seed -1 --critic_lr 0.001 --actor_lr 0.001 --eval_freq 1000 --batch_size 128 \
  --num_train_steps 200000 --save_tb --save_video --demo_model_dir expert/FetchPickAndPlace-v1 \
  --demo_model_step 195000 --demo_samples 500 \
  --warmup_cpc 1600 --warmup_cpc_ema \
  --demo_special_reset grip --change_model`
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
## Installing pytorch and mujoco
`python -m pip install torch==1.8.1+cu111 torchvision==0.9.1+cu111 -f https://download.pytorch.org/whl/torch_stable.html`  
https://github.com/openai/mujoco-py  
`wget https://mujoco.org/download/mujoco210-linux-x86_64.tar.gz`

`pip install mujoco-py<2.2,>=2.1`  
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
## Install robotic arm environment
Also needs to be used after updating the code
`pip install -e ./gym-robotic_arm/`

## arduino
Install pwm servo driver: https://forum.arduino.cc/t/adafruit-pwm-servo-driver-exists-but-does-not-see/509542
You need to add the Adafruit PWM Servo driver library. Go into the Library Manager and search for it.
Sketch --> Include Library -->Manage Libraries
## Setup Com port on wsl2
 https://devblogs.microsoft.com/commandline/connecting-usb-devices-to-wsl/
To attach busid. Restart wsl. Sent random file to the com port using arduino ide. Than run the command.
Check device is visible in ubuntu: `lsusb`
Enable permissions in ubuntu to access: `sudo chmod 666 /dev/tty3`
## Install cuda on ubuntu
https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=20.04&target_type=runfile_local
# Errors
If you get `cannot import name 'Serial' from 'serial' ` do  
  ```
  python -m pip uninstall serial
  python -m pip install pyserial  
  ```
