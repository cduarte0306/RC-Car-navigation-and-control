# About
The following is a Linux user-space application for RC car controls. This includes anything from GPS navigation to machine vision for the RC autonomous car.

# How to run
1. Clone the repository:
   ```bash
   git clone
2. Navigate to the project directory:
   ```bash
   cd rc_car_control
   ```
3. Build the project:
   ```bash
   make
   ```
4. Run the application (qemu):
   ```bash
   qemu-aarch64 -L /opt/poky/4.0.26/sysroots/armv8a-poky-linux ./build/src/rc-car-nav
   ```

# Tests
1. Create virtual environment and system site packages:
```bash
python3 -m venv --system-site-packages venv
```

2. Activate the virtual environment:
```bash
source venv/bin/activate
```

2. Install required python dependencies:
```bash
pip install -r requirements.txt
```

### To run the ML mode
#### 1. Install mmsegmentation + dependencies
pip install -U openmim
mim install mmengine
mim install "mmcv>=2.0.0"
pip install "mmsegmentation>=1.0.0"

#### 2. Download the model (pick one):
#### Fast (ResNet18):
mim download mmsegmentation --config deeplabv3plus_r18-d8_4xb2-80k_cityscapes-512x1024 --dest checkpoints/

#### Or accurate (ResNet101):
mim download mmsegmentation --config deeplabv3plus_r101-d8_4xb2-80k_cityscapes-512x1024 --dest checkpoints/

#### 3. Run it
python test_sidewalk_cityscapes.py --input your_video.MOV --model-size small

### Diagram
<img width="271" height="825" alt="image" src="https://github.com/user-attachments/assets/009a22b3-0706-4b36-97f1-5b67818c2270" />
