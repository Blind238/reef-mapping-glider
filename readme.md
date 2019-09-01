We're developing a drone to help map the coral reefs around Aruba.

This includes a hardware component, based on [Alex Williams' design](https://hackaday.io/project/20458-open-source-underwater-glider) ([github](https://github.com/alex-williams-2150/underwater-glider), [onshape](https://cad.onshape.com/documents/23c169f94c57ba9867aa3c3f/w/62170f6efac11f679de9c4fe/e/8302463fee19e14abe453dcc)), and a software component to analyze images and create meaningful outputs.

So far we have made a few modifications to the drone design, and are almost finished constructing it. We're just starting with the programming of the electronics, which use a PrintrBoard Rev. F5 running marlin to control the motors, and a Raspberry Pi and noIR camera to collect images and control the PrintrBoard. [Here](https://cad.onshape.com/documents/ce8cabb952959788f72162f0/w/012f00113c8357c5636e2f60) is the link to the modified onshape document, containing all but one of the printed files. The only one missing is the SCAD file soon to be uploaded here separately.

We're planning to use a trial of IBM Watson for our first round of image classification, but we'd like to use Tensor Flow later as a more open AI option. We're also talking with the local park to figure out the best format for the output data.

Raspberry Pi preparation:

libraries to install via apt-get:
python-smbus python-serial

raspi-config options:
enable i2c
enable camera
disable serial console
