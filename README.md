# alpaga-ros

## Install

- Ubuntu 16.04.7 LTS (Xenial)
- ROS Kinetic

cf.  https://downloads.ubiquityrobotics.com/pi.html
```2020-11-07-ubiquity-xenial-lxde 	8585c3839e2405163302f0fef774c1caa22b4ebc6f2582d9266a30ecef6987b4 ```

- Python dependencies

```
git clone https://github.com/cocagne/pysrp.git
cd pysrp
sudo python setup.py install

git clone -b xtend-python-2 https://github.com/alexglzg/xbee-python.git
cd xbee-python
sudo python setup.py install
```

## Specific Code

```
cd ~/catkin_ws/src
git clone https://github.com/ceri-num/alpaga-ros.git
```

- *alpaga-ros/xbee_bridge* adapted from https://github.com/BrunoScaglione/usv_comms.git
    used to tunnel topics between UAV and GCS
    both UAV and GCS have their own ROS Master
    cf. [xbee_bridge/]
