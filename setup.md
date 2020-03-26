# Setup

These setup instructions assume that you have already added the [Robotic Vision repositories](https://roboticvisionorg.github.io/docs/)

## Requirements

- Ubuntu 18.04
- ROS Melodic

## Installing ROS

Follow the installation instructions provided for [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu#Installation).

*Note: We recommend installing either the desktop or desktop-full distribution*

## Setting up the Real-Time Kernel

To install the real-time kernel run the following command:

```bash
sudo apt install linux-image-5.4.13-rt7 linux-headers-5.4.13-rt7
```

Add the user to the realtime group

```bash
sudo addgroup realtime
sudo usermod -a -G realtime $(whoami)
```

Add the following limits to the realtime group in `/etc/security/limits.conf`

```
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock 102400
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock 102400
```

Reboot your machine then run the following command to make sure that you are using the RT kernel.

```bash
uname -a
```

The output of the command should contain 5.0.21-rt16 #1 SMP PREEMPT RT

## Installing the RV Panda Driver

Install the RV Panda Driver by running the command:

```bash
sudo apt install ros-melodic-rv-panda-driver
```