# Configuration for Ability Hand

## Introduction

This device firmware can be configured to act as a bridge for udp/tcp sockets and an ability hand. It requires proper configuration.

### Network Configuration 

First and foremost, you must connect the device to the network. Network configuration can be found in the [top level readme](https://github.com/Ocanath/wifi-uart-bridge-esp32/blob/master/README.md). Connect to your network (must be 2.4Ghz), configure your static ip and port. Make sure those settings match your networking communication node, then continue to the next steps.

Recommended flow:

```bash
setssid <NetworkSSID2Ghz>
setpwd <NetworkPassword>
restart
ipconfig
setstaticip <current device ip>
setport 5067
ipconfig
```

Your ipconfig response should read like this:
```bash
Connected to: NetworkSSID2Ghz
UDP server on port: 5067
TCP server on port: 5067
No TCP client connected
Server Response Offset: 0
IP address: <valid ip address>
Requested Static IP: <same valid ip address as line above>
Subnet Mask: 255.255.255.0
Gateway: 0.0.0.0
DNS: 0.0.0.0
```

### Hardware Configuation 

Send the following commands to the device over serial terminal. 

```bash
setbaudrate 460800
setencoding 1
```


This configures the hand to recognize PPP frame characters for forwarding uart frames, sets the uart baudrate to the hand default, and gives you led activity that maps to network traffic activity.
