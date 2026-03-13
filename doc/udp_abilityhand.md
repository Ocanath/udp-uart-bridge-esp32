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

### Ability Hand Configuration

The Ability Hand must be in UART mode with PPP byte stuffing enabled. To do this:

```bash
Wo
We16
We46
We47
```

- The TX pin of the ESP32 must be wired to hand RX, and the RX pin of the ESP32 must be wired to hand TX. Grounds must obviously be shared.

- A zener diode protection network (equivalent to the one in the hand board - 10ohm resistor with 3.3V zener) is used to prevent destruction of the ESP32 from hot plugging hands. **DO NOT HOT PLUG HANDS**. Make sure the hand is fully powered off before removing from the socket.

- This demo uses an amazon 3.3V buck converter which is wired to VUSB and GND of the ESP32 - it is recommended to source a different converter for this because the one used in the demo is noisy and causes interesting interactions with the built-in 1S charging IC in the Xiao ESP32S3. 

### Laptop Configuration

You must ensure the laptop and bridge are on the same network, and that there are no firewall rules blocking your selected port.