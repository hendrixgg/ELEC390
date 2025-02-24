# Accessing the Pi
Instructions are identical on windows and linux:
1. `ssh elec390@team2pi.local`
# Raspberry Pi MAC Address
```
> ip link show
1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN mode DEFAULT group default qlen 1000
    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
2: eth0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc mq state UP mode DEFAULT group default qlen 1000
    link/ether d8:3a:dd:e7:af:c6 brd ff:ff:ff:ff:ff:ff
3: wlan0: <NO-CARRIER,BROADCAST,MULTICAST,UP> mtu 1500 qdisc pfifo_fast state DOWN mode DEFAULT group default qlen 1000
    link/ether 72:5e:9d:ac:0f:e9 brd ff:ff:ff:ff:ff:ff
```
from the above command line interaction, the MAC Address is `72:5e:9d:ac:0f:e9`.