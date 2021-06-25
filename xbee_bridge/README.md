# XBee communication test

On GCS:
```
$ roslaunch xbee_bridge gcs.launch
$ rosnode info /xbee_gcs
--------------------------------------------------------------------------------
Node [/xbee_gcs]
Publications:
 * /xbee_gcs/receive/xbee_uav [std_msgs/String]

Subscriptions:
 * /xbee_gcs/send/xbee_uav [unknown type]
$ rostopic echo /xbee_gcs/receive/xbee_uav
```


On UAV:
```
$ roslaunch xbee_bridge uav.launch
$ rosnode info /xbee_uav
--------------------------------------------------------------------------------
Node [/xbee_uav]
Publications:
 * /xbee_uav/receive/xbee_gcs [std_msgs/String]

Subscriptions:
 * /xbee_uav/send/xbee_gcs [unknown type]
$ rostopic echo /xbee_uav/receive/xbee_gcs
```

Send message from GCS:
```
$ rostopic pub -1 /xbee_gcs/send/xbee_uav std_msgs/String "data: 'Hello from GCS to UAV'"
```
