# Path Record Node

recording path base on distance and angular motion filter, Steps for usage:

1. launch node just launch:

```asm
ros2 launch path_record path_record.launch.py record_with_start:=True
```
2. Param please look at path_record.yaml in params folder

3. call service to start recording

```asm
ros2 service call /path_record_server/path_record path_record/srv/PathRecord "{ request_code: 0 }"
```

request_code：   
0： start record path  
1: stop record path  
2: reset record path  
3: get record path   