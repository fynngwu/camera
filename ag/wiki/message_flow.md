# Message Flow

## 1. Path Publish

1. Ground station plans a polyline via A*
2. Ground station sends:

```json
{"type":"path","seq":1,"points":[[0,0],[0.8,0.2]],"target_speed":0.2}
```

3. Sky validates and ACKs
4. Sky smooths and resamples path

## 2. Control Loop

1. Sky control thread updates virtual pose (odom integration)
2. Sky chooses lookahead target index
3. Sky generates `cmd_vel(vx, vy, wz)`
4. Sky sends command to ground robot
5. Ground robot prints command and ACKs

## 3. Telemetry Push

1. Sky periodically emits telemetry to ground station
2. Ground station updates pose, cmd numbers, and arrow visualization

```json
{
  "type":"telemetry",
  "seq":18,
  "pose":{"x":0.21,"y":-0.03,"yaw":0.12},
  "cmd":{"vx":0.19,"vy":0.0,"wz":0.08},
  "target_index":7,
  "goal_reached":false
}
```

## 4. Goal Completion

1. Sky detects `goal_distance <= goal_tolerance`
2. Sky outputs zero command (`vx=vy=wz=0`)
3. Telemetry sets `goal_reached=true`
