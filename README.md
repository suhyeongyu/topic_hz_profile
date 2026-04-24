# `ros2 topic hz` QoS Patch

`ros2 topic hz` CLI에 QoS 조절 옵션을 추가한 패치.

## 추가된 옵션

| 옵션 | 기본값 | 선택지 |
|------|--------|--------|
| `--qos-depth` | `100` | 양의 정수 |
| `--qos-reliability` | `best_effort` | `best_effort`, `reliable`, `system_default` |
| `--qos-durability` | `volatile` | `volatile`, `transient_local`, `system_default` |
| `--qos-history` | `keep_last` | `keep_last`, `keep_all`, `system_default` |

## 설치

```bash
HZ_PATH=$(python3 -c "import ros2topic.verb.hz; print(ros2topic.verb.hz.__file__)")
cp "$HZ_PATH" "$HZ_PATH.bak"
cp hz.py "$HZ_PATH"
```

복구:
```bash
cp "$HZ_PATH.bak" "$HZ_PATH"
```

## 사용

```bash
# 기본
ros2 topic hz /topic

# depth 조절
ros2 topic hz /topic --qos-depth 500

# reliable publisher
ros2 topic hz /topic --qos-reliability reliable

# latched topic
ros2 topic hz /robot_description \
    --qos-reliability reliable --qos-durability transient_local
```
