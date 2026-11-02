# Проверка перед подключением Frob

Выполнить на фактическом роботе и ноутбуке; сведения публичного репозитория не заменяют эту проверку:

```bash
ros2 topic list -t
ros2 topic info /odom -v
ros2 topic info /odometry/filtered -v
ros2 topic info /imu/mpu6050 -v
ros2 topic echo /odometry/filtered --once
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo odom base_footprint
```

Проверить frame_id/child_frame_id, единицы скорости, ковариации, частоту и монотонность временных меток. Уточнить, существует ли `base_link` вместо `base_footprint` и кто публикует `odom → base_*`.

После проверки:

```bash
source install/setup.bash
ros2 run gnss_resilient_demo frob_adapter --ros-args \
  -p odometry_topic:=/odometry/filtered \
  -p base_frame:=base_footprint
```

До появления реального ground truth абсолютную ошибку в интерфейсе отключить. Не использовать оценку фильтра как «истину».
