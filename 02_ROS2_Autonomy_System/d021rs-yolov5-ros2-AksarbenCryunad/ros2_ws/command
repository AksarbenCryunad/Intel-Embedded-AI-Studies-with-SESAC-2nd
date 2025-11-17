# 모든 ROS 노드 종료
~/.stop_ros.sh

cd ~/ros2_ws 
# 전체 빌드
#colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
# 특정 패키지만 빌드
colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install --packages-select xxx

# 보정(Calibration)

# 선형 속도 보정 (ROSMentor_Mecanum, MentorPi_Acker)
ros2 launch calibration linear_calib.launch.py

# 각속도 보정 (ROSMentor_Mecanum)
ros2 launch calibration angular_calib.launch.py

# IMU 보정
ros2 launch ros_robot_controller ros_robot_controller.launch.py
ros2 run imu_calib do_calib --ros-args -r imu:=/ros_robot_controller/imu_raw --param output_file:=/home/ubuntu/ros2_ws/src/calibration/config/imu_calib.yaml

# IMU 보정 효과 확인
ros2 launch peripherals imu_view.launch.py

# 센서 시각화

# 깊이 카메라 포인트 클라우드 시각화 (ascamera)  
# 깊이 카메라 RGB 이미지 시각화 (ascamera)
ros2 launch peripherals depth_camera.launch.py
rviz2

# 단일 카메라 시각화
ros2 launch peripherals usb_cam.launch.py
rviz2

# 라이다 데이터 시각화
ros2 launch peripherals lidar_view.launch.py

# 라이다 기능

# 라이다 실행
ros2 launch app lidar_node.launch.py debug:=true

# 라이다 장애물 회피 (ROSMentor_Mecanum, MentorPi_Acker)
ros2 service call /lidar_app/enter std_srvs/srv/Trigger {}
ros2 service call /lidar_app/set_running interfaces/srv/SetInt64 "{data: 1}"

# 라이다 추종 (ROSMentor_Mecanum, MentorPi_Acker)
ros2 service call /lidar_app/enter std_srvs/srv/Trigger {}
ros2 service call /lidar_app/set_running interfaces/srv/SetInt64 "{data: 2}"

# 라이다 경계 모드 (ROSMentor_Mecanum)
ros2 service call /lidar_app/enter std_srvs/srv/Trigger {}
ros2 service call /lidar_app/set_running interfaces/srv/SetInt64 "{data: 3}"

# 영상 기반 기능

# 라인 추종
ros2 launch app line_following_node.launch.py debug:=true
ros2 service call /line_following/enter std_srvs/srv/Trigger {}
# 마우스 좌클릭으로 화면에서 색상 추출
ros2 service call /line_following/set_running std_srvs/srv/SetBool "{data: True}"

# 객체 추적
ros2 launch app object_tracking_node.launch.py debug:=true
ros2 service call /object_tracking/enter std_srvs/srv/Trigger {}
# 마우스 좌클릭으로 화면에서 색상 추출
ros2 service call /object_tracking/set_running std_srvs/srv/SetBool "{data: True}"

# 손 제스처 인식
ros2 launch app hand_gesture_node.launch.py debug:=true
ros2 service call /hand_gesture/enter std_srvs/srv/Trigger {}
ros2 service call /hand_gesture/set_running std_srvs/srv/SetBool "{data: True}"

# 예제 기능 (Python 실행)

# QR 코드 생성
cd ~/ros2_ws/src/example/example/qrcode && python3 qrcode_creater.py

# 깊이 카메라 서비스 실행 필요
ros2 launch peripherals depth_camera.launch.py

# QR 코드 검출
cd ~/ros2_ws/src/example/example/qrcode && python3 qrcode_detecter.py

# 얼굴 검출
cd ~/ros2_ws/src/example/example/mediapipe_example && python3 face_detect.py

# 얼굴 메시
cd ~/ros2_ws/src/example/example/mediapipe_example && python3 face_mesh.py

# 손 관절 검출
cd ~/ros2_ws/src/example/example/mediapipe_example && python3 hand.py

# 신체 관절 검출
cd ~/ros2_ws/src/example/example/mediapipe_example && python3 pose.py

# 배경 분리
cd ~/ros2_ws/src/example/example/mediapipe_example && python3 self_segmentation.py

# 전체 인식(Holistic)
cd ~/ros2_ws/src/example/example/mediapipe_example && python3 holistic.py

# 3D 물체 인식
cd ~/ros2_ws/src/example/example/mediapipe_example && python3 objectron.py

# 손가락 궤적
cd ~/ros2_ws/src/example/example/mediapipe_example && python3 hand_gesture.py

# 색상 인식
cd ~/ros2_ws/src/example/example/color_detect && python3 color_detect_demo.py

# 자율주행 & SLAM

# 자율주행
ros2 launch example self_driving.launch.py

# 2D 지도 작성
ros2 launch slam slam.launch.py

# RViz에서 지도 확인
ros2 launch slam rviz_slam.launch.py

# 키보드 제어 (선택사항)
ros2 launch peripherals teleop_key_control.launch.py

# 지도 저장
cd ~/ros2_ws/src/slam/maps && ros2 run nav2_map_server map_saver_cli -f "map_01" --ros-args -p map_subscribe_transient_local:=true

# 3D 지도 작성 (ascamera)
ros2 launch slam rtabmap_slam.launch.py

# RViz에서 3D 지도 확인
ros2 launch slam rviz_rtabmap.launch.py

# 네비게이션

# 2D 네비게이션
# RViz에서 목표 지점 설정
ros2 launch navigation rviz_navigation.launch.py
ros2 launch navigation navigation.launch.py map:=지도이름

# 3D 네비게이션 (Dabai)
ros2 launch navigation rtabmap_navigation.launch.py

# RViz에서 목표 지점 설정
ros2 launch navigation rviz_rtabmap_navigation.launch.py

# 시뮬레이션

# URDF 시각화
ros2 launch jetrover_description ack.launch.py

# 도구

# 상위 PC에서 소프트웨어 실행 시 (앱 자동 실행 OFF일 때 카메라 서비스 필요)
ros2 launch peripherals depth_camera.launch.py

# Lab Tool 실행
python3 ~/software/lab_tool/main.py

# Servo Tool (PWM 서보 모터 보정)
python3 ~/software/servo_tool/main.py
