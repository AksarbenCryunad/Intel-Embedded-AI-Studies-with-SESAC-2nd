# 🚌 TAYO-AutoDrive-X 

[![License](https://img.shields.io/badge/license-MIT-blue)]()
[![Stack](https://img.shields.io/badge/Tech_Stack-ROS2%2C%20YOLOv5%2C%20Python-ff69b4)](https://github.com/your-repo/your-project)
[![Status](https://img.shields.io/badge/Status-In%20Progress-yellowgreen)]()

> 💡 **한 줄 소개**: 자율주행 핵심 기능 (**인식, 제어, 경로 계획**)을 통합한 **ROS2 기반** 팀 프로젝트입니다.

---

## 🧭 목차

1.  [프로젝트 개요](#프로젝트-개요)
2.  [주요 기능](#주요-기능)
3.  [아키텍처](#아키텍처)
4.  [설치 및 실행](#설치-및-실행)
5.  [YOLOv5 ROS2 노드 실행](#yolov5-ros2-노드-실행)
6.  [데이터 & 모델](#데이터--모델)
7.  [팀원](#팀원)
8.  [라이선스](#라이선스)

---

## 📚 프로젝트 개요

| 구분 | 내용 |
| :--- | :--- |
| **목표** | **ROS2 Humble** 기반 자율주행 파이프라인 설계 및 구현 |
| **핵심 스택** | Python, **ROS2**, OpenCV, **YOLOv5**, Docker |
| **대상** | 자율주행 로봇 또는 시뮬레이션 환경 |

---

## ✨ 주요 기능

이 프로젝트의 핵심 모듈은 다음과 같습니다.

* **차선 인식 (Lane Detection)**: **OpenCV**와 **Depth Map** 정보를 활용한 정확한 차선 인식.
* **객체 감지 (Object Detection)**: 최신 **YOLOv5** 모델을 이용한 실시간 주변 환경 객체 감지.

---

## 🏗️ 아키텍처

전체 시스템의 구성 요소와 데이터 흐름은 아래 다이어그램과 같습니다.

![Architecture Diagram](./docs/architecture.png)

1.  **센서 드라이버**: RGB 카메라 및 Depth 카메라 등 센서 데이터 취합.
2.  **인식 노드**: 처리된 센서 데이터를 **토픽으로 퍼블리시 (Publish)**.
3.  **제어 노드**: 인식 결과를 바탕으로 로봇의 **액추에이터 명령**을 생성.

---

## 🛠️ 설치 및 실행

### 1. Set up(환경 설정)

프로젝트에 필요한 Python 가상 환경 및 의존성을 설치합니다.

```bash
# Python 가상환경 생성 및 활성화
python3 -m venv .venv
source .venv/bin/activate
(.venv) pip install -r requirements.txt
```

### 2. Build ROS2 workspace(ROS2 워크스페이스 빌드)
```bash
(docker) cd ros2_ws
(docker) colcon build --packages-select autonomous --symlink-install
(docker) source install/setup.zsh
```

## 3. Run auto_driving(ROS2 launch file 실행)

* **Terminal 1 자율주행 실행**
```bash
bash docker_shell.sh
(docker) ros2 launch autonomous auto_driving.launch.py
```

* **Terminal 1-1 차선인식 기능 부분 실행**
```bash
bash docker_shell.sh
(docker) ros2 launch autonomous auto_driving.launch.py only_line_follow=true
```

* **Terminal 2 result_img check(카메라 화면 확인)**
```bash
bash docker_shell.sh
(docker) rqt
```

## ▶️ YOLOv5 ROS2 노드 실행

## 🖼️ 데이터 & 모델

## 🧑‍💻 팀원

이 프로젝트의 팀원은 다음과 같습니다.

| 역할 | 이름 | 담당 파트 | GitHub |
| :--- | :--- | :--- | :--- |
| **팀장** | **남대문** | 미정 | [![남대문](https://img.shields.io/badge/남대문-darkblue)](https://github.com/AksarbenCryunad) |
| **팀원** | **최종인** | 미정 | [![최종인](https://img.shields.io/badge/최종인-darkgreen)](https://github.com/ChoiJonginhub) |
| **팀원** | **김유광** | 미정 | [![김유광](https://img.shields.io/badge/김유광-skyblue)](https://github.com/daheung) |


## ⚖️ 라이선스

# YOLOv5 ROS2 Subscription Node
