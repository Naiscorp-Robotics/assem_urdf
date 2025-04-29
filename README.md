# Robot Assembly Description (assem_urdf)

Mô tả mô hình URDF của robot với hỗ trợ ROS 2 Humble và Ignition Fortress, bao gồm mô phỏng và điều khiển.

## Cấu trúc thư mục

- `urdf/`: Chứa các file mô tả URDF của robot

  - `assem_urdf.urdf.xacro`: File mô tả chính của robot với cấu hình màu sắc và comments

- `launch/`: Chứa các file launch

  - `display.launch.py`: File khởi chạy với Ignition Gazebo, RViz2, robot state publisher và các controller

- `config/`: Chứa các file cấu hình

  - `diff_drive_control_velocity.yaml`: Cấu hình điều khiển vi sai cho 4 bánh xe
  - `ekf.yaml`: Cấu hình bộ lọc Kalman mở rộng cho localization

- `rviz/`: Chứa cấu hình RViz

  - `assem_urdf.rviz`: Cấu hình RViz2 với đầy đủ comment giải thích

- `worlds/`: Chứa các môi trường mô phỏng

  - `empty.sdf`: Môi trường trống
  - `bookstore.sdf`: Môi trường mô phỏng nhà sách

- `meshes/`: Chứa các file mesh cho mô hình 3D

## Cài đặt

### Yêu cầu

- ROS 2 Humble
- Ignition Fortress
- Các package cần thiết:
  - ros_gz_bridge
  - ros_gz_sim
  - ros2_control
  - robot_localization
  - controller_manager
  - diff_drive_controller
  - rviz2
  - joint_state_publisher
  - robot_state_publisher
  - xacro

### Build

1. Tạo workspace (nếu chưa có):

```bash
mkdir -p ~/robot_check_ws/src
cd ~/robot_check_ws/src
```

2. Clone repository:

```bash
git clone https://github.com/Naiscorp-Robotics/assem_urdf.git
```

3. Build workspace:

```bash
cd ~/robot_check_ws
colcon build
```

4. Source workspace:

```bash
source ~/robot_check_ws/install/setup.bash
```

## Sử dụng

### Hiển thị robot trong RViz và Ignition Gazebo:

```bash
ros2 launch assem_urdf display.launch.py
```

### Các tùy chọn launch:

- `use_sim_time`: Sử dụng thời gian mô phỏng (mặc định: false)
- `use_localization`: Bật/tắt localization (mặc định: true)
- `run_headless`: Chạy Ignition Gazebo ở chế độ không giao diện (mặc định: false)
- `world_file_name`: Tên file world (mặc định: empty.sdf)

## Tính năng

- Mô phỏng robot trong Ignition Gazebo
- Điều khiển vi sai cho 4 bánh xe
- Localization sử dụng EKF
- Bridge giữa ROS2 và Ignition cho:
  - LiDAR (/scan)
  - IMU (/imu)
  - Điều khiển vận tốc (/cmd_vel)
  - Odometry (/odom)
  - Clock synchronization

## Cấu trúc URDF

Mô hình robot được xây dựng sử dụng URDF và Xacro với các thành phần:

- Base link: Thân chính của robot
- 4 bánh xe: Điều khiển vi sai
- LiDAR: Cảm biến khoảng cách
- Cánh tay (trái/phải): Khớp quay
- Chân (trái/phải): Khớp quay
- Đầu: Khớp quay

## Lưu ý

- Các controller được tự động load sau khi spawn robot
- Topic relay được cấu hình để chuyển đổi giữa các topic ROS2 và Ignition
- EKF được sử dụng để kết hợp dữ liệu từ IMU và odometry
- RViz2 được cấu hình sẵn để hiển thị mô hình robot và các transformation
