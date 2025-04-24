# Robot Assembly Description (assem_urdf)

Mô tả mô hình URDF của robot và hiển thị trong RViz2 sử dụng ROS 2.

## Cấu trúc thư mục

- `urdf/`: Chứa các file mô tả URDF của robot

  - `assem_urdf.urdf.xacro`: File mô tả chính của robot

- `launch/`: Chứa các file launch

  - `display.launch.py`: File khởi chạy RViz2 với robot state publisher và joint state publisher

- `rviz/`: Chứa cấu hình RViz
  - `assem_urdf.rviz`: Cấu hình RViz2 với đầy đủ comment giải thích

## Cài đặt

### Yêu cầu

- ROS 2 (đã thử nghiệm trên ROS 2 Humble)
- joint_state_publisher_gui package
- robot_state_publisher package
- rviz2 package

### Build

1. Tạo workspace (nếu chưa có):

```bash
mkdir -p ~/robot_check_ws/src
cd ~/robot_check_ws/src
```

2. Clone repository:

```bash
git clone https://github.com/YOUR_USERNAME/assem_urdf.git
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

Để hiển thị robot trong RViz:

```bash
ros2 launch assem_urdf display.launch.py
```

## Cấu trúc URDF

Mô hình robot được xây dựng sử dụng URDF và Xacro. Cấu trúc xacro được thiết kế để dễ dàng mở rộng và tùy chỉnh.

## Lưu ý

- joint_state_publisher_gui cung cấp giao diện để điều khiển các khớp của robot
- robot_state_publisher nhận thông tin từ joint_state_publisher_gui và xuất bản các transformation (tf) giữa các link
- RViz2 được cấu hình sẵn để hiển thị mô hình robot và các transformation
