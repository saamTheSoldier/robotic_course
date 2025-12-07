# HW2 Robotics Localization

مسیر کار: `/home/saam/courses/robotics/hw`

## پیش‌نیاز
- ROS2 (نصب شده روی سیستم)
- Python venv: `.venv_hw2`
- بسته‌های apt: `texlive-xetex texlive-lang-other texlive-latex-extra`

## راه‌اندازی سریع
```bash
cd /home/saam/courses/robotics/hw
source .venv_hw2/bin/activate
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## اجرای کامل (شبیه‌سازی + رسم + PDF)
```bash
bash hw2_run_all.sh
```
این اسکریپت شبیه‌سازی گازیبو، نودهای پیش‌بینی/اندازه‌گیری/EKF، تست مسیر مستطیلی، رسم نمودارها و کامپایل لاتک را انجام می‌دهد. خروجی‌ها در `robotic_course/report/` است.

## مسیرهای مهم
- URDF با موتور مستقل و نویز IMU: `robotic_course/robot_description/src/description/robot.urdf`
- نود C++ تبدیل `cmd_vel` به RPM: `robotic_course/robot_description/scripts/motor_command.cpp`
- پکیج پایتون EKF: `robotic_course/src/robot_autonomy/robot_local_localization/`
- لانچ اصلی: `robotic_course/robot_description/launch/gazebo.launch.py`
- اسکریپت رسم: `robotic_course/scripts/plot_results.py`
- گزارش PDF: `Exercise-Template-wcover (1)/main.pdf` و کپی در `robotic_course/report/HW2_solution.pdf`

## اجرای دستی نودها
```bash
ros2 launch robot_description gazebo.launch.py
ros2 run robot_local_localization prediction_node
ros2 run robot_local_localization measurement_node
ros2 run robot_local_localization ekf_node
ros2 run robot_local_localization test_node
```

## خروجی‌ها
- CSV: `robotic_course/report/csvs/`
- نمودارها: `robotic_course/report/figures/trajectory.png`, `error.png`, `theta.png`
- وضعیت: `robotic_course/report/status.txt`

