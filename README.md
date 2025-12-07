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

# گزارش کامل برای تحویل تکلیف

## آنچه انجام شده
- مدل ربات در `robotic_course/robot_description/src/description/robot.urdf` به دو موتور مستقل با کنترل RPM تغییر کرد و نویز IMU افزوده شد.
- نود C++ `cmd_vel_to_motor` (`robotic_course/robot_description/scripts/motor_command.cpp`) سرعت خطی/زاویه‌ای را به RPM چرخ‌ها تبدیل می‌کند.
- پکیج پایتون `robot_local_localization` شامل چهار نود پیش‌بینی، اندازه‌گیری، EKF، و تست مسیر مستطیلی پیاده‌سازی شد؛ پارامترها در `config/ekf_params.yaml`.
- لانچ `robotic_course/robot_description/launch/gazebo.launch.py` گازیبو، پل ROS-GZ، VO، موتور و نودهای محلی‌سازی را بالا می‌آورد.
- اسکریپت خودکار `hw2_run_all.sh` ساخت، اجرا، جمع‌آوری CSV، رسم نمودارها و کامپایل لاتک را انجام می‌دهد.
- گزارش لاتک در `Exercise-Template-wcover (1)/main.tex` تکمیل و PDF در همان پوشه و کپی در `robotic_course/report/HW2_solution.pdf` قرار دارد.

## مراحل اجرا (یک‌خطی)
```bash
cd /home/saam/courses/robotics/hw
source .venv_hw2/bin/activate
source /opt/ros/${ROS_DISTRO}/setup.bash
bash hw2_run_all.sh
```

## نتایج و فایل‌های خروجی
- PDF نهایی: `Exercise-Template-wcover (1)/main.pdf` و `robotic_course/report/HW2_solution.pdf`
- CSV مسیرها: `robotic_course/report/csvs/` (motion/measurement/ekf/ground_truth)
- نمودارها: `robotic_course/report/figures/trajectory.png`, `error.png`, `theta.png`
- وضعیت عددی: `robotic_course/report/status.txt` (RMSE مسیرها)

## اگر نیاز به اجرا/بازبینی دستی است
1. گازیبو و پل: `ros2 launch robot_description gazebo.launch.py`
2. نودهای محلی‌سازی:
   - `ros2 run robot_local_localization prediction_node`
   - `ros2 run robot_local_localization measurement_node`
   - `ros2 run robot_local_localization ekf_node`
   - `ros2 run robot_local_localization test_node`
3. رسم دستی اگر CSV دارید: `python robotic_course/scripts/plot_results.py`

## موارد باقی‌مانده/چک‌لیست
- اطمینان از نصب فونت‌های فارسی XeLaTeX (در صورت خطا: `echo "1273588312" | sudo -S apt install -y texlive-lang-arabic`).
- در صورت تغییر تاپیک‌های VO/IMU، نام تاپیک‌ها را در `measurement_node.py` و لانچ تنظیم کنید.
- برای انتشار به ریموت، ابتدا `git status` و سپس `git push` به شاخه مورد نظر. 
