# $\color{#00c853}{\text{ct joy အသုံးပြုနည်း}}$

ဤ ROS 2 Humble C++ (`ament_cmake`) joystick controller သည် PX4 X500 SITL အတွက်ဖြစ်သည်။ `joy_node` မှ `sensor_msgs/Joy` ကို လက်ခံပြီး uXRCE-DDS မှတစ်ဆင့် PX4 ROS 2 offboard message များကို ထုတ်လွှင့်သည်။

## $\color{red}{\text{ထိန်းချုပ်မှု ခလုတ်များ}}$

ပုံမှန် mapping သည် Linux Logitech F710 `joy_node` ပုံစံကို အသုံးပြုထားသည်။ သင့်စက်တွင် `ros2 topic echo /joy` ဖြင့် စစ်ဆေးပါ။

| Input | လုပ်ဆောင်ချက် |
|---|---|
| ဘယ် stick အပေါ်အောက် (axis 1) | အမြင့် setpoint ကို တိုး/လျှော့သည် |
| ဘယ် stick ဘယ်ညာ (axis 0) | Yaw လှည့်နှုန်း |
| ညာ stick အပေါ်အောက် (axis 4) | ရှေ့/နောက် အမြန်နှုန်း (`vx`) |
| ညာ stick ဘယ်ညာ (axis 3) | ညာ/ဘယ် အမြန်နှုန်း (`vy`) |
| D-pad အပေါ်/အောက် (axis 7) | Altitude ကို အနည်းငယ်တိုး/လျှော့သည် |
| X (button 2) | PX4 ကို Offboard mode သို့ပြောင်းပြီး arm လုပ်ရန် တောင်းဆိုသည် |
| B (button 1) | Disarm လုပ်ရန် တောင်းဆိုသည်။ ဆင်းပြီးမှသာ နှိပ်ပါ |
| Y (button 3) | `takeoff_altitude_m` အမြင့်သို့တက်သည်။ လိုအပ်ပါက Offboard/arm ကိုပါ တောင်းဆိုသည် |
| A (button 0) | PX4 land command ပို့သည် |

Setpoint များသည် PX4 local NED frame ကို အသုံးပြုသည်။ Controller သည် ပုံမှန် F710 stick ဦးတည်ချက်ကို သဘာဝကျသော တက်/ရှေ့/ညာ command များအဖြစ် ပြောင်းပေးသည်။ F710 mode နှင့် Linux driver mapping ကွဲပြားနိုင်သဖြင့် axis/button index နှင့် ပြောင်းပြန်ညွှန်ကြားမှုကို ROS parameter များဖြင့် သတ်မှတ်နိုင်သည်။

ဘယ် stick ၏ ဒေါင်လိုက် command ကို altitude setpoint တွင် ပေါင်းထည့်ပြီး ညာ stick က horizontal velocity ကို ထိန်းချုပ်သည်။ D-pad သည် ထို altitude setpoint ကို ချိန်ညှိသည်။ Stick ကိုလွှတ်လျှင် altitude ကို ထိန်းထားသည်။ `joy_timeout_sec` အတွင်း `/joy` message မရလျှင် joystick ရွေ့လျားမှု command များကို သုညထားပြီး altitude ကို ထိန်းသည်။

Joystick ဖြင့် မောင်းရန် PX4 ၏ **Offboard flight mode** ကို အသုံးပြုပါ။ Velocity နှင့် position သည် flight mode များမဟုတ်ဘဲ Offboard ထံပို့သည့် setpoint ပုံစံများဖြစ်သည်။ ယခု controller သည် ရှေ့/ဘေးရွေ့လျားမှုကို velocity setpoint ဖြင့်၊ အမြင့်ကို position setpoint ဖြင့် ထိန်းထားသော ပေါင်းစပ်ပုံစံကို သုံးထားသည်။ ထို့ကြောင့် joystick ကိုလွှတ်လျှင် အမြင့်မကျဘဲ ဆက်ထိန်းနိုင်သည်။

## $\color{red}{\text{လိုအပ်ချက်များ}}$

- ROS 2 Humble နှင့် `joy` package။
- ROS 2/uXRCE-DDS client နှင့် agent ချိတ်ဆက်ထားသော PX4 SITL။
- PX4 firmware version နှင့် ကိုက်ညီသည့် message definition များဖြင့် build လုပ်ထားသော `px4_msgs`။
- `/fmu/in/...` နှင့် `/fmu/out/vehicle_local_position` ကို ဖော်ပြပေးမည့် DDS topic bridge။

PX4 ROS 2 interface သည် uXRCE-DDS agent ကို အစားမထိုးပါ။ Controller ကို launch မလုပ်မီ သင့် PX4 version အတွက် သင့်လျော်သော command များဖြင့် agent နှင့် PX4 SITL ကို စတင်ပါ။

## $\color{red}{\text{Build နှင့် run}}$

ROS 2 workspace ၏ root directory မှ အောက်ပါ command များကို လုပ်ဆောင်ပါ။

```bash
sudo apt install ros-humble-joy
colcon build --packages-up-to ct_joy
source install/setup.bash
ros2 launch ct_joy ct_joy_controller.launch.py
```
```bash
ros2 launch ct_joy ct_joy_controller.launch.py px4_namespace:=px4_2
```

အခြား PX4 namespace သုံးရန် `px4_namespace` launch argument ကို သတ်မှတ်နိုင်သည်။ ဥပမာ `ros2 launch ct_joy ct_joy_controller.launch.py px4_namespace:=px4_2` ဖြစ်သည်။ Controller layout ကို `ros2 topic echo /joy` ဖြင့်စစ်ပြီး PX4 topic များကို `ros2 topic list | grep /fmu` ဖြင့် အတည်ပြုပါ။ X/B/Y/A ခလုတ်များသည် တစ်ကြိမ်နှိပ်တိုင်း တစ်ကြိမ်သာ အလုပ်လုပ်သည်။ X ကိုနှိပ်၍ arm လုပ်၊ Y ဖြင့် takeoff၊ A ဖြင့် land လုပ်ပြီး ဆင်းပြီးမှ B ဖြင့် disarm လုပ်ပါ။ ခလုတ်အညွှန်းများကွဲပါက အပေါ်ရှိ button index များကို `ros2 topic echo /joy` ဖြင့်စစ်ပါ။

## $\color{red}{\text{အသုံးဝင်သော parameter များ}}$

Launch file သည် ပုံမှန်အားဖြင့် `px4_1` namespace တွင် controller ကို လုပ်ဆောင်ပြီး `px4_namespace` launch argument ဖြင့် ပြောင်းနိုင်သည်။ PX4 topic များသည် `/<px4_namespace>/fmu/...` အောက်တွင် ရှိပြီး joystick input သည် မျှဝေထားသော `/joy` topic ကို ဆက်သုံးသည်။ Launch file တွင် အဓိက speed/altitude တန်ဖိုးများလည်း သတ်မှတ်ထားသည်။ ဥပမာ command line မှ node parameter များကို အောက်ပါအတိုင်း ပြောင်းနိုင်သည်။

```bash
ros2 run ct_joy ct_joy --ros-args \
  -p takeoff_altitude_m:=3.0 -p max_horizontal_speed_mps:=2.0
```

Parameter များတွင် `joy_topic`, `publish_rate_hz`, `joy_timeout_sec`, `takeoff_altitude_m`, `max_horizontal_speed_mps`, `max_vertical_speed_mps`, `max_yaw_rate_rps`, `altitude_trim_speed_mps`, `deadzone`, axis index များ၊ D-pad axis၊ `button_arm`, `button_disarm`, `button_takeoff`, `button_land` နှင့် `invert_*_axis` flag များ ပါဝင်သည်။

## $\color{red}{\text{PX4 မှတ်ချက်များ}}$

Node သည် `OffboardControlMode` နှင့် `TrajectorySetpoint` ကို 20 Hz ဖြင့် ဆက်တိုက်ထုတ်လွှင့်သည်။ ခလုတ်တောင်းဆိုမှုတစ်ခုသည် PX4 ၏ Offboard mode နှင့် arm command များ မပို့မီ setpoint ကို အနည်းဆုံး တစ်စက္ကန့်ကြာ ထုတ်လွှင့်ထားရန် စောင့်သည်။ Offboard mode အသုံးပြုရာတွင် ROS 2/DDS ချိတ်ဆက်မှု ကျန်းမာစွာရှိနေရမည်။ Hardware တွင် မသုံးမီ PX4 failsafe parameter များကို သတ်မှတ်ပြီး SITL တွင် အပြုအမူကို စစ်ဆေးပါ။
