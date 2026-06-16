# scout.inc
# 第二个机器人项目 (Task 2) —— Stage 机器人模型定义
#
# 机器人:AgileX Scout Mini,全向 (Omnidirectional) 运动学
# 这个文件定义【可复用的模型】,被 scout.world 引用。
#
# 关键点:
#   - drive "omni"  —— 全向运动学(作业明确要求,不是 diff!)
#   - 激光传感器参数对应从 RSLidar pointcloud 转换出的 laserscan
#   - localization "odom" + odom_error 模拟里程计噪声8

# -------------------- 激光雷达传感器定义 --------------------
# 这里把 RSLidar(转成 2D laserscan 后)建模成一个 ranger 传感器。
# fov 360°,样本数和量程要和真实雷达接近。
define rslidar ranger
(
  sensor
  (
    # range [最小距离 最大距离],单位米。RSLidar 量程通常较大
    range [ 0.05 30.0 ]
    # 视场角:360 度全向扫描
    fov 360.0
    # 采样点数:激光每圈的射线数。可按你 laserscan 的实际值调整
    samples 720
  )
  # 传感器外观(仅显示用)
  color "black"
  size [ 0.06 0.06 0.05 ]
)

# -------------------- Scout Mini 机器人本体定义 --------------------
define scout_mini position
(
  # 初始位姿(会被 .world 里的实例覆盖)
  pose [ 0.0 0.0 0.0 0.0 ]

  # 里程计来源:用仿真内置的 odom,并加噪声模拟真实里程计漂移
  localization "odom"
  # odom_error [x误差 y误差 z误差 theta误差],模拟里程计噪声
  # 真实 /odom 是 noisy 的,这里也加一点让仿真更真实
  odom_error [ 0.03 0.03 0.0 0.05 ]

  # 机器人尺寸 [长 宽 高],单位米 —— 接近真实 Scout Mini
  size [ 0.612 0.580 0.245 ]
  # 旋转中心(几何中心)
  origin [ 0.0 0.0 0.0 0.0 ]

  # 显示一个"鼻子"指示朝向,方便观察
  gui_nose 1

  # *** 关键:全向运动学 ***
  # 课程示例是 "diff",但 Scout Mini 要求 "omni"
  drive "omni"

  color "blue"

  # 把激光雷达装到机器人上,位置在中心略偏上
  rslidar( pose [ 0.0 0.0 0.15 0.0 ] )
)
