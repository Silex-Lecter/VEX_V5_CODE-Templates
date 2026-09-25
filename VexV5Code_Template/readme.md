# 底盘控制流程
目标位姿 / 目标速度
        ↓
位姿控制器 / 速度控制器
        ↓
期望底盘速度 vx, vy, omega
        ↓
底盘逆运动学
        ↓
每个轮子的目标速度
        ↓
电机 PID

# 定位系统计算流程
1. 编码器读数转换成本体x和y位移、不同的定位体系算法不同
- lemlib圆弧近似算法
$$
\Delta \mathbf{p}_{body}
=
2\sin\left(\frac{\Delta\theta}{2}\right)
\left(
\frac{\mathbf{s}}{\Delta\theta}
+
\mathbf{r}
\right)
$$

2. 本体位移、旋转到世界坐标系、累加（使用中点积分）
$$
\Delta \mathbf{p}_{world}
=
R\left(\theta+\frac{\Delta\theta}{2}\right)
\Delta \mathbf{p}_{body}
$$

# 坐标约定
参考ROS REP-103标准
X 轴‌：‌向前‌（Forward）
‌Y 轴‌：‌向左‌（Left）
‌Z 轴‌：‌向上‌（Up）