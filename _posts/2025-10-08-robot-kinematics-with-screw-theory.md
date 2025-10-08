---
title: "掌握现代机器人学：基于旋量理论的正运动学"
date: 2025-10-08 10:00:00 +0800
category: 机器人学
tags: [旋量理论, 机器人运动学, 正运动学, 指数积, PoE, 教程]
author: Bathelor
toc: true
math: true
---

使用旋量（Screw Theory）来表述含螺旋副（Screw Joints）的机器人末端位姿，是非常优雅且强大的方法。这种方法的核心就是**指数积（Product of Exponentials, PoE）**公式。

与传统的D-H参数法相比，旋量法有几个显著优点：
* **统一性**：无论是旋转关节、移动关节还是螺旋关节，都可以用统一的“旋量”概念来描述。
* **直观性**：不需要为了满足D-H建模规则而建立复杂的中间坐标系，所有关节的运动轴都可以在一个固定的世界坐标系下描述。
* **通用性**：非常适合描述串联、并联以及更复杂的机器人构型。

下面将分解如何使用旋量来表述。

### 1. 核心概念：旋量 (Screw)

一个旋量（在机器人运动学中也叫**螺旋轴 Screw Axis**）是一个六维向量 $S$，它完整地定义了一个刚体的瞬时运动。

一个在世界坐标系 `{s}` 下定义的螺旋轴 $S$ 写成：

$$S = \begin{pmatrix} \omega \\ v \end{pmatrix} \in \mathbb{R}^6$$

其中：
* $\omega \in \mathbb{R}^3$ 是一个三维向量，表示**旋转轴的方向**。对于纯移动关节，$\omega = 0$。对于旋转或螺旋关节，我们通常将其定义为单位向量。
* $v \in \mathbb{R}^3$ 是一个三维向量，表示**线性运动**的部分。

#### 如何确定不同关节的螺旋轴 S？

我们在机器人的**零位姿（Zero Configuration）**下，确定每个关节的螺旋轴：

* **旋转关节 (Revolute Joint)**:
    * $\omega$: 关节旋转轴的方向向量（单位向量）。
    * $q$: 轴线上任意一点，则 $v = -\omega \times q$。

* **移动关节 (Prismatic Joint)**:
    * $\omega = 0$。
    * $v$: 关节移动方向的单位向量。

* **螺旋关节 (Screw Joint)**:
    * $\omega$: 螺旋轴的方向向量（单位向量）。
    * $h$ (螺距 pitch): 每旋转 $2\pi$ 弧度沿轴线移动的距离。
    * $q$: 轴线上任意一点，则 $v = -\omega \times q + \frac{h}{2\pi}\omega$。

### 2. 核心工具：指数映射 (Exponential Map)

指数映射将螺旋轴 $S$ 和关节变量 $\theta$ 转换为一个 4x4 的齐次变换矩阵 $T$。对于任意螺旋轴 $S=(\omega, v)$，其对应的变换矩阵为：

$$T(\theta) = e^{[\mathcal{S}]\theta} = \begin{pmatrix} R(\theta) & p(\theta) \\ 0 & 1 \end{pmatrix}$$

其中，旋转矩阵 $R(\theta)$ 和平移向量 $p(\theta)$ 为：

* **旋转部分 (Rodrigues' 公式):**
    $$R(\theta) = e^{[\omega]\theta} = I + \sin(\theta)[\omega] + (1 - \cos(\theta))[\omega]^2$$

* **平移部分:**
    $$p(\theta) = \left( I\theta + (1-\cos(\theta))[\omega] + (\theta - \sin(\theta))[\omega]^2 \right) v$$

* $[\omega]$ 是 $\omega$ 向量的3x3反对称矩阵:
    $$[\omega] = \begin{pmatrix} 0 & -\omega_z & \omega_y \\ \omega_z & 0 & -\omega_x \\ -\omega_y & \omega_x & 0 \end{pmatrix}$$

### 3. 核心公式：指数积 (Product of Exponentials, PoE)

对于一个 n 自由度的串联机器人，其末端位姿 $T_{sb}$ 可以表示为：

$$T_{sb}(\theta) = e^{[\mathcal{S}_1]\theta_1} e^{[\mathcal{S}_2]\theta_2} \cdots e^{[\mathcal{S}_n]\theta_n} M$$

* $\theta_i$: 第 $i$ 个关节的变量（角度或距离）。
* $S_i$: 第 $i$ 个关节的螺旋轴（在零位姿下，于世界坐标系 `{s}` 中定义）。
* $e^{[\mathcal{S}_i]\theta_i}$: 第 $i$ 个关节运动产生的4x4齐次变换。
* $M$: 机器人在零位姿时，末端坐标系 `{b}` 相对于世界坐标系 `{s}` 的**初始位姿**，即 $M=T_{sb}(0)$。

### 4. 建模实战流程 (Practical Modeling Workflow)

本节将理论转化为一个清晰、可重复的流程。遵循以下步骤，您可以为任何串联机器人建立基于旋量法的正运动学模型。

1.  **建立坐标系与定义零位姿 (Establish Frames & Define Zero Pose)**
    * **建立坐标系**: 确定世界坐标系 `{s}` (通常在基座) 和工具坐标系 `{b}` (在末端)。
    * **定义零位姿**: 选择一个所有关节变量 $\theta_i$ 均为 $0$ 的参考构型。

2.  **计算初始位姿 `M` (Calculate the Initial Pose `M`)**
    * 在**零位姿**下，通过几何观察计算出 $M = T_{sb}(0)$。

3.  **计算所有关节的螺旋轴 `Sᵢ` (Calculate All Joint Screw Axes `Sᵢ`)**
    * **关键原则**：始终在**零位姿**下，于**世界坐标系 `{s}`** 中进行计算。
    * 对每个关节 $i=1 \dots n$，依次确定其轴向 `ωᵢ`、轴上一点 `qᵢ`，并最终算出 `vᵢ`，组合得到 $S_i$。

4.  **写出完整的PoE公式 (Assemble the Complete PoE Formula)**
    * 将计算出的 $M$ 和所有 $S_i$ 代入指数积公式，得到完整的运动学模型 $T_{sb}(\theta)$。

### 5. 实战演练：机器人运动学分析

下面，我们使用上述方法解决一个具体的例题。

* **题目图纸**:
    ![机器人运动学例题图纸](/assets/images/posts/problem.png)
* **符号约定**：所有螺旋轴均在**空间坐标系 {s}** 中表示；PoE 采用 $T_{sb}(\theta)=e^{[\mathcal{S}_1]\theta_1}e^{[\mathcal{S}_2]\theta_2}e^{[\mathcal{S}_3]\theta_3}M$。

#### 步骤 0: 已知条件与关节变量

* **几何参数**: $L_1=10, L_2=5, L_3=5, L_4=3$。
* **关节1**: 螺旋关节, 螺距 $h=2$。
* **目标**: 求 $t=4$s 时的末端位姿 $T_{sb}(4)$。
* **关节变量**:
    * $\theta_1(4) = (\pi/4) \cdot 4 = \pi$ rad
    * $\theta_2(4) = (\pi/8) \cdot 4 = \pi/2$ rad
    * $\theta_3(4) = (-\pi/4) \cdot 4 = -\pi$ rad

#### 步骤 1: 初始位姿 `M`

根据该机器人的几何设定，其在零位姿下的末端位姿矩阵 `M` 为：
$$M = \begin{pmatrix} 0 & -1 & 0 & 0 \\ 1 & 0 & 0 & 8 \\ 0 & 0 & 1 & 5 \\ 0 & 0 & 0 & 1 \end{pmatrix}$$

#### 步骤 2: 关节螺旋轴 `Sᵢ`

* **关节 1 (螺旋关节)**:
    * **求 `ω₁`**: 关节1绕世界系 `z` 轴旋转，因此 $\omega_1 = (0, 0, 1)^T$。
    * **求 `q₁`**: 旋转轴经过世界系原点，因此取轴上一点 $q_1 = (0, 0, 0)^T$。
    * **求 `v₁`**: 使用螺旋关节公式 $v = -\omega \times q + \frac{h}{2\pi}\omega$。
        $$v_1 = -(0,0,1) \times (0,0,0) + \frac{2}{2\pi}(0,0,1) = (0, 0, 1/\pi)^T$$
    * **组合 `S₁`**:
        $$S_1 = (\omega_1, v_1)^T = ((0,0,1), (0,0,1/\pi))^T$$

* **关节 2 (旋转关节)**:
    * **求 `ω₂`**: 关节2绕平行于世界系 `y` 轴的方向旋转，因此 $\omega_2 = (0, 1, 0)^T$。
    * **求 `q₂`**: 在零位姿下，该关节轴线经过点 $q_2 = (0, L_2, L_1) = (0, 5, 10)^T$。
    * **求 `v₂`**: 使用旋转关节公式 $v = -\omega \times q$。
        $$v_2 = -(0,1,0) \times (0,5,10) = - (10, 0, 0) = (-10, 0, 0)^T$$
    * **组合 `S₂`**:
        $$S_2 = (\omega_2, v_2)^T = ((0,1,0), (-10,0,0))^T$$

* **关节 3 (旋转关节)**:
    * **求 `ω₃`**: 关节3绕平行于世界系 `x` 轴的方向旋转，因此 $\omega_3 = (1, 0, 0)^T$。
    * **求 `q₃`**: 在零位姿下，该关节轴线经过点 $q_3 = (0, L_2, L_1-L_3) = (0, 5, 5)^T$。
    * **求 `v₃`**: 使用旋转关节公式 $v = -\omega \times q$。
        $$v_3 = -(1,0,0) \times (0,5,5) = - (0, -5, 5) = (0, 5, -5)^T$$
    * **组合 `S₃`**:
        $$S_3 = (\omega_3, v_3)^T = ((1,0,0), (0,5,-5))^T$$

#### 步骤 3: 指数映射 `e^[Sᵢ]θᵢ`

* **关节 1 ($\theta_1=\pi$)**:
    对于纯螺旋运动（$v$ 平行于 $\omega$），平移公式简化为 $p(\theta)=v\theta$。
    $R_1(\pi) = \text{Rot}(z, \pi)$, $p_1(\pi) = v_1 \pi = (0,0,1/\pi) \cdot \pi = (0,0,1)^T$。
    $$T_1 = e^{[\mathcal{S}_1]\pi} = \begin{pmatrix} -1 & 0 & 0 & 0 \\ 0 & -1 & 0 & 0 \\ 0 & 0 & 1 & 1 \\ 0 & 0 & 0 & 1 \end{pmatrix}$$

* **关节 2 ($\theta_2=\pi/2$)**:
    $R_2(\pi/2) = \text{Rot}(y, \pi/2)$。使用平移公式 $p(\theta) = (I\theta + \dots)v$ 计算得 $p_2(\pi/2) = (-10, 0, 10)^T$。
    $$T_2 = e^{[\mathcal{S}_2]\pi/2} = \begin{pmatrix} 0 & 0 & 1 & -10 \\ 0 & 1 & 0 & 0 \\ -1 & 0 & 0 & 10 \\ 0 & 0 & 0 & 1 \end{pmatrix}$$

* **关节 3 ($\theta_3=-\pi$)**:
    $R_3(-\pi) = \text{Rot}(x, -\pi)$。使用平移公式计算得 $p_3(-\pi) = (0, 10, 10)^T$。
    $$T_3 = e^{[\mathcal{S}_3](-\pi)} = \begin{pmatrix} 1 & 0 & 0 & 0 \\ 0 & -1 & 0 & 10 \\ 0 & 0 & -1 & 10 \\ 0 & 0 & 0 & 1 \end{pmatrix}$$

#### 步骤 4: 末端位姿 `T_sb(4)`

将所有矩阵按公式 $T_{sb}(4) = T_1 \cdot T_2 \cdot T_3 \cdot M$ 连乘。

* **第一步: $T_{12} = T_1 \cdot T_2$**
    $$T_{12} = \begin{pmatrix} 0 & 0 & -1 & 10 \\ 0 & -1 & 0 & 0 \\ -1 & 0 & 0 & 11 \\ 0 & 0 & 0 & 1 \end{pmatrix}$$

* **第二步: $T_{123} = T_{12} \cdot T_3$**
    $$T_{123} = \begin{pmatrix} 0 & 0 & 1 & 0 \\ 0 & 1 & 0 & -10 \\ -1 & 0 & 0 & 11 \\ 0 & 0 & 0 & 1 \end{pmatrix}$$

* **第三步: $T_{sb}(4) = T_{123} \cdot M$**
    $$T_{sb}(4) = \begin{pmatrix} 0 & 0 & 1 & 0 \\ 0 & 1 & 0 & -10 \\ -1 & 0 & 0 & 11 \\ 0 & 0 & 0 & 1 \end{pmatrix} \begin{pmatrix} 0 & -1 & 0 & 0 \\ 1 & 0 & 0 & 8 \\ 0 & 0 & 1 & 5 \\ 0 & 0 & 0 & 1 \end{pmatrix}$$

连乘得到最终结果：
$$
T_{sb}(4) = \begin{pmatrix}
0 & 0 & 1 & 5 \\
1 & 0 & 0 & -2 \\
0 & 1 & 0 & 11 \\
0 & 0 & 0 & 1
\end{pmatrix}
$$
