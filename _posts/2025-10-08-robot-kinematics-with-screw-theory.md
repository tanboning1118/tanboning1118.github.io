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
    * **建立坐标系**: 首先，确定两个关键的坐标系。
        * **世界坐标系 `{s}`**: 一个固定不变的惯性系，通常建立在机器人的基座上。
        * **工具坐标系 `{b}`**: 一个附着在机器人末端执行器上的坐标系，随机器人运动而运动。
    * **定义零位姿**: 为机器人选择一个简单、方便计算的**“零位姿” (Zero Pose)**。这是一个指定的参考构型，在此姿态下，所有关节变量 $\theta_i$ 均定义为 $0$。通常选择所有连杆完全伸直或处于某个基础角度的姿态，因为这能极大地简化后续的几何计算。

2.  **计算初始位姿 `M` (Calculate the Initial Pose `M`)**
    * 将机器人固定在您所定义的**零位姿**下。
    * 通过几何观察，直接计算出此刻工具坐标系 `{b}` 相对于世界坐标系 `{s}` 的齐次变换矩阵 $M$。
    * $M$ 是一个固定不变的矩阵，代表了运动链的初始偏移量，即 $M = T_{sb}(0)$。

3.  **计算所有关节的螺旋轴 `Sᵢ` (Calculate All Joint Screw Axes `Sᵢ`)**
    * 这是建模过程中最核心的一步。**关键原则：在整个此步骤中，机器人必须始终保持在零位姿，并且所有计算出的向量 (`ω`, `q`, `v`) 都必须是在世界坐标系 `{s}` 下表示的。**
    * 对于从基座到末端的每一个关节 $i=1, \dots, n$，按以下流程操作：

        a. **确定旋转轴方向 `ωᵢ`**:
            * 观察关节 `i` 在零位姿下的运动轴线。
            * 在世界坐标系 `{s}` 中，将该轴线的方向表示为一个单位向量 `ωᵢ`。（对于移动关节，`ωᵢ = 0`）。

        b. **确定轴上一点 `qᵢ`**:
            * 在关节 `i` 的运动轴线上，任选一个方便计算的点。
            * 在世界坐标系 `{s}` 中，确定该点的坐标向量 `qᵢ`。

        c. **计算线性分量 `vᵢ`**:
            * 根据关节类型，使用已知的 `ωᵢ` 和 `qᵢ` 来计算线性分量 `vᵢ`。
                * **旋转关节**: $v_i = -\omega_i \times q_i$
                * **移动关节**: $v_i$ 是移动方向的单位向量 (此情况下无需 `qᵢ` 来计算 `vᵢ`)。
                * **螺旋关节**: $v_i = -\omega_i \times q_i + \frac{h}{2\pi}\omega_i$

        d. **组合得到螺旋轴 `Sᵢ`**:
            * 将 `ωᵢ` 和 `vᵢ` 组合成最终的六维螺旋轴向量 $S_i = (\omega_i, v_i)^T$。

4.  **写出完整的PoE公式 (Assemble the Complete PoE Formula)**
    * 将上一步计算出的初始位姿 $M$ 和所有螺旋轴 $S_1, \dots, S_n$ 代入指数积公式中。
    * 至此，您便得到了该机器人的完整正运动学模型：
        $$T_{sb}(\theta) = e^{[\mathcal{S}_1]\theta_1} e^{[\mathcal{S}_2]\theta_2} \cdots e^{[\mathcal{S}_n]\theta_n} M$$
    * 这个公式现在是一个只依赖于关节变量 $\theta = (\theta_1, \dots, \theta_n)$ 的函数，可以计算出机器人在任何构型下的末端位姿。
