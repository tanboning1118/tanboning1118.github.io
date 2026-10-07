/* Edit this file to update both the homepage and the printable CV. */
window.PROFILE = {
  email: 'tanponing@gmail.com', updated: '2026-10-07',
  media: {
    hero: {file:'dexhand-grasp.png',en:'Dexhand grasping a drone model · Competition prototype',zh:'灵巧手抓取无人机模型 · 竞赛项目实物'},
    exo: [
      {file:'exoskeleton-test.png',en:'Knee flexion experiment with the soft exoskeleton prototype',zh:'柔性外骨骼原型机膝关节屈伸实验'},
      {video:'exo-wear.mp4',en:'Walking test in the full exoskeleton (land trial)',zh:'外骨骼整机穿戴行走测试（陆上）'},
      {video:'exo-treadmill.mp4',en:'Treadmill walking trial with the backpack control unit',zh:'跑步机步行试验（背负电控舱）'},
      {file:'exoskeleton-winch.png',en:'Bidirectional winch design',zh:'正反转双控绞盘设计'},
      {file:'exoskeleton-cuff.png',en:'Wearable cuff and compliant mechanism design',zh:'腿部穿戴与柔性机构设计'},
      {file:'exoskeleton-pcb.png',en:'Power distribution and emergency-stop PCB design',zh:'分线板与急停开关板 PCB 设计'}
    ],
    dex: [{file:'dexhand-grasp.png',en:'Dexterous hand grasping a drone model',zh:'灵巧手抓取无人机模型'}],
    arm: [
      {file:'robot-arm-cad.png',en:'IRB2600 robot arm · SolidWorks model',zh:'IRB2600 机械臂 · SolidWorks 建模'},
      {video:'arm-simulink.mp4',en:'Simscape simulation: the arm tracking its target trajectory',zh:'Simscape 仿真：机械臂跟踪目标轨迹'},
      {file:'arm-tracking.png',en:'Cartesian trajectory tracking and error on x/y/z',zh:'笛卡尔空间 x/y/z 轨迹跟踪与误差'},
      {file:'arm-edge.png',en:'Desired trajectory extracted by Canny edge detection',zh:'Canny 边缘检测提取的期望轨迹'},
      {file:'arm-path.png',en:'Trajectory points ordered by the greedy pass',zh:'贪心算法排序后的轨迹点访问顺序'},
      {file:'arm-workspace.png',en:'Workspace envelope from Monte Carlo sampling',zh:'蒙特卡洛法求出的工作空间包络面'},
      {file:'trajectory-result.png',en:'Trajectory extraction and fitting result',zh:'轨迹提取与拟合结果'}
    ],
    chip: [{file:'fpga-prototype.png',en:'FPGA combination lock · Hardware demonstration',zh:'FPGA 数字密码锁 · 实物演示'}]
  },
  skills: ['Python / PyTorch', 'MATLAB / Simulink', 'ROS', 'OpenCV', 'CAD / 3D printing', 'PCB / Embedded'],
  en: {
    navResearch:'Research',navProjects:'Projects',navAbout:'Experience',navContact:'Contact',navCV:'View CV ↗',role:"MASTER'S STUDENT · TONGJI UNIVERSITY",
    statement:'Making robots\nactually do things.',
    intro:"I'm Boning Tan, a first-year master's student in Mechanical Engineering at Tongji University. These days I work on embodied AI — vision-language-action (VLA) models and robot manipulation. Before that I spent four years building hardware: an underwater exoskeleton, robot arms, embedded boards. That background makes me care about what works on a real robot, not just in a demo.",
    explore:'See my work ↗',viewCV:'Curriculum vitae ↗',stripLabel:'HANDS-ON',stripText:'From turning wrenches to training models — I like owning the whole stack.',
    researchHeading:'Research & publications',researchAside:'EMBODIED AI, ON REAL HARDWARE',researchIntro:'My focus is embodied AI: how a robot can understand a scene, follow an instruction, and get a task done — concretely, VLA models and manipulation. Coming from wearable sensing and hardware, I judge every idea by the same question: does it hold up on a physical robot?',
    interests:['Vision-language-action models','Robot manipulation','Imitation learning','Wearable sensing'],thirdAuthor:'THIRD AUTHOR',paperContribution:'My part: CNN-based classification of hydrogel sensor signals to recognize four swimming postures.',readPaper:'Read publication ↗',projectsHeading:'Selected projects',projectsAside:'THINGS I BUILT',
    experienceHeading:'Experience & education',experienceLabel:'EXPERIENCE',educationLabel:'EDUCATION',toolkit:'TOOLS I USE',awardsLabel:'HONORS',contactEyebrow:'GET IN TOUCH',contactHeading:'Say hi.',contactText:'Research collaboration, internship opportunities, or just talking robots and embodied AI — my inbox is open.',footer:'Shanghai · Tongji University',backTop:'Back to top ↑',details:'Project details',print:'Print / Save as PDF',home:'← Homepage',cvTitle:'Curriculum vitae',publication:'PUBLICATION',videoLabel:'TEST FOOTAGE',videoLabelHint:'muted · loop',
    projects:[
      {id:'01',type:'WEARABLE ROBOTICS',title:'Underwater soft knee exoskeleton',date:'2025.12 — 2026.05',summary:'My undergraduate thesis: a soft exoskeleton that assists a diver\u2019s kick — mechanical design, electronics, and learning-based prediction, all validated in the pool.',tags:['Mechanical design','IMU / CAN bus','CNN-LSTM'],details:['Single-motor bidirectional winch driving Bowden cables for flexion–extension assistance; dual-material 3D-printed leg interface with flexure hinges that follows the knee\u2019s moving rotation center.','Two custom four-layer PCBs (power distribution + emergency stop) sealed in a waterproof control enclosure — passed a 10-hour immersion test at 2.5 m depth.','An SO(3) + PCA pipeline turns four IMUs into knee angles, and a CNN-LSTM predicts them 60–200 ms ahead. Across six pool trials: RMSE 5–6° at the 100 ms horizon, mean response delay 38 ms, assistance efficiency ≥ 98%.'],art:'exo'},
      {id:'02',type:'COMPETITION · NATIONAL FIRST PRIZE',title:'Smart manufacturing competition',date:'2024 — 2025.08',summary:'National First Prize (undergraduate group) at the 2025 Chinese College Students Mechanical Engineering Innovation and Creativity Competition — 8th \u201CXipu Intelligent Cup\u201D, smart equipment & production-line track.',tags:['Team of three','Smart equipment','National First Prize'],details:['Team project with Yafei Zhang and Weijun Wang, advised by Tongji faculty.','The competition prototype included a dexterous-hand grasping setup — the hero photo at the top of this page shows the hand holding a drone model.'],art:'dex'},
      {id:'03',type:'MODELING & CONTROL',title:'Six-DOF robot arm: kinematics, vision & control',date:'2024.09 — 2025.01',summary:'A course project done to a research standard: full kinematic and dynamic modeling of an IRB2600-class arm, a vision pipeline that reads a path from a photo, and a controller that tracks it.',tags:['MATLAB / Simulink','OpenCV','Trajectory planning'],details:['DH model with analytic inverse kinematics (Pieper criterion), geometric Jacobian, and Lagrangian dynamics.','Compared cubic, quintic, LSPB, Bézier, and B-spline interpolation under velocity and acceleration limits.','OpenCV pipeline — Canny edges, Hough lines, greedy point ordering, Zhang camera calibration — turning a drawn curve into Cartesian waypoints.','Feedforward dynamics compensation plus PD tracking in Simulink; Cartesian MAE about 15–17 mm on x/y.'],art:'arm'},
      {id:'04',type:'EMBEDDED SENSING',title:'Radar-based human presence sensing',date:'2024.11 — 2024.12',summary:'A small embedded pipeline from presence detection to cloud reporting.',tags:['ESP32','mmWave radar','MQTT'],details:['Integrated an LD2410 millimeter-wave radar to detect presence, distance, and moving versus stationary targets.','Uploaded structured data to OneNet through MQTT, with periodic reporting and automatic reconnection.'],art:'radar'},
      {id:'05',type:'DIGITAL SYSTEMS',title:'FPGA digital combination lock',date:'2024.06 — 2024.07',summary:'A complete interactive digital system, built as a finite-state machine.',tags:['Verilog','Finite-state machine','FPGA'],details:['Designed a six-state controller for password setup, verification, alarms, and timed lockout.','Integrated keypad and buzzer feedback, with a 13-bit LFSR for pseudorandom sequences.'],art:'chip'}
    ],
    experience:[{date:'2026.06 — 2026.08',title:'Tencent Robotics X',subtitle:'Data & Evaluation Team Intern',text:'Kinematic singularity analysis for teleoperation measurement arms; ROS-based sensor data acquisition, synchronization, and recording.'}],
    education:[{date:'2026.09 — Present',title:'Tongji University',subtitle:"Master's student · Mechanical Engineering",text:'School of Mechanical Engineering and Robotics.'},{date:'2022.09 — 2026.06',title:'Tongji University',subtitle:'B.Eng. · Intelligent Manufacturing Engineering',text:'GPA: 4.70 / 5.00 · CET-6: 543 · Thesis: underwater soft knee exoskeleton'}],
    awards:['National First Prize · Chinese College Students Mechanical Engineering Innovation and Creativity Competition (\u201CXipu Intelligent Cup\u201D), 2025','Tongji University Undergraduate Outstanding Student Scholarship · 2022–2023, 2023–2024, 2024–2025','Second Prize, Shanghai Selection · Mechanical Engineering Innovation and Creativity Competition, 2024']
  },
  zh: {
    navResearch:'研究',navProjects:'项目',navAbout:'经历',navContact:'联系',navCV:'查看简历 ↗',role:'同济大学 · 机械工程硕士在读',statement:'让机器人\n把事做成。',intro:'我是谭泊宁，同济大学机械工程专业研一在读。现在主要做具身智能，研究 VLA（视觉-语言-动作）模型和机器人操作。本科四年我一直在跟硬件打交道：水下外骨骼、机械臂、嵌入式板子，从设计、加工到下水实测都完整做过。这段经历让我判断一个想法时总会多问一句：放到真机上，它还成立吗？',explore:'看看我做的东西 ↗',viewCV:'个人简历 ↗',stripLabel:'动手派',stripText:'从拧螺丝到训模型，整条链路我都喜欢自己做。',researchHeading:'研究与论文',researchAside:'具身智能，落在真机上',researchIntro:'我现在的方向是具身智能：让机器人看懂场景、听懂指令、把任务做完——具体说，是 VLA 模型与机器人操作。因为做过可穿戴传感和整套硬件，我习惯用同一个标准衡量想法：在真实机器人上跑不跑得通。',interests:['视觉-语言-动作模型','机器人操作','模仿学习','可穿戴传感'],thirdAuthor:'第三作者',paperContribution:'我的部分：用 CNN 对水凝胶传感器信号分类，识别四种泳姿。',readPaper:'阅读论文 ↗',projectsHeading:'做过的项目',projectsAside:'从图纸到实物',experienceHeading:'经历与教育',experienceLabel:'实习经历',educationLabel:'教育背景',toolkit:'常用工具',awardsLabel:'荣誉',contactEyebrow:'保持联系',contactHeading:'欢迎来聊。',contactText:'科研合作、实习机会，或者单纯聊聊机器人和具身智能，都欢迎写邮件给我。',footer:'上海 · 同济大学',backTop:'回到顶部 ↑',details:'项目细节',print:'打印 / 另存为 PDF',home:'← 返回主页',cvTitle:'个人简历',publication:'学术论文',videoLabel:'实测视频',videoLabelHint:'静音 · 循环播放',
    projects:[
      {id:'01',type:'可穿戴机器人',title:'水下柔性膝关节外骨骼',date:'2025.12 — 2026.05',summary:'我的本科毕业设计：一套给潜水员踢水助力的柔性外骨骼，机械、电控、学习预测全都自己做，最后在泳池里验证。',tags:['机构设计','IMU / CAN 总线','CNN-LSTM'],details:['单电机双向绞盘驱动鲍登线，实现屈伸方向互补牵拉；腿部接口用柔性铰链加微型导轨双料打印，能跟着膝关节瞬时旋转中心的漂移走。','自己画的两块四层 PCB（分线板 + 急停保护），密封进防水电控舱，2.5 米水深泡了 10 小时没进水。','SO(3) + PCA 流水线把 4 个 IMU 解算成膝关节角度，CNN-LSTM 提前 60–200 ms 做预测。6 次泳池试验下来：100 ms 提前量下 RMSE 5–6°，平均响应延迟 38 ms，助力效率不低于 98%。'],art:'exo'},
      {id:'02',type:'学科竞赛 · 全国一等奖',title:'智能制造竞赛',date:'2024 — 2025.08',summary:'2025 年中国大学生机械工程创新创意大赛（第八届"犀浦智能杯"智能制造赛）本科生组全国一等奖，方向是智能装备与产线开发。',tags:['三人团队','智能装备','全国一等奖'],details:['与张亚飞、王伟俊组队完成，同济大学教师指导。','竞赛原型里有一套灵巧手抓取装置——本页顶部的大图，就是它在抓一个无人机模型。'],art:'dex'},
      {id:'03',type:'建模与控制',title:'六自由度机械臂：建模、视觉与控制',date:'2024.09 — 2025.01',summary:'一门课设，但按做项目的标准做的：给 IRB2600 型机械臂建完整的运动学、动力学模型，再用视觉把照片里的轨迹读出来，让控制器跟上。',tags:['MATLAB / Simulink','OpenCV','轨迹规划'],details:['DH 法建模，按 Pieper 准则推逆运动学解析解，几何雅可比 + 拉格朗日动力学。','在速度和加速度约束下对比三次、五次、LSPB、贝塞尔、B 样条五种插值。','OpenCV 视觉链路：Canny 边缘、霍夫直线、贪心排序控制点、张正友标定，把画出来的曲线变成笛卡尔路径点。','Simulink 里做前馈动力学补偿加 PD 跟踪，笛卡尔空间 x/y 轴 MAE 约 15–17 mm。'],art:'arm'},
      {id:'04',type:'嵌入式感知',title:'毫米波雷达人体感知',date:'2024.11 — 2024.12',summary:'从人体存在检测到云端上报的一套嵌入式小系统。',tags:['ESP32','毫米波雷达','MQTT'],details:['集成 LD2410 毫米波雷达，识别人体存在、距离和动静状态。','通过 MQTT 向 OneNet 定时上传结构化数据，支持自动重连。'],art:'radar'},
      {id:'05',type:'数字系统',title:'FPGA 数字密码锁',date:'2024.06 — 2024.07',summary:'用有限状态机做的一套完整可交互数字系统。',tags:['Verilog','有限状态机','FPGA'],details:['设计六状态控制器，覆盖密码设置、验证、错误报警和锁死倒计时。','集成按键与蜂鸣器反馈，用 13 位 LFSR 生成伪随机序列。'],art:'chip'}
    ],
    experience:[{date:'2026.06 — 2026.08',title:'腾讯 Robotics X',subtitle:'数据与评测团队实习生',text:'遥操作测量臂的运动学奇异性分析；基于 ROS 的传感器数据采集、同步与记录。'}],
    education:[{date:'2026.09 — 至今',title:'同济大学',subtitle:'机械工程 · 硕士研究生',text:'机械工程与机器人学院。'},{date:'2022.09 — 2026.06',title:'同济大学',subtitle:'智能制造工程 · 本科',text:'GPA：4.70 / 5.00 · 英语六级 543 · 毕设：水下柔性膝关节外骨骼'}],
    awards:['2025 年中国大学生机械工程创新创意大赛（"犀浦智能杯"智能制造赛）· 全国一等奖','同济大学优秀学生奖学金 · 2022–2023、2023–2024、2024–2025 学年','2024 年中国大学生机械工程创新创意大赛 · 上海赛区二等奖']
  }
};
