### 波浪滤波与海况自适应相关专利（Sperry / Honeywell / ABB-Fossen / Siemens / Kongsberg / Furuno / 三菱重工）

本节覆盖"波浪滤波（wave filtering）"与"海况自适应（sea-state / weather adaptation）"两条技术线上的已公开专利：含 DP 动力定位波浪滤波谱系（Honeywell → Siemens → Kongsberg）、Sperry 系自适应气象调节自动舵谱系、Fossen/NTNU 的气象最优定位（WOPC）专利，以及商用自动舵的恶劣天气操舵逻辑。核实方式：检索期间 Google Patents 对本机 IP 持续限流（HTTP 503 反滥用页），US/JP 条目改为逐页打开 FreePatentsOnline 著录页与全文页核实标题、受让人、申请/公开日期与说明书正文；EP 条目直接打开 EPO publication-server 官方公布文本 PDF 核实；NO 条目打开 patentimages.storage.googleapis.com 上挪威专利局（Patentstyret）官方扫描扉页核实。无法打开页面核实的候选条目（如 Tokimec JPH0443840B2、Sperry US2796576A）已按要求丢弃，未列入。

#### 专利清单

| 公开号/授权号 | 名称 | 申请人 | 申请/公开年份 | 对应商用产品 | 对应算法/主题 |
|---|---|---|---|---|---|
| US3604907A | Steering command computer for navigable craft | Sperry Rand Corp. | 1970 申请 / 1971 授权 | Sperry 早期自动舵的 Weather Adjustment（气象调节）功能（具体型号不可考） | 海况自适应：按海况设定艏摇阈值的双增益（高/低增益 5:1）操舵计算机，艏摇区内低增益减少无效舵动作与舵机磨损、越限后高增益快速回航——手动 weather adjust 的专利形态 |
| US3867712A | Adaptive filter | Honeywell Inc. | 1972 申请 / 1975 授权 | Honeywell 早期 DP 动力定位系统（1970 年代钻井船定位，推断对应） | 波浪滤波：两级中心频率可变的串联陷波滤波器，谱分析输出信号并按"三段频带功率各占 1/3"算法在线调整陷波频率，跟踪海浪谱中心频率与带宽变化——自适应（谱跟踪）陷波波浪滤波 |
| US4074648A | Adaptive autopilot for marine vessels | Sperry Rand Corp. | 1976 申请 / 1978 授权 | Sperry 自适应自动舵谱系（→ 后续 NAVIPILOT 系列，谱系推断） | 海况自适应增益：以性能指标 J=λ/U²·ψe²+δo²（计及船速的艏摇-舵角加权平方和）为目标，AGC 寻优控制器自动调节保向回路比例/微分增益，随风浪、航速变化保持近似临界阻尼并最小化推进损失（节能） |
| JPS59184099A | 船舶の自動操舵装置（Auto pilot device for ship） | 三菱重工業（Mitsubishi Heavy Industries） | 1983 申请 / 1984 公开 | 未见对应产品 | 海况自适应：由真风向运算单元与相对风向偏置设定单元给出"相对风向保持既定舷角"的设定航向，与固定航向设定可切换——恶劣海况下按风/浪向自动调整航向以防横摇加剧与倾覆 |
| EP0277737A2（优先权 US 07/010,861） | Autopilot with adaptive weather adjustment for marine vessels | Sperry Marine Inc. | 1988 申请（优先权 1987）/ 1988 公开 | Sperry Marine 自适应自动舵谱系（→ 现今 NAVIPILOT 4500N 的 self-adaptive 宣称，谱系推断） | 海况自适应增益调度：将 RMS 艏摇与瞬时艏摇分别与多档艏摇增量比较，自动选择对应增益控制信号并取最高者——把 US3604907A 的手动 weather adjust 全自动化的直接后继专利 |
| WO2000034837A1 | A method for automatic positioning of a vessel | ABB Industri AS（挪威；发明人 Thor I. Fossen） | 1999 申请（NO 优先权 1998）/ 2000 公开 | 未见对应产品（NTNU CyberShip I 水池模型试验验证；Fossen 个人简历页自述持有该 weather-optimal positioning 专利） | 海况自适应 / 节能（WOPC 气象最优定位）：无需测量风流浪即可使船舶绕移动圆心收敛到艏向对准平均环境合力的最小能耗姿态（摆锤比拟），即 DP 的自动 weather-vaning；为 Fossen & Strand 2001 年 *Automatica* WOPC 论文的专利形态 |
| EP2587321A1 | Wave filtering using differential evolution for dynamic positioning systems | Siemens AG（发明人 Tu Duc Nguyen） | 2011 申请 / 2013 公开 | 未见明确对应产品（西门子船舶 DP/电力推进业务） | 波浪滤波：用差分进化（DE）优化算法在线辨识测量信号中高频一阶波频（WF）分量与低频（LF）分量的幅值/频率/相位参数（二阶波频模型），剔除 WF 分量后将滤波信号送 DP 反馈回路 |
| US9340270B2 | Nautical automatic steering device, automatic steering method, and non-transitory computer-readable medium storing automatic steering program | Furuno Electric Co., Ltd.（古野电气） | 2014 申请（JP 优先权 2013）/ 2016 授权 | NavPilot 系列（推断对应） | 海况自适应 / 恶劣天气操舵：检测转首角速度反转时机自动发出"操舵停止"指令锁定舵角，角速度过零时"操舵重启"——抑制大风浪中航向偏差大幅摆动时过舵引发的蛇行发散，稳定保向并减少油耗 |
| NO20141217A1 | Minimering av kraftforbruk for automatisk styrte fartøy utsatt for eksterne forstyrrelser（受外部扰动自动操舵船舶的能耗最小化） | Kongsberg Maritime AS（发明人 Morten Breivik、Vincenzo Calabrò） | 2014 申请 / 2016 公开 | 未见公开对应产品（概念与 K-Pos DP 的 weather-vaning/能耗优化功能相近） | 海况自适应 / 节能：根据外部扰动信息在允许艏向域 Ψ 内计算并跟踪船舶最优艏向，同时在允许位置域 P 内做松弛 DP 定位/循迹，降低低频环境力能耗与高频扰动造成的执行器磨损——自动舵/DP 统一的"松弛约束+最优艏向"方案 |

#### 要点观察

- **海况自适应增益的 Sperry 谱系清晰可循**：US3604907A（1971，手动 weather adjust 阈值+双增益）→ US4074648A（1978，性能指标 J 寻优的全自动增益适应）→ EP0277737A2（1988，RMS 艏摇分档自动 weather adjustment），三件构成商用自动舵"气象调节"从手动到自适应的完整专利链，与今日 Sperry NAVIPILOT 4500N 官网"self-adaptive / 减少舵动作以节油"的宣称一脉相承。
- **DP 波浪滤波专利呈现"谱跟踪"主线**：Honeywell US3867712A（1975）已是按海浪谱实时调整陷波频率的自适应波浪滤波器；Siemens EP2587321A1（2013）改用群体智能优化（差分进化）辨识波频模型参数——两者与 Fossen 谱系的"波浪模型参数自适应（Hassani/Sørensen 极大似然自适应波浪滤波）"学术路线技术上同族。
- **Fossen/NTNU 谱系在本域的核心专利是 WOPC**（WO2000034837A1，ABB Industri 申请）：本质是"不测环境力、靠闭环收敛到最小能耗艏向"的海况自适应定位；Kongsberg NO20141217A1（2014）可视为同一思想在"允许艏向域+允许位置域"松弛约束下的工程化演进。
- **商用舵的"恶劣天气模式"确有专利支撑**：Furuno US9340270B2 的"角速度反转→停舵、过零→复舵"逻辑即是消费/商用级自动舵 rough-weather 稳定化的专利实例；三菱 JPS59184099A 则是更早期的"按风向自动调整设定航向"思路。
- **与本文其他节的交叉**：东京计器 US3739251A（1973，自动 weather setting 限幅电平，海况自适应的早期代表）已收录于 `02_japan.md`，本节不再重复；哈尔滨工程大学的 WOPC 类中国专利（CN104199464B 等）与 DP 波频模型参数估计专利（CN105867122B 等）见 `06_china.md`。
- **未能核实而丢弃的条目**：Tokimec JPH0443840B2（艏摇信号估计海况并调整操舵，Google Patents 限流无法打开、FreePatentsOnline 无该号映射）；Sperry US2796576A（1957，带 Weather Adjustment 旋钮的机动自动舵，FreePatentsOnline 仅余分类号残页）。宁缺毋滥，均未列入清单。

#### 产品-功能对照（厂商官方宣称）

- **Sperry Marine NAVIPILOT 4500N**（官网 Total Steering System 产品页，已核实）：宣称 NAVIPILOT 4500N 为 "adaptive autopilot that reduces fuel consumption by minimising rudder movements"（自适应自动舵，通过最小化舵动作降低油耗）；Riviera 报道（SAL 新船 IBS 交付）称其为 "self-adaptive NaviPilot 4500N autopilot, which dynamically adjusts to vessel characteristics and changing conditions"。与 US4074648A/EP0277737A2 的自适应增益谱系呼应。
- **Raytheon Anschütz NautoPilot 5000 / 5000 NX**（NP5000 NX 官方样本 PDF + NauticExpo 官方目录页，已核实）：ECOnomy Mode 即自适应模式——"自动舵持续监测船舶随载荷与海况变化的实际艏摇运动，识别并滤除周期性（对称）运动和因船舶惯性无法修正的运动，不予操舵修正"，一键减少舵动作实现持续节油；NP5000 目录页原文 "In Economy mode the intelligent adaptivity of NP 5000 analyses the yawing movements of the vessel"。属于典型的海况自适应波浪滤波商用宣称。
- **Kongsberg K-Pos DP**（K-Pos 操作手册 301027/C，经 PDFCOFFEE 镜像核实；K-Pos 官方样本见 NauticExpo 目录页）：扩展卡尔曼滤波器"根据噪声水平与量测更新率对艏向/位置量测做最优自适应滤波"，模型不确定性随时间降低、"the process is adaptive"——即 DP 内嵌的自适应波浪滤波宣称；与 NO20141217A1 的能耗优化艏向控制同属 K-Pos 技术语境。
- **Furuno NavPilot 系列**：NavPilot 700 官方页宣称 "self-learning and adaptive software algorithm"（详见 `02_japan.md` 的对照条目）；US9340270B2 的恶劣天气操舵停止/重启逻辑与该系列的自学习自适应软件定位一致（推断对应）。

#### 来源链接

- [US3604907A Steering command computer for navigable craft（Sperry Rand）— FreePatentsOnline](https://www.freepatentsonline.com/3604907.html)
- [US3867712A Adaptive filter（Honeywell，DP 自适应陷波波浪滤波）— FreePatentsOnline](https://www.freepatentsonline.com/3867712.html)
- [US4074648A Adaptive autopilot for marine vessels（Sperry Rand）— FreePatentsOnline](https://www.freepatentsonline.com/4074648.html)
- [JPS59184099A 船舶の自動操舵装置（三菱重工）— FreePatentsOnline](https://www.freepatentsonline.com/JPS59184099.html)
- [EP0277737A2 Autopilot with adaptive weather adjustment for marine vessels（Sperry Marine）— EPO 公布文本 PDF](https://data.epo.org/publication-server/rest/v1.0/publication-dates/19880810/patents/EP0277737NWA2/document.pdf)
- [WO2000034837A1 A method for automatic positioning of a vessel（ABB Industri / Fossen，WOPC）— FreePatentsOnline](https://www.freepatentsonline.com/WO2000034837A1.html)
- [Fossen 个人简历页（自述持有 weather-optimal positioning 专利）](https://fossen.biz/)
- [EP2587321A1 Wave filtering using differential evolution for dynamic positioning systems（Siemens）— FreePatentsOnline](https://www.freepatentsonline.com/EP2587321.html)
- [US9340270B2 Nautical automatic steering device（Furuno）— FreePatentsOnline](https://www.freepatentsonline.com/9340270.html)
- [NO20141217A1（Kongsberg Maritime，自动操舵船舶能耗最小化）— 挪威专利局扫描扉页 PDF（patentimages）](https://patentimages.storage.googleapis.com/0f/80/1b/9484f3085a778e/NO20141217A1.pdf)
- [Sperry Marine 官网：Total Steering System（NAVIPILOT 4500N adaptive autopilot 宣称）](https://www.sperrymarine.com/solutions/autopilot/total-steering-system/)
- [Anschütz NautoPilot 5000 NX 官方样本 PDF（ECOnomy Mode 自适应宣称）](https://aegeanel.com/wp-content/uploads/2023/03/brochure-np-5000-nx.pdf)
- [NauticExpo：Anschütz NautoPilot 5000 官方目录页（Economy mode intelligent adaptivity）](https://pdf.nauticexpo.com/pdf/anschuetz/nautopilot-5000/30676-122589.html)
- [Riviera：Raytheon Anschütz 发布 NautoPilot 5000NX（economy mode 按载荷与海况监测艏摇）](https://www.rivieramm.com/news-content-hub/news-content-hub/raytheon-anschutz-launches-fuel-saving-autopilot-system-74553)
- [Kongsberg K-Pos DP (OS) 操作手册镜像（EKF 自适应噪声/波浪滤波章节）](https://pdfcoffee.com/kongsberg-k-pos-dp-os-dynamic-positioning-system-operator-manual-pdf-free.html)
- [NauticExpo：Kongsberg K-Pos Dynamic Positioning 官方样本目录页](https://pdf.nauticexpo.com/pdf/kongsberg-maritime/kongsberg-k-pos-dynamic-positioning/31233-38087.html)
