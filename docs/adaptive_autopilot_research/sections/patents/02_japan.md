### 日本厂商专利（Tokyo Keiki / YOKOGAWA / Furuno / JRC）

本节覆盖日本厂商——东京计器（Tokyo Keiki，PR 系列自动舵）、横河（YOKOGAWA，原横河电机/横河北辰电机，现 YDK Technologies，PT 系列自动舵与 BNAAC 自适应操舵）、古野电气（Furuno，NavPilot 系列）、日本无线（JRC）——在船舶自动舵/操舵自适应控制方向的已公开专利。核实方式：US/EP 条目逐页打开 Google Patents 核实（公开号、申请人、日期、法律状态以页面为准）；JP 条目因检索期间 Google Patents 持续限流（HTTP 503），改为逐页打开 FreePatentsOnline 的 JPO 著录页核实申请人、申请/公开日期与摘要。

#### 专利清单

| 公开号/授权号 | 名称 | 申请人 | 申请/公开年份 | 对应商用产品 | 对应算法/主题 |
|---|---|---|---|---|---|
| US3739251A | Marine autopilot system | Tokyo Keiki Co., Ltd.（东京计器） | 1971 申请（优先权 1970）/ 1973 授权 | PR 系列早期自动舵（自动气象调节功能的源头，具体型号不可考） | 海况自适应：根据外扰自动设定限幅电平（自动 weather setting），抑制无效高频操舵以节能、减少舵机磨损 |
| US4336594A | Automatic steering apparatus for ships | Tokyo Keiki Co., Ltd.（东京计器） | 1978 申请（优先权 1977，JPS60277B2）/ 1982 授权 | 未见明确对应产品 | 非线性补偿：小航向偏差区提高补偿增益，抑制肥大型船"异常现象"导致的航向不稳定（狩猎抑制） |
| EP4116186A1（同族 WO2021177213A1、JPWO2021177213A1） | Steering device | Tokyo Keiki Inc.（东京计器） | 2021 申请（优先权 2020）/ 2023 公开（EP 已撤回，JP 同族有效中） | 面向 Gate Rudder 双舵板船的操舵装置，未见具体产品型号 | 操舵模式切换：按转首角速度/对水速度/电子海图区域切换双舵板平行（节能优先）与内倾（舵效优先）模式 |
| JPS62143795A | Automatic steering system for vessel（船舶自动操舵系统） | 三菱重工 + 横河電機（Yokogawa Electric） | 1985 申请 / 1987 公开 | 未见对应产品 | 节能操舵：多位置（multiposition）控制形式驱动舵机，以最小操舵频度稳定操舵、抑制 hunting |
| JP3751239B2（公开号 JP2003-104291A） | 自動操舵装置及び自動操舵方法 | 横河電子機器（Yokogawa Denshikiki，现 YDK Technologies） | 2001 申请 / 2006 授权 | PT 系列自适应自动舵的减摇功能（具体型号未公开） | 舵减摇（RRS）：控制型多元自回归模型（被控量=横摇角速度+航向偏差，操作量=舵角）+ 外扰适应，兼顾航向保持与横摇抑制 |
| JP2008230484A | 自動操舵装置及び自動操舵方法 | 横河電子機器（YDK） | 2007 申请 / 2008 公开 | PT500A 系列（航向变动抑制型，推断对应） | 控制型 AR 模型 + 最优操舵量（LQG/自校正调节器谱系）；设虚拟设定航向测量航迹偏差并自动修正固定偏差 |
| JP2011042284A（申请号 JP2009-192384） | 自動操舵装置及び自動操舵方法 | 横河電子機器（YDK） | 2009 申请 / 2011 公开 | PT900A 的 BNAAC（Batch Noise Adaptive Autopilot Controller） | 海况自适应/节能：按"批量区间"（batch section）周期更新船体运动模型与控制增益，区间内以更短控制周期运算指令舵角，抑制多余操舵——与 BNAAC 的官方原理描述（持续更新船舶参数以反映气象海况变化）一致 |
| JP2014136509A（申请号 JP2013-006345） | 自動操舵装置 | 横河電子機器（YDK） | 2013 申请 / 2014 公开 | PT900A / BNAAC（推断对应） | 节能自适应：基于预先获取的运动性能与航行中取得的外扰特性，将船体运动模型辨识为控制型多元 AR 模型，用最优控制增益输出舵角指令，减少外扰引起的无效操舵 |
| US4768153A | Automatic pilot system | Furuno Electric Co., Ltd.（古野电气） | 1985 申请（优先权 1984）/ 1988 授权 | 未见明确对应型号（Loran C/Omega 时代产品） | 航迹保持前身：电子导航接收机实测船位与罗经推算船位比较，自动求出风流压差/罗经误差修正角并修正航向 |
| US9725151B2 | Automatic steering device and automatic steering method | Furuno Electric Co., Ltd.（古野电气） | 2013 申请（优先权 2012）/ 2017 授权 | NavPilot 300（Fantum Feedback 免舵角反馈安装） | 免舵角传感器的脉冲式操舵：记忆上次指令舵角、按目标舵角变化量脉冲驱动；支持增益自动调整的自适应 PD 控制 |
| US11383813B2 | Automatic steering device, automatic steering method and automatic steering program | Furuno Electric Co., Ltd.（古野电气） | 2019 申请（优先权 2018）/ 2022 授权 | NavPilot 系列 NAV 模式/航线转向（推断对应） | 航线转向跟踪：基于船体响应（追随性 T、旋回性 K）推算船前方中间航路点并提前计算指令舵角，使船舶更准确沿折线航线转向（前馈+反馈） |

#### 要点观察

- **横河系（YDK）专利是大津（Ohtsu）学派的直接产业化**：JP2008230484A、JP2011042284A、JP2014136509A 均以"控制型自回归（AR）模型 + 最优控制增益"为核心，即 Ohtsu 等 1979 年 *Automatica* 论文确立的随机模型/LQG 路线的专利化形态；JP2011042284A 的"按批量区间更新模型与增益"与 BNAAC（Batch Noise Adaptive Autopilot Controller）的官方原理说明逐点对应，可视为 BNAAC 的核心专利。JP3751239B2 则把同一 AR 模型框架扩展到舵减摇。学术侧另有 Arie 等 1986 年 IEEE Control Systems Magazine 论文记载 NKK 与横河北辰电机联合开发的自适应操舵系统（保向用 hill-climbing、转向用 MRAC），但该系统的对应专利未能在公开库中核实到条目。
- **东京计器的专利谱系**从 1970 年代的自动气象调节（海况自适应限幅，US3739251A）、1980 年代肥大型船非线性补偿（US4336594A），演进到当前 PR-9000 的 NCT 自适应控制与 ACE 航线控制；近年专利（EP4116186A1 族）转向新舵型（Gate Rudder）的操舵模式切换。NCT/ACE 的具体算法细节未见对应公开专利（检索期间未找到可核实的 NCT 专利条目）。
- **古野的专利面向游艇/渔船小型舵机市场**：免舵角反馈（Fantum Feedback，US9725151B2）、自适应学习参数、基于船体响应的转向跟踪（US11383813B2），与其 NavPilot 产品宣称的"self-learning, adaptive software algorithm"直接呼应。
- **JRC（日本无线）**：本轮检索未找到 JRC 名下与"自适应操舵/航向控制算法"直接对应且可核实的公开专利。JRC 的自动舵/航迹控制更多体现为 ECDIS 与外部自动舵（如横河/YDK 自动舵）联动的 Track Control System 集成方案（见 JRC JMR-7230-S 手册中"将 Autopilot 操舵模式切换为 NAVI 模式（e.g. YOKOGAWA A/P）"的操作流程），而非自有控制算法专利。

#### 产品-功能对照（厂商官方宣称）

- **Tokyo Keiki PR-9000**（官网产品页，已核实）：选装的 Adaptive Control（NCT : Notable Control Technology）"自动计算适应船舶操纵运动特性与外部影响的合适设定，使船舶'自适应'，显著减少波浪等外部影响导致的多余舵动作，实现节能航行"；另有选装 Route Control（ACE : Advanced Control for Ecology，直线航路控制，自动计算潮流等外扰并实施最佳舵控制）与 TCS（Track Control System，与 ECDIS 联动的航迹保持并自动修正漂移）。2024 年 ACE（Straight Leg Course Control）获日本海事协会 ClassNK"Innovation Endorsement"认证。
- **YDK（横河電子機器）PT900A / BNAAC**（川崎汽船 2010 年新闻稿，已核实）：BNAAC "通过持续监测并反映气象、海况变化来随时更新船舶参数，而原系统只反映初始参数设定"；K Line 与 YDK 用营运散货船试验，确认约 1% 的 CO₂ 减排（节油）。PT900 系列分 PT900A（自适应，标配 BNAAC）与 PT900D（PID）两型。
- **Furuno NavPilot 系列**（Furuno USA 官网 NavPilot 700 产品页，已核实）：宣称"self-learning and adaptive software algorithm……航行中动态调整船速、纵倾、吃水、潮流、风、死区、气象等关键参数并持续优化存入系统存储器"；Fantum Feedback "使舷外机/尾机安装不再需要物理舵角反馈单元，同时提供增强的操舵控制"（与 US9725151B2 对应）。NavPilot 300 在 Furuno USA 产品目录中直接标注为 "Adaptive Autopilot"。
- **JRC**：未检索到 JRC 自动舵产品的"自适应控制"官方宣称；其公开资料强调 JAN 系列自动舵与 ECDIS 的 TCS 集成（见上）。

#### 来源链接

- [US3739251A Marine autopilot system（东京计器）— Google Patents](https://patents.google.com/patent/US3739251A/en)
- [US4336594A Automatic steering apparatus for ships（东京计器）— Google Patents](https://patents.google.com/patent/US4336594A/en)
- [EP4116186A1 Steering device（东京计器）— Google Patents](https://patents.google.com/patent/EP4116186A1/en)
- [JPS62143795A 船舶自动操舵系统（三菱重工+横河電機）— FreePatentsOnline](https://www.freepatentsonline.com/JPS62143795A.html)
- [JP3751239B2 自動操舵装置及び自動操舵方法（横河電子機器，舵减摇）— FreePatentsOnline](https://www.freepatentsonline.com/JP3751239B2.html)
- [JP2008230484A 自動操舵装置及び自動操舵方法（横河電子機器）— FreePatentsOnline](https://www.freepatentsonline.com/JP2008230484A.html)
- [JP2011042284A 自動操舵装置及び自動操舵方法（横河電子機器，BNAAC 对应）— FreePatentsOnline](https://www.freepatentsonline.com/JP2011042284A.html)
- [JP2014136509A 自動操舵装置（横河電子機器）— FreePatentsOnline](https://www.freepatentsonline.com/JP2014136509A.html)
- [US4768153A Automatic pilot system（古野电气）— Google Patents](https://patents.google.com/patent/US4768153A/en)
- [US9725151B2 Automatic steering device and automatic steering method（古野电气）— Google Patents](https://patents.google.com/patent/US9725151B2/en)
- [US11383813B2 Automatic steering device, method and program（古野电气）— Google Patents](https://patents.google.com/patent/US11383813B2/en)
- [Tokyo Keiki PR-9000 Autopilot 官方产品页（NCT/ACE/TCS 功能说明）](https://www.tokyokeiki.jp/e/products/detail.html?pdid=273)
- [Tokyo Keiki 新闻：Straight Leg Course Control（ACE）获 ClassNK Innovation Endorsement](https://www.tokyokeiki.jp/e/topics.html?itemid=676&dispmid=1125)
- [川崎汽船新闻稿：YDK BNAAC 最新航向控制系统扩大装船（约 1% CO₂ 减排）](https://www.kline.co.jp/en/news/other/other1641459226107681229.html)
- [Furuno USA NavPilot 700 官方产品页（self-learning adaptive algorithm / Fantum Feedback）](https://www.furunousa.com/en/products/navpilot_700)
- [Arie et al., "An adaptive steering system for a ship"（NKK+横河北辰，IEEE CSM 1986）— NASA/ADS](https://ui.adsabs.harvard.edu/abs/1986ICSyM...6....3A/abstract)
