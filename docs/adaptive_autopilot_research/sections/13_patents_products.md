## 相关产品与专利

本章汇总与船舶自动舵自适应控制直接相关的公开专利，并给出"公司 — 产品 — 专利"的对应关系。检索覆盖 Google Patents、Espacenet/EPO publication server、FreePatentsOnline 及厂商官方资料；**每件专利均逐页打开核实**（公开号、申请人、年份与摘要一致才收录），检索不到可核实条目的厂商如实标注"未检出"，不凑数。专利法律状态与同族关系以官方数据库（USPTO/EPO/JPO/CNIPA）为准，本章信息仅供技术调研参考。

### 公司 — 产品 — 专利映射总览

| 公司 | 代表产品 | 已核实专利（择要） | 对应本文主题 |
|------|---------|------------------|-------------|
| Sperry Marine（Sperry Rand → Northrop Grumman） | NAVIPILOT 4000/4500N（self-tuning fully adaptive） | US4074648A（1978，性能指标 J 寻优自适应）、US4692868A（1987，迭代增益搜索）、US4799163A（1989，海况自适应）、US3604907A / EP0277737A2（weather adjustment 谱系） | 自适应 PID / 海况自适应（第 3、8 章） |
| Raytheon Anschütz | NautoPilot 5000/NX（ECOnomy 模式） | US3517285A（1970，历史谱系）、US5179905A（1993，按横摇频率自适应微分阻尼）、US5331558A（1994，自适应死区）；近代 ECO 功能未检出公开专利 | 波浪滤波 / 节能（第 8 章） |
| Kongsberg（原 Simrad） | K-Bridge Autopilot（"self-adapting"） | 自适应操舵算法未检出可核实专利（疑以商业秘密保护）；NO20141217A1（允许艏向域 + 松弛 DP 能耗最小化） | 节能 / 第 3 章 |
| Tokyo Keiki（东京计器） | PR-9000（NCT 自适应、ACE 节能） | US3505577A（1970，恶劣海况自动降灵敏度，ECO 模式鼻祖）、US3739251A（1973，自动气象调节）、EP4116186A1（Gate Rudder 模式切换）；NCT 算法本身未检出公开专利 | 海况自适应 / 节能（第 3、8 章） |
| 横河电子机器（YDK，横河电机系） | PT500A / PT900A（BNAAC） | JP2011042284A、JP2014136509A（批量区间更新 AR 模型 + 最优增益，与 BNAAC 官方原理逐点对应）、JP2008230484A（AR 模型航向保持）、JP3751239B2（舵减摇） | STR/自校正（第 2 章）、舵减摇（第 10 章） |
| Furuno（古野电气） | NavPilot 系列（self-learning / Fantum Feedback） | US9725151B2（免舵角反馈）、US11383813B2（基于 K/T 的前馈转向）、US9340270B2（大风浪防蛇行发散）、JP6513677B2（表层流/风在线估计）、JP2019196103A（燃費最优旋回） | 自适应 PID / 海况自适应 / 节能 |
| Garmin | GHP/Reactor（Shadow Drive，10-K 明确 "patented"） | US20190204837A1（舵轮脱开）、US12632049（2026，按船动力学与风流条件的路径级自适应 + 避碰） | 自适应 PID / 航迹（第 3、9 章） |
| 荷兰谱系（VRH/Imtech） | ASSA 自适应操舵减摇自动舵（M 级/F124 护卫舰） | EP0129287B1（van der Klugt & van Amerongen，1984 申请） | MRAC / 舵减摇（第 1、10 章） |
| 美国海军（NSWCCD） | 舰载 RRS 系统 | US4380206A / US4388889A / US4398486A（Baitis & Woolaver，统计变增益 = 海况自适应 RRS） | 舵减摇（第 10 章） |
| ABB（Fossen 谱系） | WOPC 气象最优定位 | WO2000034837A1（发明人 Thor I. Fossen） | 海况自适应（第 8 章） |
| Honeywell / Siemens / Q-Tagg | DP / 推进控制 | US3867712A（1975，自适应陷波波浪滤波）、EP2587321A1（2013，差分进化在线辨识波频模型）、CN103813958B（推进-横摇联合控制） | 波浪滤波（第 8 章） |
| 海兰信 | SCS 操舵仪（上市文件列明的核心专利） | CN101281409A/B（双 CAN 冗余自动操舵仪） | 系统架构 |
| 中国高校/研究所（大连海事、哈工程、上海海事、中船 707 所等） | 多为算法专利，未见对应商用产品 | CN101615000A（闭环增益成形+神经网络逆）、CN100589055C（递归模糊神经网络+滑模）、CN110703605A/B（自适应模糊最优）、CN116699995A（GESO 海浪滤波 ILQG）、CN113031614B（复合优化节油）、CN114879671B（RL-MPC 航迹跟踪）、CN118605188B（CNN 整定 LQR）等 | 模糊 / NN / 滑模 / RL / 节能（第 4–7 章） |
| 三菱重工 / 三菱造船 / 商船三井 | 入出港操纵支援 | JPS59184099A（1984，恶劣海况自动调设定航向）、JPS62143795A（节能多位置操舵，三菱重工+横河）、JP7804271B2（DRL 入出港操纵，已授权）、WO2024248067A1（MOL 操船规则推定） | 海况自适应 / 节能 / RL（第 7、8 章） |

### 检索方法与可信度说明

- **核实途径**：每件专利至少经一个官方/准官方全文源（Google Patents 专利页、EPO publication server 授权文本、FreePatentsOnline 著录页、patentimages 扫描扉页）逐页核实；核实途径在各项下注明。
- **负面发现同样记录**：Kongsberg/Simrad、Anschütz 近代产品、东京计器 NCT、JRC、丹麦 Blanke 谱系、韩国 Avikus/三星重工均未检出可核实的自适应操舵算法公开专利——合理的解释是核心算法多以商业秘密保护，公开专利更多落在系统架构、模式切换与外围功能上。这些厂商仅有官方产品宣称佐证（见各小节）。
- **高校专利与产品脱节是普遍现象**：中国高校/研究所专利算法覆盖最全（模糊、NN、滑模、LQG、RL 均有），但除海兰信 SCS 外均未对应商用产品；欧美日厂商则相反——产品有明确自适应功能宣称，专利却多在早期或外围。
- **历史谱系的看点**：从 Sperry 1957–1978 的 weather adjustment 与 J 寻优专利，到东京计器 1967 年"海况恶劣自动降灵敏度"，再到 YDK 的 BNAAC（自校正思想的产品化）与 Garmin 2026 年的路径级自适应——专利谱系与本文第 1–3 章的学术谱系（MRAC→STR→增益调度+自适应 PID）完全同构，互相印证。

以下按专利域分节列出明细。

### 欧美老牌厂商专利（Sperry Marine / Raytheon Anschütz / Kongsberg-Simrad / Garmin）

本节覆盖欧美老牌厂商——Sperry Marine（Sperry Rand → Sperry Corp → Litton Marine → Northrop Grumman Sperry Marine 谱系，NAVIPILOT 系列）、Raytheon Anschütz（NautoPilot / PilotStar 系列，含 Anschütz & Co 的历史专利）、Kongsberg（原 Simrad / Robertson 自动舵线，K-Bridge Autopilot）、Garmin（GHP / Reactor 系列，Shadow Drive）——在船舶自动舵/操舵自适应控制方向的已公开专利。核实方式：逐条打开 Google Patents 专利页核实（公开号、申请人、申请/公开日期、摘要、法律状态以页面为准）；检索期间 Google Patents 对直接访问持续限流（HTTP 503），部分条目经 r.jina.ai 阅读代理打开同一 Google Patents 页面核实；US12632049 因授权时间很新、Google Patents 尚未收录，改用 Justia 专利页与 USPTO 官方公报（Official Gazette）交叉核实。

#### 专利清单

| 公开号/授权号 | 名称 | 申请人 | 申请/公开年份 | 对应商用产品 | 对应算法/主题 |
|---|---|---|---|---|---|
| US3517285A | Electrical automatic pilot for rudder-controlled vehicles, particularly for ships | Anschütz & Co GmbH（页面所列当前受让人为 Raytheon Anschuetz GmbH） | 1968 申请（优先权 1967，DE）/ 1970 授权 | 未见明确对应型号（Anschütz 操舵仪历史谱系，PilotStar/NautoPilot 的前身） | 历史谱系：电气式自动操舵仪（模拟 PID 时代），证明 Anschütz 自动舵专利谱系可追溯到 1960 年代 |
| US4074648A | Adaptive autopilot for marine vessels | Sperry Rand Corp.（后归 Litton Marine Systems） | 1976 申请 / 1978 授权 | Sperry 自适应自动舵谱系源头（ADG / SRP 系列，具体型号不可考） | 增益调度 + 节能：以性能指标 J（航向误差平方 + 舵令平方，含船速平方加权）寻优的自动增益控制（AGC）；保向/转向双回路切换；转向增益按船速平方反比调度 |
| US4692868A（同族 EP0189248B1、JPS61169392A、DE3675953D1） | Adaptive autopilot | Sperry Corp. | 1985 申请 / 1987 授权 | Sperry ADG 3000VT Adaptive Digital Gyropilot（推断对应） | 海况自适应 + 节能：三模式迭代步进增益寻优（以 RMS 航向误差为判据，在保持航向控制前提下搜索最小船体+舵阻力点），无需先验船舶模型——自校正思想的早期工程化 |
| US4799163A | Autopilot with adaptive weather adjustment for marine vessels | Sperry Marine Inc. | 1987 申请 / 1989 授权 | Sperry 自动舵的自动气象调节（weather）功能（对应 NAVIPILOT 前身产品的 weather setting） | 海况自适应：根据实测艏摇统计量自动调整增益/天气设定，替代人工 weather 档位 |
| US5179905A | Adaptive autopilot | Raytheon Co.（Raytheon Marine，后并入 Raytheon Anschütz） | 1991 申请 / 1993 授权 | Raytheon 商用船自动舵（PilotStar 谱系，具体型号未见对应） | 波浪滤波 / 海况自适应增益：保向用 PID、转向用 PD；微分（阻尼）项增益按船舶横摇频率自适应——顶浪（横摇频率超阈值）时将微分项置零，避免无效操舵与舵机磨损 |
| US5331558A | Autopilot having an adaptive deadband feature | Raytheon Co. | 1991 申请（与 US5179905A 同优先权日）/ 1994 授权 | 同上（Raytheon / PilotStar 谱系） | 波浪滤波 / 节能：自适应死区——按最大允许航向误差、最大允许角速度、船速计算四组死区值并做冗余表决，另一路死区补偿计程仪/罗经/横摇传感器噪声，减少无效操舵 |
| US20190204837A1（同族含 US10073453B2 等授权件） | Autopilot features for marine navigation | Garmin Switzerland GmbH | 2016 优先权 / 2019 公开 | Garmin GHP 10/20、Reactor 系列的 Shadow Drive（舵轮转动自动脱开自动舵） | 人机接管安全（非控制律本身）：舵柄运动量传感器无线发送至船上网络，操舵量超过脱开阈值时自动舵暂时脱开、人工接管——Garmin 年报中"patented Shadow Drive"的对应专利族 |
| US12632049（申请公开号 US20230195118A1） | Autonomous marine autopilot system | Garmin International, Inc. | 2022 申请（临时申请 2021）/ 2026 授权 | 未见明确对应产品（方向对应 Garmin Auto Guidance 自动航线与自动靠泊功能） | 自主航行 / 海况自适应路径控制：基于海图与众包数据生成路径，按船舶动力学与风、流条件沿路径控制，传感器探测并分类碍航物后实时更新轨迹避碰 |

#### 要点观察

- **Sperry 谱系是"自适应自动舵"专利化的起点**：US4074648A（1978）把"以性能指标寻优的自动增益调整"写入专利，比 van Amerongen 的 MRAC 自适应舵论文（1980 年代初）更早，且明确以"减少推进损失（节油）"为目标；US4692868A 进一步去掉对船舶模型的先验要求（RMS 误差驱动的迭代增益搜索），US4799163A 则把人工 weather 档位自动化。三者与当代 NAVIPILOT 4000/4500N 宣称的"self-tuning, fully adaptive heading control"一脉相承（见产品-功能对照）。
- **Raytheon（含后来的 Raytheon Anschütz）1990 年代专利聚焦"减少无效操舵"**：US5179905A 按横摇频率自适应调整微分阻尼（顶浪置零微分项），US5331558A 做自适应死区——本质都是波浪频率段操舵抑制（波浪滤波的工程实现），与 Anschütz 现行 NautoPilot 5000NX Economy Mode 宣称的"滤除因船体惯性无法修正的对称艏摇运动、不纠正"高度对应。
- **Anschütz 自有专利谱系（Anschütz & Co GmbH）可核实条目早至 1968 年**（US3517285A，Google Patents 页面所列当前受让人即为 Raytheon Anschuetz GmbH，直接印证品牌传承）；但 NautoPilot 系列近代自适应算法（NP 5000/5300/5500 的"fully adaptive"宣称）未检索到可核实的对应公开专利——其算法细节可能以商业秘密形式保护。
- **Kongsberg / Simrad（Robertson）**：本轮检索未能找到 Kongsberg Maritime 或 Simrad/Robertson 名下与"自适应操舵/航向控制算法"直接对应且可核实的公开专利条目。Simrad 的自动舵历史极早（1951/1952 年世界首台渔船自动舵 AP1），Kongsberg K-Bridge Autopilot 官方宣称"self-adapting……'learns' how the vessel reacts to different rudder commands"（见下），属自适应（学习式）操舵的官方表述，但对应专利未检出。Navico（Simrad 母公司）与 Garmin 2015–2018 年的专利诉讼涉及 sonar 与 Auto Guidance 专利，说明两家的专利布局集中在航线引导而非操舵控制律。
- **Garmin 的自动舵专利面向游艇市场**：Shadow Drive 族（US20190204837A1）解决"自动舵与人工操舵冲突"的安全接管问题；US12632049（2026 授权）把自动舵扩展到自主路径规划与避碰，控制量考虑"船舶动力学与风、流条件"，属环境自适应的路径级控制，而非舵角环的自适应控制律。

#### 产品-功能对照（厂商官方宣称）

- **Sperry Marine NAVIPILOT 4000**（NauticExpo 产品样本，已核实）：宣称 "capable of tuning itself to adapt automatically to the ship's load characteristics and weather conditions"（Fully self-tuning, adaptive heading control；可按海况手动选择操舵策略；节油）。
- **Sperry Marine NAVIPILOT 4500N**（2022 年发布，MarineLink/Smart Maritime Network 报道，已核实）："networked, self-tuning, fully adaptive heading control system"，能针对单船单航次的 squat、纵倾、装载调整舵控制，并实时适应燃油消耗引起的纵倾变化与船舶老化等长期行为变化。
- **Raytheon Anschütz NautoPilot 5000NX**（2023 年发布，Riviera Maritime Media 报道，已核实）：Economy Mode "permanently monitors the actual yawing movements of the vessel, depending on load and sea conditions. Symmetrical movements, as well as movements that cannot be corrected due to the inertial behavior of the ship, are filtered out and not corrected"——与 US5179905A/US5331558A 的波浪段抑制思想一致；另有 Course Control（COG 航迹+漂移补偿）与双舵 toe angle 节能功能。NP 5300/5500 官方定位为 "fully adaptive autopilot"。
- **Kongsberg K-Bridge Autopilot**（Kongsberg 官方数据单，已核实）："self-adapting, automated track and heading controller……'learns' how the vessel reacts to different rudder commands and adapts its steering commands accordingly"，即学习式自适应操舵的官方宣称；Kongsberg 官网 K-Nav 页面进一步说明该学习主要在海试/安装配置阶段进行。
- **Simrad 自动舵谱系**（Simrad AP70/AP80 产品样本，已核实）：宣称传承自 1951 年世界首台渔船自动舵 AP1，"over 60 years of experience in the autosteering business"。
- **Garmin Shadow Drive**（Garmin 10-K 年报（SEC 存档）与 GHP 20 产品页，已核实）："patented Shadow Drive™ technology, which automatically disengages the autopilot if the helm is turned"——对应 US20190204837A1 专利族。

#### 来源链接

- [US3517285A Electrical automatic pilot for rudder-controlled vehicles（Anschütz & Co）— Google Patents](https://patents.google.com/patent/US3517285A/en)
- [US4074648A Adaptive autopilot for marine vessels（Sperry Rand）— Google Patents](https://patents.google.com/patent/US4074648A/en)
- [US4692868A Adaptive autopilot（Sperry Corp）— Google Patents](https://patents.google.com/patent/US4692868A/en)
- [EP0189248B1 Adaptive autopilot & method of heading keeping（Sperry，US4692868A 同族）— Google Patents](https://patents.google.com/patent/EP0189248B1/en)
- [US4799163A Autopilot with adaptive weather adjustment（Sperry Marine）— Google Patents](https://patents.google.com/patent/US4799163A/en)
- [US5179905A Adaptive autopilot（Raytheon）— Google Patents](https://patents.google.com/patent/US5179905A/en)
- [US5331558A Autopilot having an adaptive deadband feature（Raytheon）— Google Patents](https://patents.google.com/patent/US5331558A/en)
- [US20190204837A1 Autopilot features for marine navigation（Garmin，Shadow Drive 族）— Google Patents](https://patents.google.com/patent/US20190204837A1/en)
- [US12632049 Autonomous marine autopilot system（Garmin）— Justia Patents](https://patents.justia.com/patent/12632049)
- [USPTO Official Gazette 专利权人索引（Singh/Helms/Ellebrecht → Garmin，US12632049 交叉核实）](https://patentsgazette.uspto.gov/week20/OG/patentee/alphaS_Utility.html)
- [Sperry NAVIPILOT 4000 产品样本（self-tuning adaptive）— NauticExpo](https://pdf.nauticexpo.com/pdf/sperry-marine/marine-autopilot-navipilot-4000/25551-77437.html)
- [Sperry Marine 发布 NAVIPILOT 4500N 自整定自适应自动舵（2022）— MarineLink](https://www.marinelink.com/news/sperry-marine-debuts-selftuning-autopilot-499246)
- [Raytheon Anschütz 发布 NautoPilot 5000NX 节能自动舵（2023）— Riviera Maritime Media](https://www.rivieramm.com/news-content-hub/news-content-hub/raytheon-anschutz-launches-fuel-saving-autopilot-system-74553)
- [Kongsberg K-Bridge Autopilot 官方数据单（self-adapting / learns）](https://www.kongsbergmaritime.com/contentassets/52c907fdef4f4cffafbf86fcebc46434/308695e---autopilot.pdf)
- [Kongsberg 官网 K-Nav 页面（自动舵海试学习机制说明）](https://www.kongsbergmaritime.com/products/bridge-systems-and-control-centres/navigation-systems/k-nav/)
- [Simrad AP70/AP80 产品样本（AP1 1951 年首台渔船自动舵）](https://www.mackaycomm.com/wp-content/uploads/2021/08/SimradPro_AP70-80_Brochure_0911_Mackayv01MR.pdf)
- [Garmin 10-K 年报（patented Shadow Drive 表述）— SEC](https://www.sec.gov/Archives/edgar/data/1121788/000095017024047315/grmn-ars-20231230.pdf)
- [Navico 与 Garmin 专利纠纷和解（2018）— Sail World](https://www.sailworldcruising.com/news/201863/Navico-and-Garmin-resolve-patent-disputes)

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

### 舵减摇与联合控制专利（RRS：美国海军 / 荷兰 van Amerongen 谱系 / 德国 B+V 谱系 / 商用 RRS）

本节覆盖"舵减摇（Rudder Roll Stabilization, RRS）与操舵-减摇联合控制"专利域：van Amerongen / van der Klugt 荷兰（代尔夫特理工 → Van Rietschoten & Houwens → Imtech）谱系、Blanke 丹麦谱系、美国海军 DTNSRDC 谱系（Baitis/Woolaver）、德国 Blohm+Voss 液压舵机谱系，以及商用 RRS 功能（Imtech ASSA、B+V Industries SIMPLEX-COMPACT、Becker Marine Systems RRS）对应的专利。核实方式：US 条目逐页打开 Google Patents 或 FreePatentsOnline 核实（公开号、申请人、日期以页面为准）；EP 条目经 EPO publication server 打开授权文本 PDF 核实；CN 条目打开授权公告文本首页（图像页）核实著录项目。**检索期间 Google Patents 多次持续限流（HTTP 503），部分条目改用 FPO / EPO / CNIPA 文本核实。**日本厂商的舵减摇专利（横河 YDK JP3751239B2）见 `02_japan.md`，本节不重复。

#### 专利清单

| 公开号/授权号 | 名称 | 申请人 | 申请/公开年份 | 对应商用产品 | 对应算法/主题 |
|---|---|---|---|---|---|
| US4380206A | Ship roll stabilization system | 美国海军部长（发明人 A.E. Baitis, D.A. Woolaver，DTNSRDC） | 1981 申请 / 1983 授权 | 美国海军 RRS 舰载系统（CG/FFG 级海试，未见商用产品） | 舵减摇+联合控制：横摇角速度反馈叠加舵令（velocity control, δ=K·φ̇），增益 K 按近期横摇统计量实时计算，使舵速率逼近但不越过饱和限值——典型的"海况自适应变增益"思想（增益调度/海况自适应） |
| US4388889A | Electrical actuator for ship roll stabilization | 美国海军部长（Woolaver, Baitis, Nigon） | 1981 申请 / 1983 授权 | 同上（US4380206A 的电子执行器分案，申请号 06/248389 与之互引） | 海况自适应增益电路：rate gain circuit 按前一时间窗统计平均横摇角速度自适应确定增益，防止舵速率饱和失稳 |
| US4398486A | Mechanical actuation device for ship roll stabilization | 美国海军部长（Woolaver, Baitis） | 1981 申请 / 1983 授权 | 同上（机械执行机构分案，申请号 06/247485 与 US4380206A 互引） | 舵减摇执行机构：RRS 操舵支路与常规操舵支路的离合/解耦机械执行装置 |
| EP0129287B1（申请号 84200833.6，优先权 NL8302077；A1 公开 1984-12-27） | Control System | Van Rietschoten & Houwens Elektrotechnische Maatschappij B.V.（鹿特丹；发明人 P.G.M. van der Klugt, J. van Amerongen） | 1983 优先权 / 1984 申请 / 1987 授权 | VRH（后并入 Imtech）RRS 自动舵 → Imtech ASSA（Adaptive Steering and Stabilising Autopilot，荷兰 M 级/LCF、德国 F124 护卫舰） | 自适应增益控制：处理器比较控制信号变化率与执行器允许最大变化率，超限时暂时按超限程度反比降低放大系数——即"舵机速率受限下的抗饱和自适应增益"，是荷兰 RRS 控制算法的核心专利（van Amerongen 个人主页专利清单与该申请号逐一吻合，并称同时在美国、日本申请） |
| US4919064A（同族 DE3322505A1、EP0129810B1、JPS6012395A、KR910005300B1） | Hydraulic system for ship rudder roll stabilization and steering | Blohm & Voss GmbH（现权利人 SKF Marine GmbH） | 1983 优先权（DE）/ 1990 授权（US） | B+V Industries SIMPLEX-COMPACT® RRS 用 Rotary Vane 舵机；Becker Marine Systems RRS 联合方案 | 舵减摇液压系统：恒流量泵+蓄能器支持 45°/s 级高速转舵、比例阀按操舵指令与减摇信号控制流量/方向、负载补偿抑制波浪对舵的扰动；非减摇时切换常规操舵泵组。JPS6012395A 即该族的日本公开（舵減揺 特許的直接对应） |
| US8567331B2（公开 US20100147204A1） | Rudder roll stabilization by nonlinear dynamic compensation | University of Wyoming（发明人 J.F. O'Brien） | 2009 申请 / 2013 授权（已因费用失效） | 未见对应产品 | 舵减摇抗饱和非线性控制：高阶 Nyquist 稳定回路 + 多路反馈非线性动态补偿（NDC），按 Popov 判据保证舵角/舵速率饱和下绝对稳定，宣称较 PD 型 RRS 提升 20–40% 减摇率 |
| CN103813958B（同族 WO2013039445A1，PCT/SE2012/050955，优先权 SE1130084-5） | 用于防止和衰减船的横摇的方法 | Q-TAGG 研发公司（Q-Tagg R&D AB，瑞典韦斯特罗斯） | 2011 优先权 / 2012 PCT 申请 / 2017 CN 授权 | Q-Tagg 推进控制系统 / DEGO 柴油机调速器（官方宣称以自动推进控制降低油耗） | 推进-横摇联合控制/节能：承认主机调速器会经螺旋桨转速对横倾角变化做出响应、进而放大参数横摇；通过改造调速器对横倾的响应抑制临界横摇起因，同时减少推进发动机燃料损耗与维护 |

#### 要点观察

- **三大 RRS 谱系的专利形态差异明显**：美国海军谱系（US4380206A 族 3 件，1981–1983）把"海况自适应"做成了统计变增益电路——增益随近期横摇统计量自适应、以舵速率饱和为硬约束，是 RRS 领域最早的"海况自适应"专利化表达；荷兰谱系（EP0129287B1，VRH/Imtech，1983–1987）把核心放在"执行器速率受限下的自适应增益"，对应 van der Klugt 1987 年代尔夫特博士论文与 van Amerongen 等的自适应 RRS 算法（MRAC/LQG 学派），并经 Imtech 产品化为 ASSA；德国 B+V 谱系（US4919064A 族，1983–1990）专利的是液压执行侧（高速舵机），控制算法专利归属荷兰/Imtech 一侧——商用 RRS 的"专利护城河"正是 BVI 舵机专利 + Imtech 算法专利的组合，Becker/BVI/Imtech 三方联合 RRS 宣传册对此有明确时间线记载（1984 年 BVI"用舵稳定船舶的方法"专利、1985 年 Imtech"控制算法核心部分"专利）。
- **丹麦谱系（Blanke/DTU）未检索到可核实的专利**：Blanke 团队的 RRS 成果以论文形式公开（Blanke, Haals & Andreasen, "Rudder roll damping autopilot experience in Denmark", CAMS'89；Blanke & Christensen, SCSS 1993），未见对应公开专利条目；本轮检索未找到 Blanke 本人或丹麦厂商名下的 RRS 专利，该谱系呈"学术公开、无专利"形态（宁缺毋滥，不列入清单）。
- **商用 RRS 的自适应宣称集中在海军市场**：Imtech ASSA 在荷兰 M 级护卫舰海试中"无需调参（no tuning required）"、"在恶劣天气下完全可用、能很好适应航速与航向变化"、减摇率最高约 60%（Becker 联合宣传册引荷兰皇家海军报告 SB8749），是"自适应 RRS"最明确的商用产品-功能证据链。
- **非舵类联合控制的边界样本**：Q-Tagg 的 CN103813958B/WO2013039445 用主机调速器抑制参数横摇并节油，属于"推进-运动联合控制"，与舵减摇同属"复用既有执行器减摇"思路，且直接面向商船节能场景，对 25 kn 游艇项目的"操舵-减摇-节能一体化"设计有参考价值。
- **舵减摇与游艇的相关性提示**：上述专利几乎全部面向军舰/商船（RRS 要求 10–20°/s 以上舵速与快速舵机）；25 kn 游艇尺度的 RRS 未见可核实的专门专利，游艇市场减摇专利集中于减摇鳍/陀螺（不属于本专利域）。

#### 产品-功能对照（厂商官方宣称）

- **Becker Marine Systems / B+V Industrietechnik / Imtech Marine & Offshore 联合 RRS 方案**（Becker 官网宣传册，已核实）：RRS 发展时间线明确列出"1984 B+V Industrietechnik 用舵稳定船舶方法专利""1985 Imtech 控制算法核心部分专利""1985 SIMPLEX-COMPACT® RRS 用 Rotary Vane 舵机设计投产""1986 RRS 控制算法在 MARIN 验证""1988–1993 装 8 艘荷兰 M 级护卫舰""1990 ASSA 海试无需调参""1999–2002 装德国 F124 与荷兰 LCF""2004 入选德国 K130 轻护舰"；宣称 M 级海试"测得最高约 60% 减摇，ASSA 在恶劣天气下完全可用、能很好适应航速与航向变化，横摇-操舵下航速不下降"。
- **SKF（Blohm+Voss Industries 后续权利人）**（SKF Evolution 官方文章，已核实）：SIMPLEX-COMPACT 产品线"包括伸缩/非伸缩减摇鳍与舵减摇系统（rudder roll stabilization systems）"，证实 B+V 专利谱系延续为在售商用 RRS 功能。
- **Imtech ASSA**（同上 Becker 宣传册）：Adaptive Steering and Stabilising Autopilot "将航向保持与减摇控制信号合成为一个复杂控制系统，舵与舵机同时完成操舵与稳定两项功能"。
- **Q-Tagg**（官网产品页，已核实）：定位为"marine green tech company，通过自动化推进控制系统降低船舶油耗"，DEGO 为可用于船用推进/发电的柴油机调速器——与 CN103813958B 所述"经调速器抑制横摇并降油耗"的技术路线一致。

#### 来源链接

- [US4380206A Ship roll stabilization system（美国海军，Baitis & Woolaver）— Google Patents](https://patents.google.com/patent/US4380206A/en)
- [US4388889A Electrical actuator for ship roll stabilization（美国海军）— FreePatentsOnline](https://www.freepatentsonline.com/4388889.html)
- [US4398486A Mechanical actuation device for ship roll stabilization（美国海军）— FreePatentsOnline](https://www.freepatentsonline.com/4398486.html)
- [EP0129287B1 Control System（Van Rietschoten & Houwens，van der Klugt & van Amerongen）— EPO publication server 授权文本 PDF](https://data.epo.org/publication-server/rest/v1.2/patents/EP0129287NWB1/document.pdf)
- [US4919064A Hydraulic system for ship rudder roll stabilization and steering（Blohm & Voss；含 DE/EP/JP/KR 同族列表）— Google Patents](https://patents.google.com/patent/US4919064A/en)
- [US20100147204A1 / US8567331B2 Rudder roll stabilization by nonlinear dynamic compensation（University of Wyoming, O'Brien）— Google Patents](https://patents.google.com/patent/US20100147204A1/en)
- [CN103813958B 用于防止和衰减船的横摇的方法（Q-TAGG）— 授权公告文本 PDF（patentimages）](https://patentimages.storage.googleapis.com/ec/ac/d9/6183a42812ada9/CN103813958B.pdf)
- [Becker Marine Systems：RUDDER ROLL STABILIZATION 联合宣传册（BVI/Imtech/Becker，含专利时间线与 ASSA 海试宣称）](https://login.becker-marine-systems.com/05_downloads/zzpdf/product_pdf/rrs_imagefolder.pdf)
- [Job van Amerongen 个人主页著作/专利清单（EP 申请 84200833.6，并称在美国、日本同时申请）](https://vanamerongen.org/?main=publications)
- [SKF Evolution：Building a global marine business（SIMPLEX-COMPACT 含 RRS 系统）](https://evolution.skf.com/us/building-a-global-marine-business/)
- [Q-Tagg 官网产品页（DEGO 调速器/推进控制，节油宣称）](https://qtagg.com/products/)
- [van der Klugt, "Rudder Roll Stabilization in Hr.Ms. Karel Doorman"（参考文献第 13 条即 European Patent no. 0129287）— IMarEST](https://library.imarest.org/record/8529/files/v33b2p12a.pdf)
- [Blanke, Haals & Andreasen, "Rudder Roll Damping Autopilot Experience in Denmark"（CAMS'89，丹麦谱系学术公开）— Aalborg University VBN](https://vbn.aau.dk/ws/files/72604842/P93_4026_v11c.pdf)

### 节能/生态操舵与 AI 操舵专利

本节覆盖"节能/生态操舵（ECO autopilot）"与"AI/机器学习操舵"两个方向的已公开专利，申请人横跨日、中、欧（含一件美国授权的早期经典专利）。所有条目均已在 Google Patents 逐页打开核实（标题、申请人、日期、法律事件以页面信息为准；个别日本授权文本的著录项取自 Google Patents 检索元数据并与全文内容交叉确认）。需要说明：检索中未找到 Anschütz（Raytheon Anschütz）针对 ECOnomy 模式的授权专利（其公开专利仅见 1970 年代早期电气自动舵），也未找到 Yokogawa（横河）的船舶节能自动舵专利——横河并不生产商船自动舵，日本节能自动舵的代表厂商为东京计器（Tokyo Keiki）。因此这两家以"产品-功能对照"形式呈现。

#### 专利清单

| 公开号/授权号 | 名称 | 申请人 | 申请/公开年份 | 对应商用产品 | 对应算法/主题 |
|---|---|---|---|---|---|
| US3505577A | Adaptive control for ship steering whereby system is less sensitive in rough sea（海况恶劣时降低灵敏度的船舶操舵自适应控制） | Tokyo Keiki Co Ltd（东京计器，发明人 Yoichi Hirokawa） | 1967 申请 / 1970 授权（已过期失效） | Tokyo Keiki PR 系列自动舵的技术源头；现行 PR-9000 的 ACE（Advanced Control for Ecology）功能与其思想一脉相承（见"产品-功能对照"） | 海况自适应/节能：按单位时间操舵次数自动调节操舵响应灵敏度，大浪中降低操舵频度——当代 ECO 模式的鼻祖方案 |
| JP6513677B2（中国同族 CN106660613B） | 船舶特性推定装置、及び自動操舵装置（船舶特性推定装置及自动操舵装置） | 古野電気株式会社（Furuno Electric） | 2015 申请 / 已授权（B2） | 未见对应产品（古野面向渔船/游艇的 NavPilot 系列自动舵未公开宣称采用该方案） | 海况自适应：在线估计表层流速度矢量与风力特性（风压差对船速的影响），将外扰估计前馈引入自动操舵，实现"海況に応じた素早い操舵" |
| JP2019196103A | 自動操舵装置、自動操舵方法および自動操舵プログラム（自动操舵装置、方法及程序） | 古野電気株式会社 | 2018 申请 / 2019 公开 | 未见对应产品 | 节能：自动生成使船舶燃費与乘坐舒适性更优的旋回航路（转向轨迹优化），由自动舵跟踪执行 |
| CN113031614B | 一种远洋船舶航向控制复合优化节油方法 | 上海海事大学 | 2021 申请 / 2022 授权 | 未见对应产品 | 节能航向控制：针对自动舵频繁操舵导致"S 形"航迹、增加阻力与油耗的问题，做航向保持的复合优化节油控制 |
| JP7804271B2（A 公开号 JP2023040453A） | 操縦パターン特定装置、制御装置、操縦パターン特定方法及びプログラム（操纵模式确定装置、控制装置、方法及程序） | 三菱造船株式会社 | 2021 申请 / 已授权（B2） | 未见对应产品（属三菱造船自动操船/自主航行研发线） | RL（深度强化学习）：以与实船操纵特性等价的虚拟船离线训练 DRL 模型，用于入出港航路跟踪，输出推进器/侧推/舵角的自动操纵模式 |
| CN114879671B | 一种基于强化学习 MPC 的无人艇轨迹跟踪控制方法 | 哈尔滨工程大学 | 2022 申请 / 已授权（B） | 未见对应产品（无人艇/USV） | RL + MPC：强化学习与模型预测控制结合的轨迹跟踪控制 |
| WO2024248067A1 | 航海支援システム（Navigation assistance system，航海支援系统） | 株式会社商船三井（Mitsui O.S.K. Lines, MOL） | 2024 申请 / 2024 公开 | 商船三井自动运航（自主航行）研发项目（未指向具体商品化产品） | AI 操船决策：内置规则推定逻辑，根据本船数据、目标物数据与周围态势生成局面认知数据，推定应适用的操船规则（COLREG 类避碰决策支持，属 AI 操舵的上层决策环节而非控制律本身） |

#### 要点观察

- **ECO 操舵的专利谱系很短、产品宣称远大于专利布局**。节能自动舵的核心卖点（降低舵机活动频度、海况自适应降灵敏度）在专利文献中可追溯到 1970 年东京计器的 US3505577A，此后厂商（Anschütz、Tokyo Keiki）多以商业秘密/算法细节不公开的方式实现 ECO 模式，公开授权专利稀缺。Anschütz 公司名下可检索到的自动舵专利均为 1968–1970 年代的电气自动舵（US3517285A、US3660743A），未见针对 ECOnomy 模式或 Toe Angle 功能的公开专利。
- **"YOKOGAWA 节能自动舵"未获证实**。横河电机（Yokogawa）无商船自动舵产品线，检索未发现其名下节能操舵专利；日本市场节能自动舵代表为东京计器 PR-9000 的 ACE 功能。若调研文档其他章节提到"YOKOGAWA 节能"，建议修正为 Tokyo Keiki。
- **AI/RL 操舵专利 2021 年后集中出现，且以"决策/特定场景"为主**：三菱造船的深度强化学习入出港跟踪（JP7804271B2，明确记载用虚拟船离线训练 DRL）、商船三井的操船规则推定（WO2024248067A1）都避开了"全程航向保持控制律"这一经典自动舵领域，聚焦靠离泊、避碰决策等高价值场景；中国高校（哈工程）则出现 RL+MPC 混合方案（CN114879671B）。
- **古野電気的路线是"外扰在线估计 + 前馈操舵"**（JP6513677B2）与"燃費最优转向轨迹"（JP2019196103A），属于海况自适应与节能的工程化中间路线，未见其采用神经网络/RL。
- **法律状态提示**：US3505577A 早已过期（可自由参考）；其余条目以 Google Patents 页面标注为准（授权有效或公开审中），非法律意见。

#### 产品-功能对照（厂商官方/公开宣称）

- **Anschütz NautoPilot 5000 NX — ECOnomy 模式**：官方宣称 ECO 模式下自动舵持续监测船舶实际艏摇运动（随载况与海况变化），滤除对称性艏摇和因船舶惯性无法修正的运动而不予纠偏，从而减少舵机活动实现持续节油；另有 Course Control（对地航迹控制、自动补偿流压差）与双舵船 Toe Angle（双舵错角）功能。经销商与行业媒体资料宣称 ECO 模式节油可达约 5%。对应主题：海况自适应 + 节能（波浪滤波思想的工程实现）。
- **Tokyo Keiki PR-9000 — ACE（Advanced Control for Ecology）**：官方样本宣称 ACE 功能通过最小化不必要的舵运动提供显著能效；其天气调节采用双增益法（dual gain）替代传统死区法，在提高航向保持性的同时减少能量损失。对应主题：海况自适应 + 节能，与 US3505577A 的技术路线一脉相承。
- **HD Hyundai Avikus HiNAS 2.0 / HiNAS Control**：官方宣称 AI 识别天气、浪高与周边船舶后实时控制操舵，2022 年 PRISM COURAGE 跨洋自主航行试验报告节油约 7%（温室气体减排约 5%），后续宣称 VLOC 船型节油超 10%；HiNAS Control 已获 DNV 型式认可。对应主题：AI/机器学习操舵（强化学习自动靠泊为其公开宣称的技术之一）。未检索到 Avikus/HD 现代名下可核实的对应授权专利，故仅列产品宣称。
- **商船三井（MOL）自动运航项目**：WO2024248067A1 对应其自动运航研发中的"规则推定"环节；MOL 公开发布中多次提到与古野、东京计器等合作的自主航行实证，未见具体商品化操舵产品。

#### 检索说明与困难

- Google Patents 正文检索接口在本轮检索中多次触发限流（HTTP 503），日文专利的著录项（申请人、日期）部分取自 Google Patents 检索元数据并经全文内容交叉确认；Espacenet 页面为 JS 渲染无法直接抓取，未作为核实来源。
- 厂商节能功能（Anschütz ECOnomy、Tokyo Keiki ACE）在专利数据库中无对应授权专利可核实，已按"宁缺毋滥"原则不虚构专利号，仅以产品宣称对照呈现。
- 韩国（三星重工、HD 现代/Avikus）RL 操舵专利检索未获可核实条目；搜索结果集中于新闻报道与产品宣称，未收录。

#### 来源链接

专利页（Google Patents）：

- [US3505577A Adaptive control for ship steering whereby system is less sensitive in rough sea（Tokyo Keiki）](https://patents.google.com/patent/US3505577A/en)
- [JP6513677B2 船舶特性推定装置、及び自動操舵装置（古野電気）](https://patents.google.com/patent/JP6513677B2/en)
- [JP2019196103A 自動操舵装置、自動操舵方法および自動操舵プログラム（古野電気）](https://patents.google.com/patent/JP2019196103A/en)
- [CN113031614B 远洋船舶航向控制复合优化节油方法（上海海事大学）](https://patents.google.com/patent/CN113031614B/en)
- [JP7804271B2 操縦パターン特定装置、制御装置、操縦パターン特定方法及びプログラム（三菱造船）](https://patents.google.com/patent/JP7804271B2/en)
- [CN114879671B 基于强化学习MPC的无人艇轨迹跟踪控制方法（哈尔滨工程大学）](https://patents.google.com/patent/CN114879671B/en)
- [WO2024248067A1 航海支援システム（商船三井）](https://patents.google.com/patent/WO2024248067A1/en)

产品宣称：

- [Anschütz 官方：NautoPilot 5000 NX（ECOnomy mode / Toe Angle）](https://kiosk.anschuetz.com/gyro-steering-dive-in/gyro-steering/autopilots/nautopilot-5000-nx)
- [MarineLink：Raytheon Anschütz Launches NautoPilot 5000NX（ECO 模式原理描述）](https://www.marinelink.com/news/raytheon-anschtz-launches-nautopilot-nx-502199)
- [SOLAS Index：NautoPilot 5000 NX（ECOnomy 节油约 5% 宣称）](https://solasindex.com/products/raytheon-anschuetz/nautopilot-5000-nx/)
- [Tokyo Keiki PR-9000 样本（ACE 功能，NauticExpo 目录页）](https://pdf.nauticexpo.com/pdf/tokio-keiki/pr-9000/30608-88165.html)
- [Mackay：Tokyo Keiki A.C.E. Route Tracking Course-Control for PR-9000（节油 1–1.5% 宣称）](https://www.mackaycomm.com/products/navigation/autopilot/tokyo-keiki-a-c-e-route-track-control-for-pr-6000-autopilots-copy/)
- [HD Hyundai：Avikus HiNAS 2.0 跨洋自主航行（节油约 7% 宣称）](https://esg.hd.com/en/news/601)
- [Korea JoongAng Daily：Avikus HiNAS Control 获 DNV 型式认可](https://www.koreajoongangdaily.com/business/hd-hyundais-avikus-wins-global-certification-for-autonomous-navigation/12567791)

### 中国专利与厂商

本节覆盖中国申请人（高校、研究所、厂商）在船舶自动舵/操舵自适应控制方向的已公开专利。所有条目均已在 Google Patents 逐页核实（公开号、申请人、日期、法律状态以页面信息为准）。

#### 专利清单

| 公开号/授权号 | 名称 | 申请人 | 申请/公开年份 | 对应商用产品 | 对应算法/主题 |
|---|---|---|---|---|---|
| CN101281409A（授权 CN101281409B，即 ZL200810106195.2） | 自动操舵仪 | 北京海兰信数据科技股份有限公司 | 2008 申请 / 2008 公开，2012 授权 | 海兰信 SCS 自动操舵仪（见下方"产品-功能对照"） | 系统架构（双 CAN 冗余网络自动舵），非控制算法本身 |
| CN100589055C（申请号 200810064256.3） | 基于递归模糊神经网络的水下潜器空间变结构控制方法 | 哈尔滨工程大学 | 2008 申请 / 2010 授权（已因未缴年费失效） | 未见对应产品 | NN（递归模糊神经网络 RFNN）+ 滑模变结构；RFNN 实时调整滑模增益、抑制抖振 |
| CN101615000A（申请号 200910012146） | 航海模拟器用鲁棒自适应自动舵控制算法 | 大连海事大学（张显库团队） | 2009 申请 / 2009 公开 | 未见对应产品（应用于航海模拟器） | 鲁棒自适应 + NN：闭环增益成形（简化 H∞）鲁棒控制 + 神经网络在线逆模型训练；性能指标含舵角/转首速率加权（节能导向） |
| CN107918393A（授权 CN107918393B） | 基于深度置信网络的船舶自动舵 | 江汉大学 | 2017 申请 / 2018 公开，2019 授权（已失效） | 未见对应产品 | NN/深度学习：DBN（RBM 逐层训练 + BP 微调）根据风、浪、流环境信息直接输出舵角 |
| CN110703605A（授权 CN110703605B） | 一种面向智能船舶自动舵系统的自适应模糊最优控制方法及系统 | 大连海事大学（李铁山团队） | 2019 申请 / 2020 公开，2022 授权 | 未见对应产品（以"育鲲"轮模型仿真验证） | 模糊 + 自适应：模糊状态观测器解决航向角速率不可测的输出反馈问题，降低能耗与舵机磨损 |
| CN112947445A（授权 CN112947445B） | 一种采用冗余通信的分布式自适应船舶航迹保持系统 | 大连海事大学（张国庆、张显库等） | 2021 申请 / 2021 公开，2024 授权 | 未见对应产品（经"育鲲"轮半实物仿真与 INAC-II 无人艇实船试验验证） | 自适应航迹舵系统：分布式航迹/航向保持、双 CAN 冗余通信、鲁棒自适应航迹保持算法 |
| CN113253720B（申请号 202110411253.8） | 一种船舶航向控制方法及系统 | 上海中船船舶设计技术国家工程研究中心有限公司（中船集团） | 2021 申请 / 2023 授权 | 未见对应产品 | 逆最优控制：针对参数未知/有扰动场景设计自适应逆最优控制器，目标为航向精度与操舵能耗综合最优（节能） |
| CN116699995A（申请号 202310731003.1） | 一种船舶自适应舵航迹保持积分最优控制方法及系统 | 哈尔滨工程大学 | 2023 申请 / 2023 公开（审中） | 未见对应产品 | LQG 最优 + 波浪滤波：GESO（高斯扩张状态观测器）滤除海浪与传感器高频干扰 + ILQR 积分最优控制，减少操舵次数、降低能耗 |
| CN118605188B（申请号 202411082539.6） | 参数自适应的水下航行器航向最优控制方法、系统及产品 | 中国船舶集团第七〇七研究所（九江分部） | 2024 申请 / 2024 授权 | 未见对应产品 | NN + 最优控制（LQR）：卷积神经网络根据航速自适应生成 LQR 的 Q/R 权重矩阵，免人工整定 |

#### 要点观察

- **高校主导算法专利**：大连海事大学（张显库、李铁山两个团队）与哈尔滨工程大学是国内船舶自动舵自适应/智能控制专利最集中的申请人，覆盖鲁棒自适应、模糊、神经网络、滑模、LQG 等方向，与本文档前述算法章节基本一一对应，但绝大多数停留在仿真与教学试验船验证阶段，未见直接商用产品。
- **厂商专利偏系统架构而非算法**：海兰信的自动操舵仪发明专利（CN101281409B 及同族 ZL200810106188.2）保护的是双 CAN 网络冗余架构与主控热备份，而非自适应控制律；其上市文件宣称 SCS 操舵仪采用"ARM+DSP 双核并行运算"以支持"大运算量的智能航迹控制算法"（见下）。
- **中船集团体系**：检索到的相关专利来自上海中船船舶设计技术国家工程研究中心（逆最优航向控制，节能导向）和 707 所九江分部（CNN 自适应 LQR 权重的水下航行器航向控制）；未检索到 716 所、系统工程研究院在自动舵自适应控制方向的公开专利。
- **新诺北斗、武汉理工大学、上海船舶运输科学研究所**：在本轮检索中未找到与"自适应操舵/航向控制算法"直接对应且可核实的公开专利（武汉理工大学的船舶控制专利集中于无人艇 MPC 航速/路径跟踪方向，与本章主题相关性不足，未收录）。
- **法律状态提示**：CN100589055C、CN101281409B、CN107918393B 已因未缴年费失效（技术方案进入公有领域，可自由参考）；其余为授权有效或审中状态（以 Google Patents 页面标注为准，非法律意见）。

#### 产品-功能对照（厂商官方宣称）

- **海兰信 SCS 自动操舵仪**：公司创业板上市文件披露，SCS 操舵仪核心技术为"ARM 和 DSP 双核控制并行运算技术"（自主研发、已获发明专利），宣称"目前国内国际自动舵普遍采用单核系统……无法应用大运算量的智能航迹控制算法"，并称其为"国内首家申请该技术自动舵的中国船级社认证企业"。上市文件列明的两项"自动操舵仪"发明专利（ZL200810106195.2、ZL200810106188.2，均 2008-05-09 申请）与 CN101281409B 对应。注意：官方宣称侧重硬件算力与系统架构，未公开具体自适应控制算法细节。

#### 来源链接

- [CN101281409A 自动操舵仪（海兰信）— Google Patents](https://patents.google.com/patent/CN101281409A/zh)
- [CN100589055C 基于递归模糊神经网络的水下潜器空间变结构控制方法（哈尔滨工程大学）— Google Patents](https://patents.google.com/patent/CN100589055C/zh)
- [CN101615000A 航海模拟器用鲁棒自适应自动舵控制算法（大连海事大学）— Google Patents](https://patents.google.com/patent/CN101615000A/zh)
- [CN107918393A 基于深度置信网络的船舶自动舵（江汉大学）— Google Patents](https://patents.google.com/patent/CN107918393A/zh)
- [CN110703605A 面向智能船舶自动舵系统的自适应模糊最优控制方法及系统（大连海事大学）— Google Patents](https://patents.google.com/patent/CN110703605A/zh)
- [CN112947445A 采用冗余通信的分布式自适应船舶航迹保持系统（大连海事大学）— Google Patents](https://patents.google.com/patent/CN112947445A/zh)
- [CN113253720B 一种船舶航向控制方法及系统（上海中船船舶设计技术国家工程研究中心）— Google Patents](https://patents.google.com/patent/CN113253720B/zh)
- [CN116699995A 船舶自适应舵航迹保持积分最优控制方法及系统（哈尔滨工程大学）— Google Patents](https://patents.google.com/patent/CN116699995A/zh)
- [CN118605188B 参数自适应的水下航行器航向最优控制方法、系统及产品（中船 707 所九江分部）— Google Patents](https://patents.google.com/patent/CN118605188B/zh)
- [北京海兰信首次公开发行股票并在创业板上市招股说明书（SCS 操舵仪核心技术与专利清单）](http://pdf.dfcfw.com/pdf/H2_AN201202290004530558_1.pdf)
- [海兰信创业板公开发行可转换公司债券募集说明书（专利号清单：ZL200810106195.2 等）](http://reportdocs.static.szse.cn/UpFiles/rasinfodisc/RAS_000172E2767CB33FE262D08756BDCD3F.pdf)
