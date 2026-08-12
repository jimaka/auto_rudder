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
