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
