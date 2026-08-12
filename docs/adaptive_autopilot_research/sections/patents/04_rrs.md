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
