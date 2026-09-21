#!/usr/bin/env python3
"""Generate curated architecture/sequence figures as editable SVG + PNG.

These figures are manually curated from source, not auto-generated execution
traces. Re-review node order and conditions whenever algorithms change.
"""
from pathlib import Path
import os

os.environ.setdefault("MPLCONFIGDIR", "/tmp/auto-rudder-architecture-mpl")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

OUT = Path(__file__).resolve().parent / "figures"
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "svg.fonttype": "none", "axes.unicode_minus": False})
PALETTE = {"core": ("#e6f2fb", "#2776ad"), "state": ("#e5f5ee", "#258566"),
           "gate": ("#fff3d8", "#b68019"), "external": ("#f3edf9", "#8860a5"),
           "warn": ("#fcecea", "#b35d53")}
MANIFEST = []


def diagram(name, title, subtitle, nodes, edges, footnote=""):
    """Nodes: (id, col, row, title, detail, palette). Edges: (a,b,label,dashed)."""
    max_col = max(n[1] for n in nodes)
    max_row = max(n[2] for n in nodes)
    cols = max_col + 1
    figw, figh = max(8, cols * 3.6), (max_row + 1) * 1.65 + 1.75
    fig, ax = plt.subplots(figsize=(figw, figh))
    fig.patch.set_facecolor("#f5f8fb")
    ax.set_facecolor("#f5f8fb")
    ax.set_xlim(-0.65, cols * 3.6 - .15)
    ax.set_ylim(-1.15, (max_row + 1) * 1.65 + .35)
    ax.axis("off")
    width, height = 3.0, .98
    positions = {n[0]: (n[1] * 3.6 + 1.5, (max_row - n[2]) * 1.65 + .65) for n in nodes}
    for a, b, label, dashed in edges:
        x1, y1 = positions[a]
        x2, y2 = positions[b]
        if abs(y1 - y2) < .1:
            side = 1 if x2 > x1 else -1
            start, end = (x1 + side * width / 2, y1), (x2 - side * width / 2, y2)
        elif abs(x1 - x2) < .1:
            side = 1 if y2 > y1 else -1
            start, end = (x1, y1 + side * height / 2), (x2, y2 - side * height / 2)
        else:
            side = 1 if y2 > y1 else -1
            start, end = (x1, y1 + side * height / 2), (x2, y2 - side * height / 2)
        arrow = FancyArrowPatch(start, end, arrowstyle="-|>", mutation_scale=13,
                                linewidth=1.4, color="#647d90", linestyle="--" if dashed else "-", zorder=1)
        ax.add_patch(arrow)
        if label:
            ax.text((start[0]+end[0])/2, (start[1]+end[1])/2+.07, label,
                    ha="center", va="center", fontsize=8.6, color="#344e62",
                    bbox=dict(facecolor="#f5f8fb", edgecolor="none", pad=1), zorder=4)
    for uid, col, row, heading, detail, color in nodes:
        x, y = positions[uid]
        bg, border = PALETTE[color]
        ax.add_patch(FancyBboxPatch((x-width/2, y-height/2), width, height,
                                    boxstyle="round,pad=0.025,rounding_size=.09",
                                    facecolor=bg, edgecolor=border, linewidth=1.3, zorder=2))
        ax.text(x, y+.18, heading, fontsize=10.4, fontweight="bold", ha="center", va="center", color="#17334d", zorder=3)
        ax.text(x, y-.20, detail, fontsize=8.7, ha="center", va="center", color="#40596b", linespacing=1.45, zorder=3)
    fig.text(.045, .962, title, fontsize=19, fontweight="bold", color="#14334d", va="top")
    fig.text(.045, .915, subtitle, fontsize=10, color="#536c7d", va="top")
    fig.text(.045, .028, footnote or "实线：源码调用/执行顺序或数据路径（见标题）   虚线：条件、回调或外部集成边界", fontsize=9, color="#617889")
    fig.subplots_adjust(left=.03, right=.97, top=.86, bottom=.09)
    OUT.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT / f"{name}.png", dpi=155, facecolor=fig.get_facecolor())
    fig.savefig(OUT / f"{name}.svg", facecolor=fig.get_facecolor())
    plt.close(fig)
    MANIFEST.append((name, title))


def n(uid, x, y, title, detail="", kind="core"):
    return (uid, x, y, title, detail, kind)


def e(a, b, label="", dashed=False):
    return (a, b, label, dashed)


def main():
    diagram("01_overview", "01 / 项目总体架构", "以 C++ 头文件控制库为中心；不同语言入口之间存在进程、文件与协议边界。", [
        n("test",0,0,"GoogleTest / CTest","9 个单测目标 + 6 个仿真冒烟", "external"),
        n("sim",1,0,"ar_sim","场景设置 → 控制循环 → 指标/CSV"),
        n("ui",2,0,"debug_ui.html","独立 JS 演示算法 + DOM/Canvas", "external"),
        n("debug",0,1,"ar_tuner_debug","CLI → TunerDebugApi → AutoTuner"),
        n("core",1,1,"AutopilotCore","模式管理 / 航向闭环 / 辨识试验", "state"),
        n("ws",2,1,"WebSocket 对端","仓库未提供 ECU 服务端/BSP", "warn"),
        n("tools",0,2,"Python 调试工具","子进程标准输入/输出", "external"),
        n("algo",1,2,"控制模块","PD · 调度 · RLS · ESC · 机动序列"),
        n("store",2,2,"ParamStore","二进制序列化 + CRC + 外部 I/O"),
        n("plot",0,3,"绘图 / 动画 / 研究文档","Python + Shell → PNG/MP4/PDF", "external"),
        n("plant",1,3,"NomotoPlant / Disturbances","仿真对象与传感器噪声", "state"),
        n("bsp",2,3,"Flash / EEPROM / 硬件","仅接口边界；尚非现成固件", "warn"),
    ], [e("test","core"),e("sim","core"),e("ui","ws","JSON",True),e("tools","debug","stdio"),e("debug","algo","调参旁路"),e("core","algo"),e("sim","plant","闭环对象",True),e("store","bsp","回调",True),e("plant","plot","CSV",True)])

    diagram("02_core_modes", "02 / AutopilotCore::step 模式分派", "入口是调用方提供的 SensorInput 与 dt；step() 本身不采集传感器，也不驱动外设。", [
        n("in",1.5,0,"step(s, dt)","读取当前 mode_", "state"),
        n("manual",0,1,"MANUAL","cmd = s.rudderDeg → 统一返回"),
        n("heading",1,1,"AUTO_HEADING","headingControlStep → 统一返回"),
        n("mv",2,1,"AUTO_MANEUVER","sequencer.update(...)"),
        n("hold",3,1,"HOLD","cmd = holdRudder_ → 统一返回"),
        n("done",2,2,"序列是否完成？","完成：试验收尾 → 切航向模式", "gate"),
        n("active",3,3,"仍在机动","试验期更新 RLS；按航段执行 PD/限幅"),
        n("finish",1,3,"完成后的同一个周期","finalizeIdentTrial（若需）\nheadingControlStep"),
        n("out",1.5,4,"统一返回","lastRudderCmd_ = cmd；return cmd", "state"),
    ], [e("in","manual"),e("in","heading"),e("in","mv"),e("in","hold"),e("mv","done"),e("done","active","否"),e("done","finish","是"),e("finish","out"),e("active","out")],
    "注意：startManeuver/startIdentifyTrial 和序列结束直接改 mode_，并不等同于调用 setMode()。")

    diagram("03_heading_sequence", "03 / 一个航向控制周期的真实顺序", "按 ①→⑦ 阅读；状态保存在同一个 AutopilotCore 对象内，下一周期继续使用。", [
        n("start",0,0,"① headingControlStep","pendingBaseline_ 时 initBaseline", "state"),
        n("sched",0,1,"② GainSchedule::query","speedKn + seaState → 名义 Kp/Kd"),
        n("rls",0,2,"③ NomotoIdentifier::update","r、实际舵角、dt → 辨识状态"),
        n("snap",1,2,"④ idSnapshot","excited / valid / K / T"),
        n("tune",1,1,"⑤ AutoTuner::tick","名义增益 + 传感器 + 辨识快照"),
        n("pd",1,0,"⑥ 配置 PD","setDt(dt) → setGains(Kp, Kd)"),
        n("out",2,0,"⑦ PdController::update","航向参考与测量 → 舵角指令", "state"),
        n("feedback",2,2,"下一周期再反馈","执行器/模型更新后\n返回实际舵角与航向", "external"),
    ], [e("start","sched"),e("sched","rls"),e("rls","snap"),e("snap","tune"),e("tune","pd"),e("pd","out"),e("out","feedback",dashed=True)],
    "MANUAL/HOLD 不执行此链；未完成的 AUTO_MANEUVER 不调用 tuner.tick()。")

    diagram("04_pd_schedule", "04 / PD、增益调度与角度工具", "左列是 PD 更新顺序；中、右列是独立辅助路径，不能把调度表查询画成 PD 内部调用。", [
        n("pd",0,0,"PdController::update","angleDiffDeg(ref, heading)"),
        n("dead",0,1,"死区判断","小误差 → saturate(0) 并返回", "gate"),
        n("law",0,2,"PD 控制律","误差差分 → 微分低通 → P+D"),
        n("limit",0,3,"输出整形","幅值限制 → 速率限制 → quantize"),
        n("state",0,4,"保存内部状态","prevErr / prevD / prevOut", "state"),
        n("sat",1,2,"saturate(raw)","独立舵角模式/死区复用限幅"),
        n("q",1,3,"quantize(x)","std::round，按舵角分辨率量化"),
        n("schedule",2,0,"GainSchedule::query","海况归一化 + 当前表逐项扫描"),
        n("inter",2,1,"按速度中点插值","同海况相邻锚点；边界用端点"),
        n("angle",2,3,"angleDiffDeg / Rad","normalizeAngleDeg / Rad"),
        n("range",2,4,"环绕角差","跨 0°/360° 或 ±π 计算误差", "state"),
    ], [e("pd","dead"),e("dead","law","非死区"),e("dead","sat","死区",True),e("law","limit"),e("limit","state"),e("sat","q"),e("limit","q","量化"),e("schedule","inter"),e("angle","range")])

    diagram("05_rls", "05 / NomotoIdentifier::update", "调用频率可以是控制周期；算法内部累计 elapsed_，达到约 1 s 才处理一拍。", [
        n("input",0,0,"输入检查","r / delta 有限、幅值、dt > 0", "gate"),
        n("beat",0,1,"累计时间","不足一拍 → return false", "gate"),
        n("excite",0,2,"计算 dr 与激励门","舵角足够，且角速度/变化率足够", "gate"),
        n("skip",1,2,"激励不足","更新必要历史状态；不做 RLS", "warn"),
        n("reg",0,3,"构造归一化回归量","Nomoto 关系 + 尺度归一化"),
        n("rls",1,3,"递推最小二乘","分母检查 → θ 更新 → P 更新"),
        n("result",2,3,"K() / T() / excited()","输出当前估计和最近激励状态", "state"),
        n("valid",2,2,"valid()","有限数 + K/T 物理范围", "gate"),
        n("core",2,1,"Core::idSnapshot","形成 AutoTuner 的输入快照"),
    ], [e("input","beat","通过"),e("beat","excite","达到节拍"),e("excite","skip","未过门"),e("excite","reg","过门"),e("reg","rls"),e("rls","result"),e("result","valid"),e("valid","core")],
    "valid() 不是统计收敛证明；辨识试验成功与长期 ESC 收敛是不同层次的判据。")

    diagram("06_tuner_tick", "06 / AutoTuner::tick 总流程", "先处理增益所有权和基线，再累计性能数据，最后按条件执行窗口更新和载波扰动。", [
        n("gate",0,0,"① 航速去激活门","更新 enabled_", "gate"),
        n("sched",0,1,"② 未建基线时跟随调度","curKp_ / curKd_ = sched 增益"),
        n("estimate",0,2,"③ 辨识估计处理","enabled 且 excited、valid\n→ applyEstimate_（漂移门）", "gate"),
        n("error",1,2,"④ 误差/参考稳定度","angleDiffDeg；累计参考稳定时间"),
        n("acc",1,1,"⑤ accumulate","误差、实际舵角、振荡检测资格"),
        n("window",1,0,"⑥ 窗口到期？","settleWindow_；内部再检查门控", "gate"),
        n("dither",2,0,"⑦ 已建立基线？","escDither；内部检查 enabled", "gate"),
        n("output",2,1,"返回 GainCommand","发送给 PD 的 Kp / Kd", "state"),
        n("owner",2,2,"三套值需要区分","调度名义值 / ESC 中心值\n实际载波扰动后的输出", "warn"),
    ], [e("gate","sched"),e("sched","estimate"),e("estimate","error"),e("error","acc"),e("acc","window"),e("window","dither"),e("dither","output")],
    "tick 的 refAge 使用输入 in.dt；ESC 相位/窗口/积分使用 Params::dt。文档不假定两者自动同步。")

    diagram("07_baseline", "07 / 建立基线、重基线与恢复", "applyEstimate_ 是普通在线辨识接管和显式试验接管的共同入口。", [
        n("tick",0,0,"tick → applyEstimate_","driftGate=true；forceRebase=false"),
        n("trial",2,0,"establishBaseline(snapshot)","有效且受激；强制重基线"),
        n("drift",1,1,"applyEstimate_","已有基线时检查 K/T 相对漂移", "gate"),
        n("pole",1,2,"polePlacement","K/T + ζ/ωn → 候选 Kp/Kd"),
        n("stable",1,3,"stabilityCheck","模型/增益/阻尼/频率/振荡边界", "gate"),
        n("commit",1,4,"setBaseline + 当前中心值","保存 baseK/baseT；标记已接管", "state"),
        n("rebase",2,4,"按条件 rebaseEsc","清梯度/滤波/缓冲；窗口重新计时"),
        n("reject",0,3,"拒绝本次候选","return false；不建立该基线", "warn"),
    ], [e("tick","drift"),e("trial","drift"),e("drift","pole","允许处理"),e("pole","stable","可计算"),e("pole","reject","失败",True),e("stable","reject","不通过",True),e("stable","commit","通过"),e("commit","rebase","已有基线/强制",True)])

    diagram("08_esc", "08 / ESC：逐样本通道与窗口通道", "本图为信号/状态依赖；控制周期内 accumulate 先于 escDither。", [
        n("acc",0,0,"accumulate(e, delta)","RingBuffer：|e|、|delta|、|Δdelta|"),
        n("hpf",0,1,"瞬时代价 c 与高通","enabled 时 HPF + sin/cos 解调累积"),
        n("osc",0,2,"OscillationDetector","仅在参考稳定资格满足时 update"),
        n("phase",1,0,"escDither(Kp, Kd)","sin/cos 乘性扰动；相位推进"),
        n("center",1,1,"ESC 中心增益","扰动输出不覆盖 curKp/curKd", "state"),
        n("settle",2,0,"settleWindow_","enabled + baselineEstablished", "gate"),
        n("step",2,1,"escStep → computeJ","解调归一化 → LPF → 梯度下降"),
        n("clamp",2,2,"clampUpdate","限制单次增量；候选非负化"),
        n("check",2,3,"稳定性检查与提交","通过：接收中心；失败：revert", "gate"),
        n("reset",1,3,"窗口状态清理","escStep 清缓冲/HPF/解调累积", "state"),
    ], [e("acc","hpf"),e("acc","osc","满足振荡资格",True),e("center","phase"),e("settle","step"),e("hpf","step","窗口统计"),e("step","clamp"),e("clamp","check"),e("step","reset","内部清理",True),e("check","center","下一窗口",True)],
    "revert 取回保存的基线；rebaseEsc 与 reset 的清理范围不同，均不能简单画成重新构造对象。")

    diagram("09_maneuver", "09 / 机动序列与辨识试验", "机动工厂先生成 vector<Leg>；ManeuverSequencer 只产出航段命令，实际舵角由 Core 决定。", [
        n("factory",0,0,"六类机动工厂","williamson / uTurn / zigzag\ncircles / cloverleaf / search"),
        n("set",0,1,"setLegs","清索引/时间/累计转角", "state"),
        n("update",0,2,"update(heading, rate, dt)","angleDiffDeg → 累计转角/时间"),
        n("trigger",1,2,"航段触发条件","HEADING_REACHED / TIME\nRATE / NONE", "gate"),
        n("return",2,2,"返回当前段命令","内部可能已推进到下一段", "state"),
        n("core",2,1,"AutopilotCore 分派","航向段 → PD；舵角段 → saturate"),
        n("trial",1,0,"startIdentifyTrial","identifier.reset → zigzag(false)\nRUNNING + AUTO_MANEUVER"),
        n("finish",2,0,"finalizeIdentTrial","idSnapshot → establishBaseline\nOK / FAILED"),
    ], [e("factory","set"),e("set","update"),e("update","trigger"),e("trigger","return"),e("return","core"),e("trial","set"),e("core","finish","序列 done 且试验中",True)],
    "末段 trigger=NONE 表示无限保持，不会自然 done；runManeuver 对末段保持另有完成判据。")

    diagram("10_storage", "10 / 参数保存与加载：实际调用顺序", "当前是可注入 I/O 的组件，不是已经接通 AutopilotCore 的启动恢复流程。", [
        n("save",0,0,"ParamStore::save(p)","可选 clock_ → timestamp；version++"),
        n("crc",0,1,"p.updateCrc","serialize → crc32"),
        n("bytes",0,2,"再次 serialize","显式小端字段；追加 CRC32"),
        n("write",0,3,"write_(data, size)","外部存储回调", "external"),
        n("load",1,0,"ParamStore::load(p)","read_ 读取 4096 字节缓冲"),
        n("decode",1,1,"p.deserializeFrom","readLEf64 / readLEu32"),
        n("readcrc",1,2,"读取尾部 CRC","检查可用长度；写入 p.crc"),
        n("verify",1,3,"p.verifyCrc","serialize → crc32 → 返回比较结果", "gate"),
        n("range",2,1,"p.validateRange","已实现；load 内未调用", "warn"),
        n("apply",2,3,"参数应用与硬件后端","由集成层补充，非现成连线", "warn"),
    ], [e("save","crc"),e("crc","bytes"),e("bytes","write"),e("load","decode","读取成功"),e("decode","readcrc","解析成功"),e("readcrc","verify"),e("range","apply","集成层可显式调用",True)],
    "不要按头部注释推断 load 已做范围校验；实际代码也未做双槽掉电保护或失败后的对象回滚。")

    diagram("11_sim", "11 / 仿真主入口与闭环数据", "ar_sim 的不同实验共用组件，但各自设置增益、激励和验收条件。", [
        n("main",0,0,"main(argc, argv)","parseDouble / parseInt / takeVal"),
        n("route",0,1,"场景分派","ident / trial / tune / esc\nvalidate / maneuver / step(all)"),
        n("setup",0,2,"Simulator::run* 初始化","plant.reset；模式/调度/调参配置"),
        n("sensor",1,2,"构造 SensorInput","plant 状态 + 可选传感器噪声"),
        n("core",1,1,"ap.step(s, dt)","得到舵角指令"),
        n("plant",1,0,"NomotoPlant::step","执行器约束 → r/psi 积分"),
        n("record",2,0,"采样 / 下一周期","记录 Sample；按场景检查结束", "state"),
        n("metrics",2,1,"指标与实验结果","computeMetrics / 辨识 / 收敛判据"),
        n("csv",2,2,"exportCsv / exportTuneCsv","文件交给 plot_* / animate_sim", "external"),
    ], [e("main","route"),e("route","setup"),e("setup","sensor"),e("sensor","core"),e("core","plant"),e("plant","record"),e("record","sensor","循环",True),e("record","metrics","结束"),e("metrics","csv")],
    "runIdent 实际用 AUTO_HEADING 的大参考阶跃形成激励，不能按旧注释画成 MANUAL 直接注舵。")

    diagram("12_debug", "12 / 调试接口、CLI 与监视器", "绿色表示调参状态；橙色路径允许直接改变内部状态，只适合测试/调试。", [
        n("accept",0,0,"tuner_debug_accept.py","Cli.cmd → 协议断言", "external"),
        n("monitor",1,0,"tuner_debug_monitor.py","轮询线程 / 控制台 / 演示线程", "external"),
        n("cli",1,1,"ar_tuner_debug::main","getline → 命令分支 → 输出一行"),
        n("api",1,2,"TunerDebugApi","公开包装器与 friend 访问"),
        n("read",0,3,"只读/计算接口","snapshot / currentJ / pole / check"),
        n("normal",1,3,"状态推进接口","tick / accumulateOnce / settleWindowOnce", "state"),
        n("force",2,3,"force* / rebase / revert","绕过正常准入的实验入口", "gate"),
        n("plot",2,0,"Monitor.draw","历史快照 → Matplotlib 曲线", "external"),
    ], [e("accept","cli","stdio",True),e("monitor","cli","加锁 stdio",True),e("monitor","plot","定时/动画回调",True),e("cli","api"),e("api","read"),e("api","normal"),e("api","force")],
    "监视器 demo 注入的是人工误差/舵角序列，不是 NomotoPlant 闭环；参数修改 setParams 会 reset。")

    diagram("13_ui", "13 / 网页调试界面：事件调用链", "所有蓝色函数均属于 debug_ui.html 的 JavaScript，不是对同名 C++ 函数的绑定。", [
        n("start",0,0,"开始仿真按钮 onclick","设置 simRunning；注册 setInterval"),
        n("tick",0,1,"controlStep","仿真时先 simStep，再按模式控制"),
        n("algo",0,2,"JS 控制与调参函数","schedQuery / idUpdate / pdUpdate\naccumulateJ / escStep / mvUpdate"),
        n("draw",0,3,"记录与显示","updateStatusUI → drawPlots\nsetupCanvas / drawSeries"),
        n("edit",1,0,"参数/机动/滑块事件","修改 State 或组装机动航段", "state"),
        n("send",1,1,"sendParamsToEcu(scope)","JSON 参数发送", "external"),
        n("ws",2,1,"WebSocket 对端","工程中没有服务端实现", "warn"),
        n("recv",2,2,"ws.onmessage","sensor → 合并 State.sensor\nack/status → 日志"),
        n("export",1,3,"CSV / JSON 导出事件","buildCsv/buildJson → downloadFile", "external"),
    ], [e("start","tick","定时回调",True),e("tick","algo"),e("algo","draw"),e("edit","send","部分事件",True),e("send","ws","JSON",True),e("ws","recv","事件",True),e("tick","ws","cmd JSON",True)],
    "onmessage 不会调用 controlStep；网页参数保存仅在内存 State 中，并非 Flash 持久化或真正 CRC32。")

    diagram("14_tools_tests", "14 / 测试、离线绘图与文档工具", "本层消费控制代码或结果文件，不属于每个控制周期。全部辅助函数也收录于附录图谱。", [
        n("cmake",0,0,"CMake 配置","ar_headers → tests / sim / tools"),
        n("ctest",0,1,"CTest / GoogleTest","110 个 TEST 显式测试用例\n另有 6 个仿真冒烟注册项"),
        n("cpp",0,2,"测试体与辅助函数","直接调用对应模块；存储注入 lambda"),
        n("shell",1,0,"gen_all_videos / matrix","Shell 场景批处理", "external"),
        n("sim",1,1,"ar_sim 子进程","生成 CSV", "external"),
        n("plot",1,2,"plot_sim / plot_convergence","load_csv → 指标/曲线 → 图片", "external"),
        n("anim",1,3,"animate_sim","load_csv → init/update 回调\nwriter → MP4/GIF", "external"),
        n("docs",2,0,"docs 下研究脚本","生成图表、论文插图、PPT", "external"),
        n("pdf",2,1,"md2pdf / render_*","Markdown/HTML/PPTX → PDF", "external"),
        n("probe",2,2,"optimization_assets/*probe","独立诊断 main，非构建目标", "gate"),
    ], [e("cmake","ctest"),e("ctest","cpp","测试执行"),e("shell","sim","启动进程",True),e("sim","plot","CSV",True),e("sim","anim","CSV",True),e("docs","pdf","文档产物",True)],
    "测试数字来自当前源码盘点，不代表本次运行通过；生成视频脚本可能覆盖结果，不应为查架构而执行。")
    (OUT.parent / "figures.md").write_text("# 分层流程图\n\n" + "\n\n".join(f"## {title}\n\n![{title}](figures/{name}.png)\n\n[SVG 源图](figures/{name}.svg)" for name, title in MANIFEST) + "\n", encoding="utf-8")
    print(f"Generated {len(MANIFEST)} diagrams as PNG and SVG in {OUT}")


if __name__ == "__main__":
    main()
