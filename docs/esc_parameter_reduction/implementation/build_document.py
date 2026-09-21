#!/usr/bin/env python3
"""Build the ESC implementation-plan illustrations and offline HTML.

No control code is imported or executed. All graphics show proposed work,
contracts, or analytical timing, NOT measured implementation progress.
Reuses the existing ESC proposal's drawing helpers without overwriting its art.
"""
from html import escape
from html.parser import HTMLParser
import importlib.util
import json
import os
from pathlib import Path
import re
import sys
from urllib.parse import unquote, urlsplit
import xml.etree.ElementTree as ET

os.environ.setdefault("MPLCONFIGDIR", "/tmp/auto-rudder-esc-plan-mpl")
sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
DOCUMENT = ROOT / "docs/ESC减参方案实施阶段计划.md"
FIGURES = HERE / "figures"
HTML = HERE / "ESC减参实施计划.html"

spec = importlib.util.spec_from_file_location("esc_proposal_figure_helpers", HERE.parent / "generate_figures.py")
art = importlib.util.module_from_spec(spec)
spec.loader.exec_module(art)
art.OUT = FIGURES
plt = art.plt
box, arrow, save = art.box, art.arrow, art.save
C = art.COLORS


def canvas(title, subtitle, height=8):
    fig, ax = plt.subplots(figsize=(15, height))
    ax.set(xlim=(0, 16), ylim=(0, 10))
    ax.axis("off")
    ax.text(.25, 9.65, title, fontsize=22, weight="bold", va="top")
    ax.text(.25, 9.0, subtitle, fontsize=11, color=C["muted"], va="top")
    return fig, ax


def roadmap():
    fig, ax = canvas("01 / 实施路线：逐阶段过门，不一次替换整套 ESC", "P0 是新增准备阶段；A0/A1/B/C 与原方案保持同名，D 拆分主机和设备，E 负责放行。")
    stages = [
        (.25,6.5,"P0 · 冻结基线","源码/配置/用例/指标\n记录现状，不默认全绿","pale","blue"),
        (4.25,6.5,"A0 · 正确性与安全","真实时间 · 全窗统计\n最终安全门 · 生命周期","sand","amber"),
        (8.25,6.5,"A1 · 参数成套推导","配置版本 → 派生计划\n保留旧代价与二维搜索","pale","blue"),
        (12.25,6.5,"B · 归一化二维","代价与坐标无量纲化\n学习档位重新标定","pale","blue"),
        (12.25,3.5,"C · 单变量带宽","固定名义阻尼，搜索 q\n完整扰动包络投影","mint","teal"),
        (8.25,3.5,"D1 · 主机与界面","三项现场设置/遥测\n新旧协议显式区分","mint","teal"),
        (4.25,3.5,"D2 · 目标设备","一个选定平台/BSP\n数值、时序、存储验证","sand","amber"),
        (.25,3.5,"E · 验证与放行","HIL → 受控实船\n按船型/版本逐项放行","sand","amber"),
    ]
    for x,y,t,b,f,c in stages:
        box(ax,x,y,3.5,1.75,t,b,f,c)
    for x in (.25,4.25,8.25):
        arrow(ax,(x+3.55,7.38),(x+3.94,7.38))
    arrow(ax,(14,6.45),(14,5.30))
    for x in (12.25,8.25,4.25):
        arrow(ax,(x-.05,4.38),(x-.44,4.38))
    box(ax,.25,.6,7.5,1.7,"可保留的中间成果","A1 可先交付成套参数；B 可成为二维备选版本\nC 未通过性能门，不需要丢弃 A0/A1/B 的改进","mint","teal")
    box(ax,8.25,.6,7.5,1.7,"门禁比日期优先","算法、主机可先做；板卡缺失只阻塞设备验收\n任何候选未通过安全/正确性检查，不进入实船","sand","amber")
    save(fig,"01_stage_roadmap")


def modules():
    fig, ax = canvas("02 / 模块改造边界：保持 AutoTuner 为调参状态所有者", "带阶段标记的对象均为拟议改造，不表示仓库已经提供这些接口。",9)
    box(ax,.25,6.9,4.5,1.45,"D1 · 用户设置 / 船型配置","现场三项与研发、安全配置分离","mint","teal")
    box(ax,5.7,6.9,4.5,1.45,"A1 · 参数推导器","纯函数：验证 → 生成不可变计划")
    box(ax,11.15,6.9,4.5,1.45,"A0/A1 · 计划应用","窗口边界切换或作废后重锚\n安全收紧/停用立即生效","sand","amber")
    arrow(ax,(4.8,7.62),(5.65,7.62))
    arrow(ax,(10.25,7.62),(11.1,7.62))
    box(ax,.25,4.4,4.5,1.55,"AutopilotCore","继续负责模式、喂辨识、转发\n补齐观测/事件契约，不搬入 ESC")
    box(ax,5.7,4.4,4.5,1.55,"AutoTuner::tick","时钟与窗口 → 代价/解调/学习\n基线、重锚、回退继续在内部")
    box(ax,11.15,4.4,4.5,1.55,"最终含扰动增益检查","安全门 → GainCommand → PD\n无可用增益时明确报告故障","sand","amber")
    arrow(ax,(4.8,5.17),(5.65,5.17))
    arrow(ax,(10.25,5.17),(11.1,5.17))
    arrow(ax,(13.4,6.85),(8,6))
    box(ax,.25,1.85,4.5,1.55,"A0 · 时钟/窗口/生命周期","流式累计；观测与相位对齐\n统一失效原因与状态迁移")
    box(ax,5.7,1.85,4.5,1.55,"B / C · 可替换搜索内核","B：归一化二维\nC：对数带宽一维")
    box(ax,11.15,1.85,4.5,1.55,"D1/D2 · 外围集成","只读遥测、受校验的配置路径\nBSP/NVM 与 Flash 调度独立","mint","teal")
    arrow(ax,(5.9,4.35),(2.5,3.45))
    arrow(ax,(8,4.35),(8,3.45))
    ax.text(.25,.65,"TunerDebugApi 的 force* 仍是调试旁路，不作为现场正式配置通道；网页 JS 不是 C++ 实现的自动绑定。",fontsize=11,color=C["muted"])
    save(fig,"02_module_boundaries")


def window_contract():
    import numpy as np
    fig = plt.figure(figsize=(15,9))
    fig.text(.07,.95,"03 / 窗口契约：冻结配置、对齐观测、整周期结算",fontsize=22,weight="bold")
    fig.text(.07,.90,"解析时间示意，N=1；不是控制器的实测波形。实际窗口可含 N 个完整载波周期。",fontsize=11,color=C["muted"])
    ax=fig.add_axes([.09,.50,.84,.31])
    x=np.linspace(0,2,500)
    ax.axvspan(0,1,color=C["blue"],alpha=.06)
    ax.axvspan(1,2,color=C["teal"],alpha=.06)
    ax.plot(x,np.sin(2*np.pi*x),color=C["blue"],lw=2.5)
    for v in (0,1,2):
        ax.axvline(v,color=C["muted"],ls="--",lw=1)
    ax.text(.5,1.25,"窗口 m：固定模型、代价、时间计划和搜索中心",ha="center",fontsize=11)
    ax.text(1.5,1.25,"窗口 m+1：接受候选后使用新中心",ha="center",fontsize=11)
    ax.set(xlim=(0,2),ylim=(-1.25,1.6),xlabel="时间 / 载波周期 P",ylabel="单位载波（示意）")
    ax.set_xticks([0,.5,1,1.5,2])
    ax.grid(alpha=.15)
    ax2=fig.add_axes([.04,.08,.92,.33])
    ax2.set(xlim=(0,16),ylim=(0,3.8));ax2.axis("off")
    for x0,title,body in [
        (.25,"① 接收观测","用应用时间/相位记录对齐\n不能拿本拍新指令解释旧反馈"),
        (4.25,"② 分段累计","用有效 Δt；跨边界拆分\n记录 Weff 与失效原因"),
        (8.25,"③ 窗末裁决","可信窗口才更新中心\n最终安全门每拍都执行"),
        (12.25,"④ 开始下一窗","正常保留连续滤波状态¹\n配置变化则作废/重锚/预热"),
    ]:
        box(ax2,x0,1.4,3.5,1.65,title,body)
    for x0 in (.25,4.25,8.25):
        arrow(ax2,(x0+3.55,2.23),(x0+3.94,2.23))
    ax2.text(.25,.35,"¹ B 起改变正常窗口滤波连续性；A0/A1 先保留旧语义并单独验证，避免把多种变化混成一次重构。",fontsize=10.5,color=C["muted"])
    save(fig,"03_window_contract")


def comparison():
    fig,ax=canvas("04 / 公平对照：用五组结果隔离四类影响", "B0/B1/B2/B3 是实验组名称，不是实施阶段 B 的子阶段；本图不包含任何试验结果。",8.5)
    columns=[
        ("REF","冻结适用基线","ESC 关闭","无学习","判断在线寻优是否有必要"),
        ("B0","完成 A0 的二维","旧代价","手工配置","修正后的比较起点"),
        ("B1","成套参数二维","旧代价","计划生成","隔离参数配套影响"),
        ("B2","归一化二维","新无量纲代价","二维相对坐标","隔离代价/坐标变化"),
        ("B3","固定阻尼一维","同 B2 的代价","单变量 q","隔离限制搜索空间影响"),
    ]
    for i,(name,path,cost,coord,use) in enumerate(columns):
        x=.25+i*3.16
        box(ax,x,4.6,2.85,3.6,name,f"{path}\n{cost}\n{coord}\n\n{use}","mint" if i in(0,4) else "pale","teal" if i in(0,4) else "blue")
    box(ax,.25,2.15,7.5,1.7,"B0 ↔ B1：先做一致性，再评估配置选择","相同合法底层参数 → 检查实现等价\n换成配套参数后 → 单独比较外部指标")
    box(ax,8.25,2.15,7.5,1.7,"B1 ↔ B2 与 B2 ↔ B3：比较口径不同","代价变了，不直接比较原始 J 数值\nB2/B3 同代价；冻结新中心后复测","sand","amber")
    ax.text(.25,.85,"所有组：同模型/扰动/参考/执行器限制/种子集合/物理时长；独立记录误差、动舵、饱和与回退。",fontsize=11,color=C["muted"])
    save(fig,"04_comparison_groups")


def release():
    fig,ax=canvas("05 / 验收漏斗：每一层都留下可复核证据", "阶段门是进入下一层的条件；安全/正确性失败不能被平均性能改善抵消。",9)
    rows=[
        (6.9,"G0 / P0","基线与指标冻结","源码哈希、配置、失败清单、阈值表"),
        (5.35,"G-A0 / G-A1","契约与配置测试","时间、长窗、失效迁移、推导一致性"),
        (3.8,"G-B / G-C","多工况闭环对照","B0～B3 + REF；接受中心的独立复测"),
        (2.25,"G-D1 / G-D2","主机协议与目标板","配置事务、WCET、RAM、NVM 故障注入"),
        (.7,"G-E","HIL 与受控实船","安全评审、人工接管、按船型/版本放行"),
    ]
    for y,gate,task,proof in rows:
        box(ax,.25,y,3,1.05,gate,"待实施 / 待评审","sand","amber")
        box(ax,4.15,y,4.45,1.05,task,"通过才继续")
        box(ax,9.5,y,6.1,1.05,"本层交付证据",proof,"mint","teal")
        arrow(ax,(3.3,y+.52),(4.1,y+.52))
        arrow(ax,(8.65,y+.52),(9.45,y+.52))
    for row in rows[:-1]:
        y=row[0];arrow(ax,(6.38,y-.04),(6.38,y-.46))
    save(fig,"05_acceptance_gates")


def config_transaction():
    fig,ax=canvas("06 / 配置生效与持久化：两种事务，不混为一谈", "D1/D2 拟议行为：运行计划原子生效与掉电恢复分别验证；普通参数变更不得产生半组状态。",9)
    boxes=[
        (.25,6.85,"接收候选设置","版本/船型/范围/有限数检查","pale","blue"),
        (5.7,6.85,"生成并验证候选计划","包含完整扰动包络与算法版本","pale","blue"),
        (11.15,6.85,"暂存 pending 计划","拒绝时保留原 active，不静默降级","sand","amber"),
        (11.15,4.1,"应用时机判定","普通变更：边界应用或作废重锚\n停用/安全收紧：立即处理","sand","amber"),
        (5.7,4.1,"控制任务原子接管","单写者/双缓冲等受验证机制\n遥测 activePlanId 与状态","mint","teal"),
        (.25,4.1,"运行/重新预热","从新计划开始；不沿用旧梯度\n接受中心不等于立即写 Flash","mint","teal"),
    ]
    for x,y,t,b,f,c in boxes: box(ax,x,y,4.5,1.5,t,b,f,c)
    arrow(ax,(4.8,7.6),(5.65,7.6));arrow(ax,(10.25,7.6),(11.1,7.6))
    arrow(ax,(13.4,6.8),(13.4,5.65))
    arrow(ax,(11.1,4.85),(10.25,4.85));arrow(ax,(5.65,4.85),(4.8,4.85))
    box(ax,.25,.8,7.5,1.9,"显式保存 → 低优先级持久化任务","写非活动槽 → 回读/校验 → 最后提交标记\n每个写入/擦除边界做掉电注入","pale","blue")
    box(ax,8.25,.8,7.5,1.9,"重启恢复 → 校验完整记录","schema/长度/CRC/提交标记/版本/适用性\n保留可用旧槽；不恢复半窗相位和梯度","sand","amber")
    arrow(ax,(7.8,1.75),(8.2,1.75))
    ax.text(.25,.1,"BSP 须确认写入粒度、擦除区与代码执行冲突；仅有 CRC 或双槽示意图不能证明掉电一致性。",fontsize=10.5,color=C["muted"])
    save(fig,"06_configuration_transaction")


def render():
    from markdown_it import MarkdownIt
    md=MarkdownIt("commonmark",{"html":False}).enable("table")
    tokens=md.parse(DOCUMENT.read_text(encoding="utf-8"))
    toc=[]
    for i,t in enumerate(tokens):
        if t.type=="heading_open" and t.tag=="h2":
            uid=f"section-{len(toc)+1}";t.attrSet("id",uid)
            toc.append((uid,tokens[i+1].content))
    def rewrite(items):
        for t in items:
            for a in ("src","href"):
                value=t.attrGet(a)
                if value and not urlsplit(value).scheme and not value.startswith(("#","/")):
                    t.attrSet(a,"../../"+value)
            if t.children:rewrite(t.children)
    rewrite(tokens)
    body=md.renderer.render(tokens,md.options,{})
    nav="".join(f'<a href="#{uid}">{escape(label)}</a>' for uid,label in toc)
    page='''<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>ESC 减参方案实施阶段计划</title><style>
*{box-sizing:border-box}body{margin:0;background:#edf3f7;color:#213d51;font:16px/1.85 system-ui,"Noto Sans CJK JP",sans-serif}aside{position:fixed;inset:0 auto 0 0;width:260px;background:#16364e;padding:24px 16px;overflow:auto;color:white}aside strong{display:block;font-size:22px;margin-bottom:18px}aside a{display:block;color:#c6e4ef;font-size:13px;padding:6px;text-decoration:none}aside a:hover{background:#30566e}main{max-width:1140px;margin-left:285px;padding:36px 42px;background:white}h1{font-size:32px;color:#163b55;line-height:1.5}h2{font-size:24px;color:#205f81;border-bottom:2px solid #dcebf2;padding:22px 0 10px;scroll-margin-top:15px}h3{font-size:19px;color:#376078;margin-top:28px}a{color:#007aa4}img{display:block;width:100%;height:auto;margin:24px 0;border:1px solid #dfebf1;border-radius:8px}blockquote{border-left:4px solid #c68b30;background:#fff7e5;padding:4px 20px;margin:20px 0}table{border-collapse:collapse;width:100%;font-size:13px;margin:20px 0}th,td{border:1px solid #d3e0e7;padding:8px 10px;vertical-align:top;text-align:left}th{background:#eaf2f7}tr:nth-child(even) td{background:#f8fbfc}code{background:#edf4f8;padding:2px 4px;border-radius:4px;overflow-wrap:anywhere;font-size:.9em}pre{background:#19384e;color:#d7edf6;padding:18px;border-radius:8px;overflow:auto;font:13px/1.7 monospace}pre code{background:none;padding:0;color:inherit}li{margin:7px 0}.toolbar{background:#eaf5f7;padding:12px;font-size:14px;border-radius:6px}@media(max-width:950px){aside{position:static;width:auto;max-height:260px}main{margin:0;padding:22px}h1{font-size:26px}}@media print{aside,.toolbar{display:none}body{background:white;font-size:10pt}main{margin:0;padding:0;max-width:none}h2,h3{break-after:avoid}img{break-inside:avoid;max-height:220mm;object-fit:contain}tr{break-inside:avoid}pre{white-space:pre-wrap}a{color:inherit;text-decoration:none}@page{size:A4;margin:16mm}}
</style></head><body><aside><strong>ESC 减参 · 实施计划</strong>NAV</aside><main><p class="toolbar"><a href="../../ESC减参方案实施阶段计划.md">Markdown 原文</a> · <a href="../../ESC减参方案设计文档.md">设计方案</a> · 支持离线阅读及浏览器打印 PDF</p>BODY</main></body></html>'''
    HTML.write_text(page.replace("NAV",nav).replace("BODY",body),encoding="utf-8")
    return tokens


class Links(HTMLParser):
    def __init__(self,text):
        super().__init__();self.links=[];self.ids=[];self.feed(text)
    def handle_starttag(self,tag,attrs):
        attrs=dict(attrs)
        if "id" in attrs:self.ids.append(attrs["id"])
        self.links.extend(attrs[a] for a in ("href","src") if a in attrs)


def validate():
    from markdown_it import MarkdownIt
    from PIL import Image
    page=Links(HTML.read_text(encoding="utf-8"))
    assert len(page.ids)==len(set(page.ids))
    count=0
    def check(origin,value):
        nonlocal count
        parsed=urlsplit(value)
        if parsed.scheme:return
        target=(origin.parent/unquote(parsed.path)).resolve() if parsed.path else origin.resolve()
        assert target.exists(),(origin,value)
        if parsed.fragment and target==HTML.resolve():assert parsed.fragment in page.ids,value
        count+=1
    for link in page.links:check(HTML,link)
    md=MarkdownIt("commonmark").enable("table")
    for origin in (DOCUMENT,HERE/"README.md"):
        def walk(tokens):
            for t in tokens:
                for a in ("href","src"):
                    if value:=t.attrGet(a):check(origin,value)
                if t.children:walk(t.children)
        walk(md.parse(origin.read_text(encoding="utf-8")))
    pngs=sorted(FIGURES.glob("*.png"));svgs=sorted(FIGURES.glob("*.svg"))
    assert len(pngs)==len(svgs)==6
    for path in pngs:
        with Image.open(path) as img:img.verify()
    for path in svgs:ET.parse(path)
    src=DOCUMENT.read_text(encoding="utf-8")
    for phase in ("P0","A0","A1","B","C","D1","D2","E"):
        assert re.search(rf"^## \d+\. {phase}：",src,re.M),phase
    assert "未执行算法测试" in src
    result={"figures":6,"local_links_checked":count,"html_sections":len(page.ids),
            "control_algorithms_executed":False,"status":"document checks passed"}
    (HERE/"validation.json").write_text(json.dumps(result,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print(json.dumps(result,ensure_ascii=False))


def main():
    art.configure()
    roadmap();modules();window_contract();comparison();release();config_transaction()
    render()
    (HERE/"validation.json").write_text('{"status":"validation pending","control_algorithms_executed":false}\n',encoding="utf-8")
    validate()


if __name__=="__main__":main()
