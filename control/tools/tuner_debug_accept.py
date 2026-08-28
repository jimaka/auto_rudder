#!/usr/bin/env python3
# tuner_debug_accept.py
# 实船调试接口全功能验收：驱动 ar_tuner_debug，对 9 个功能组的每条命令逐项断言
# 预期输出（数值经手工核算，与 control/tests/test_tuner_debug_api.cpp 互补——
# 那里测 API 层，这里测 CLI 端到端含解析与输出协议）。
# 用法：python3 tools/tuner_debug_accept.py [--exe PATH]
# 退出码：全部 PASS 为 0，任一 FAIL 为 1（CI 可用）。
import argparse
import math
import os
import subprocess
import sys

PASS, FAIL = 0, 0


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  PASS {name}")
    else:
        FAIL += 1
        print(f"  FAIL {name}  {detail}")


class Cli:
    def __init__(self, exe):
        self.p = subprocess.Popen([exe], stdin=subprocess.PIPE,
                                  stdout=subprocess.PIPE, text=True, bufsize=1)

    def cmd(self, line):
        self.p.stdin.write(line + "\n")
        self.p.stdin.flush()
        return self.p.stdout.readline().strip()

    def fields(self, line):
        resp = self.cmd(line)
        assert resp.startswith("OK"), f"{line} -> {resp}"
        out = {}
        for kv in resp.split()[1:]:
            if "=" in kv:
                k, v = kv.split("=", 1)
                try:
                    out[k] = float(v)
                except ValueError:
                    pass
        return out

    def close(self):
        self.cmd("quit")
        self.p.wait(timeout=3)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument("--exe", default=os.path.join(here, "..", "build", "tools", "ar_tuner_debug"))
    args = ap.parse_args()
    c = Cli(args.exe)

    # ========== 组 1：层2 极点配置与稳定门 ==========
    print("[组1] 极点配置 / 稳定门")
    f = c.fields("pole 0.2 8")
    check("pole 默认值 Kp=1.6 Kd=8.6",
          abs(f["Kp"] - 1.6) < 1e-6 and abs(f["Kd"] - 8.6) < 1e-6, str(f))
    f = c.fields("pole 0.2 8 1.0 0.25")
    check("pole 自定义 ζ=1.0 ωn=0.25 → Kp=2.5 Kd=15",
          abs(f["Kp"] - 2.5) < 1e-6 and abs(f["Kd"] - 15.0) < 1e-6, str(f))
    r = c.cmd("pole 0 8")
    check("pole 退化输入拒绝", r.startswith("ERR"), r)
    check("check 标称过门", c.fields("check 0.2 8 1.6 8.6 0")["pass"] == 1)
    check("check Kp<0 拒绝", c.fields("check 0.2 8 -1.6 8.6 0")["pass"] == 0)
    check("check 带宽超限拒绝", c.fields("check 0.2 8 100 8.6 0")["pass"] == 0)
    check("check 低阻尼拒绝", c.fields("check 0.2 8 1.6 0 0")["pass"] == 0)
    check("check 振荡超限拒绝", c.fields("check 0.2 8 1.6 8.6 6")["pass"] == 0)

    # ========== 组 2：基线管理 ==========
    print("[组2] 基线管理")
    check("baseline 建立", c.fields("baseline 0.2 8")["established"] == 1)
    s = c.fields("status")
    check("status 基线一致", abs(s["baseKp"] - 1.6) < 1e-6 and abs(s["baseK"]) - 0.2 < 1e-9, str(s))
    check("eval 漂移 10% 在阈值内拒绝", c.fields("eval 0.22 8.8 0 1 0")["accepted"] == 0)
    check("eval 漂移 40% 超阈值接受", c.fields("eval 0.28 11.2 0 1 0")["accepted"] == 1)
    s = c.fields("status")
    check("重基线后 baseK=0.28 且梯度清零", abs(s["baseK"] - 0.28) < 1e-9 and s["gradKp"] == 0.0, str(s))
    r = c.cmd("baseline 0 8")
    check("baseline 退化 K 拒绝", "established=0" in r, r)
    c.cmd("baseline 0.2 8")  # 恢复标准基线供后续组使用

    # ========== 组 3：ESC 各环节 ==========
    print("[组3] ESC 载波/累积/结算/裁决/限幅")
    f = c.fields("dither 1 1")  # enabled_=false（reset 后未开）
    check("dither 未开 enabled 不动", f["Kp"] == 1.0 and f["Kd"] == 1.0, str(f))
    c.cmd("forceenabled 1")
    f = c.fields("dither 1 1")  # phase=0 → sin=0, cos=1
    check("dither 相位0: Kp 不变 Kd+5%", abs(f["Kp"] - 1.0) < 1e-9 and abs(f["Kd"] - 1.05) < 1e-9, str(f))
    c.cmd("forcephase 1.5707963267948966")  # π/2
    f = c.fields("dither 1 1")
    check("dither 相位π/2: Kp+5% Kd 不变", abs(f["Kp"] - 1.05) < 1e-9 and abs(f["Kd"] - 1.0) < 1e-6, str(f))
    c.cmd("forcephase 0")
    for k in range(600):  # 600 拍 e=3°·sin(2πk/1000), δ=1.5（相位保持 0 → 全部进 Kd 通道）
        c.cmd(f"accum {3.0 * math.sin(2 * math.pi * k / 1000.0):.4f} 1.5 1")
    s = c.fields("status")
    check("accum 600 拍入窗", s["samples"] == 600 and s["demodN"] == 600, str(s))
    check("demod 单通道: demodKp=0 demodKd>0", s["demodKp"] == 0.0 and s["demodKd"] > 40.0, str(s))
    j = c.fields("j")["J"]
    check("J 累积为正且量级合理", 100.0 < j < 130.0, f"J={j}")
    f = c.fields("step 1.6 8.6")
    check("step 梯度下降: Kd 减小、Kp 不变", f["Kd"] < 8.6 and abs(f["Kp"] - 1.6) < 1e-9, str(f))
    check("step 限幅内", 8.6 - f["Kd"] <= 0.86 + 1e-9, str(f))
    s = c.fields("status")
    check("step 后窗口清空", s["samples"] == 0 and s["demodN"] == 0, str(s))
    f = c.fields("settle 1 0.2 8")
    check("settle 接受路径: 增益持久化", abs(f["curKp"] - 1.6) < 1e-6 and f["curKd"] < 8.6, str(f))
    c.cmd("forcegains 1000000 1000000")
    c.cmd("forcegrad 1000000 1000000")
    for _ in range(10):
        c.cmd("accum 1.0 1.0 1")
    f = c.fields("settle 1 0.2 8")
    check("settle 回退路径: 坏增益被拦下", f["curKp"] < 100.0, str(f))
    check("clamp 比例上限", abs(c.fields("clamp 5.0 1.0")["value"] - 1.1) < 1e-9)
    check("clamp 绝对下限防锁死", abs(c.fields("clamp 1.0 0.0")["value"] - 1e-3) < 1e-12)

    # ========== 组 4：振荡检测 ==========
    print("[组4] 振荡检测")
    c.cmd("oscreset")
    for i in range(200):  # 两个完整方波周期 ±2°
        e = 2.0 if (i // 50) % 2 == 0 else -2.0
        c.cmd(f"oscfeed {e}")
    o = c.fields("osc")["osc"]
    check("方波 ±2° → 峰-峰 ≈4°", 3.5 < o < 4.5, f"osc={o}")
    c.cmd("oscreset")
    check("oscreset 清零", c.fields("osc")["osc"] == 0.0)

    # ========== 组 5：回退/重锚/生命周期 ==========
    print("[组5] 回退 / 重锚 / suspend / reset")
    f = c.fields("revert")
    check("revert 返回基线", abs(f["Kp"] - 1.6) < 1e-6 and abs(f["Kd"] - 8.6) < 1e-6, str(f))
    c.cmd("forcegrad 0.5 -0.5")
    for _ in range(10):
        c.cmd("accum 1.0 1.0 1")
    c.cmd("rebase")
    s = c.fields("status")
    check("rebase 清梯度清窗留基线",
          s["gradKp"] == 0.0 and s["samples"] == 0 and s["established"] == 1, str(s))
    for _ in range(10):
        c.cmd("accum 1.0 1.0 1")
    c.cmd("suspend")
    s = c.fields("status")
    check("suspend 清测量留基线",
          s["enabled"] == 0 and s["samples"] == 0 and s["established"] == 1, str(s))
    c.cmd("reset")
    s = c.fields("status")
    check("reset 全量复位", s["established"] == 0 and s["curKp"] == 0.0, str(s))

    # ========== 组 6：参数读写 ==========
    print("[组6] 参数读写")
    f = c.fields("params")
    check("params 默认值 escAmp=0.05", abs(f["escAmp"] - 0.05) < 1e-9, str(f))
    c.cmd("setparam escAmp 0.08")
    f = c.fields("params")
    check("setparam 生效", abs(f["escAmp"] - 0.08) < 1e-9, str(f))
    s = c.fields("status")
    check("setparam 复位状态（基线清空）", s["established"] == 0, str(s))
    r = c.cmd("setparam badname 1")
    check("setparam 未知名报 ERR", r.startswith("ERR"), r)
    c.cmd("setparam escAmp 0.05")  # 恢复默认，避免参数跨组泄漏（reset 不恢复 Params）

    # ========== 组 7：强制注入 ==========
    print("[组7] 强制注入")
    c.cmd("baseline 0.2 8")
    c.cmd("forcegains -1 999")
    s = c.fields("status")
    check("forcegains 旁路稳定门", s["curKp"] == -1.0 and s["curKd"] == 999.0, str(s))
    check("forcegains 不污染基线", abs(s["baseKp"] - 1.6) < 1e-6, str(s))
    f = c.fields("revert")
    check("revert 恢复", abs(f["Kp"] - 1.6) < 1e-6, str(f))
    c.cmd("forceenabled 1")
    c.cmd("forcephase 3.141592653589793")  # π
    f = c.fields("dither 1 1")             # sin(π)=0, cos(π)=-1 → Kd -5%
    check("forcephase 控制载波", abs(f["Kd"] - 0.95) < 1e-6, str(f))
    c.cmd("forcephase 0")
    c.cmd("forcegrad 2.0 3.0")
    s = c.fields("status")
    check("forcegrad 写入积分器", abs(s["gradKp"] - 2.0) < 1e-6 and abs(s["gradKd"] - 3.0) < 1e-6, str(s))
    c.cmd("reset")

    # ========== 组 8：tick 主流程 ==========
    print("[组8] tick 主流程编排")
    f = c.fields("tick 10 9 1 20 0.02 1.0 2.0")   # 20kn > 15kn
    s = c.fields("status")
    check("高速去激活 enabled=0", s["enabled"] == 0, str(s))
    f = c.fields("tick 10 9 1 12 0.02 9.9 9.9")   # 无基线 → 跟随调度
    check("无基线跟随调度值", abs(f["Kp"] - 9.9) < 1e-9 and abs(f["Kd"] - 9.9) < 1e-9, str(f))
    for _ in range(5):                            # 带辨识快照 → 建基线
        c.fields("tick 10 9.5 1 12 0.02 9.9 9.9 1 1 0.2 8")
    s = c.fields("status")
    check("tick 辨识评估建基线", s["established"] == 1 and abs(s["curKp"] - 1.6) < 1e-6, str(s))
    f = c.fields("tick 10 9.5 1 12 0.02 9.9 9.9 1 1 0.2 8")
    check("基线建立后不跟随调度", abs(f["Kp"] - 9.9) > 0.1, str(f))

    # ========== 组 9：协议与健壮性 ==========
    print("[组9] 输出协议")
    r = c.cmd("nosuchcmd")
    check("未知命令报 ERR", r.startswith("ERR"), r)
    r = c.cmd("pole 0.2")
    check("参数不足报 ERR 用法", r.startswith("ERR"), r)
    r = c.cmd("accum")
    check("accum 缺参报 ERR", r.startswith("ERR"), r)

    c.close()
    print(f"\n==== 验收结果: {PASS} PASS / {FAIL} FAIL ====")
    sys.exit(1 if FAIL else 0)


if __name__ == "__main__":
    main()
