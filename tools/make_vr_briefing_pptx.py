#!/usr/bin/env python3
"""例会用简报：当前限制对照 OpenArm，以及给香农的需求。"""

import html
import zipfile
from pathlib import Path

OUT = Path("/home/szxn/x1_coffee_robot_sdk/VR示教采集_例会汇报.pptx")

W, H = 12192000, 6858000
NAVY = "14324F"
INK = "1C2834"
MUTED = "5E6A75"
PAPER = "F4F1EB"
WHITE = "FFFFFF"
LINE = "E4DDD2"
TEAL = "1F6B54"
TEAL_BG = "E7F2ED"
CLAY = "C4622D"
CLAY_BG = "F8EDE6"
GOLD = "F3E6C8"


def esc(text: str) -> str:
    return html.escape(text, quote=False)


def run(text, size, color, bold=False):
    b = ' b="1"' if bold else ""
    return (
        f'<a:r><a:rPr lang="zh-CN" altLang="en-US" sz="{size}"{b} dirty="0">'
        f'<a:solidFill><a:srgbClr val="{color}"/></a:solidFill>'
        f'<a:latin typeface="Microsoft YaHei"/>'
        f'<a:ea typeface="微软雅黑"/>'
        f'<a:cs typeface="Microsoft YaHei"/>'
        f"</a:rPr><a:t>{esc(text)}</a:t></a:r>"
    )


def paragraph(text, size=1800, color=INK, bold=False, align="l", spc=0):
    al = {"l": "l", "c": "ctr", "r": "r"}[align]
    before = f' spcBef="{spc}"' if spc else ""
    return (
        f'<a:p><a:pPr algn="{al}"{before}><a:buNone/></a:pPr>'
        f"{run(text, size, color, bold)}</a:p>"
    )


def shape(sid, name, x, y, cx, cy, fill, paragraphs, anchor="t", radius=6000):
    body = "".join(paragraphs)
    geom = (
        f'<a:prstGeom prst="roundRect"><a:avLst>'
        f'<a:gd name="adj" fmla="val {radius}"/></a:avLst></a:prstGeom>'
        if radius
        else '<a:prstGeom prst="rect"><a:avLst/></a:prstGeom>'
    )
    return f"""
    <p:sp>
      <p:nvSpPr>
        <p:cNvPr id="{sid}" name="{esc(name)}"/>
        <p:cNvSpPr/>
        <p:nvPr/>
      </p:nvSpPr>
      <p:spPr>
        <a:xfrm><a:off x="{x}" y="{y}"/><a:ext cx="{cx}" cy="{cy}"/></a:xfrm>
        {geom}
        <a:solidFill><a:srgbClr val="{fill}"/></a:solidFill>
        <a:ln><a:noFill/></a:ln>
      </p:spPr>
      <p:txBody>
        <a:bodyPr wrap="square" lIns="140000" tIns="90000" rIns="140000" bIns="80000" anchor="{anchor}"/>
        <a:lstStyle/>
        {body}
      </p:txBody>
    </p:sp>"""


def textbox(sid, name, x, y, cx, cy, paragraphs, anchor="t"):
    body = "".join(paragraphs)
    return f"""
    <p:sp>
      <p:nvSpPr>
        <p:cNvPr id="{sid}" name="{esc(name)}"/>
        <p:cNvSpPr txBox="1"/>
        <p:nvPr/>
      </p:nvSpPr>
      <p:spPr>
        <a:xfrm><a:off x="{x}" y="{y}"/><a:ext cx="{cx}" cy="{cy}"/></a:xfrm>
        <a:prstGeom prst="rect"><a:avLst/></a:prstGeom>
        <a:noFill/>
        <a:ln><a:noFill/></a:ln>
      </p:spPr>
      <p:txBody>
        <a:bodyPr wrap="square" lIns="0" tIns="0" rIns="0" bIns="0" anchor="{anchor}"/>
        <a:lstStyle/>
        {body}
      </p:txBody>
    </p:sp>"""


def cell(text, fill, color, bold=False, size=1300):
    return f"""
    <a:tc>
      <a:txBody>
        <a:bodyPr anchor="ctr" lIns="90000" tIns="40000" rIns="80000" bIns="40000"/>
        <a:lstStyle/>
        {paragraph(text, size, color, bold)}
      </a:txBody>
      <a:tcPr>
        <a:solidFill><a:srgbClr val="{fill}"/></a:solidFill>
        <a:lnL w="6350"><a:solidFill><a:srgbClr val="FFFFFF"/></a:solidFill></a:lnL>
        <a:lnR w="6350"><a:solidFill><a:srgbClr val="FFFFFF"/></a:solidFill></a:lnR>
        <a:lnT w="6350"><a:solidFill><a:srgbClr val="FFFFFF"/></a:solidFill></a:lnT>
        <a:lnB w="6350"><a:solidFill><a:srgbClr val="FFFFFF"/></a:solidFill></a:lnB>
      </a:tcPr>
    </a:tc>"""


def table(sid, x, y, cx, cy, col_w, row_h, rows):
    grid = "".join(f'<a:gridCol w="{w}"/>' for w in col_w)
    trs = []
    for i, row in enumerate(rows):
        cells = "".join(cell(*item) for item in row)
        trs.append(f'<a:tr h="{row_h[i]}">{cells}</a:tr>')
    body = "".join(trs)
    return f"""
    <p:graphicFrame>
      <p:nvGraphicFramePr>
        <p:cNvPr id="{sid}" name="对比表"/>
        <p:cNvGraphicFramePr><a:graphicFrameLocks noGrp="1"/></p:cNvGraphicFramePr>
        <p:nvPr/>
      </p:nvGraphicFramePr>
      <p:xfrm><a:off x="{x}" y="{y}"/><a:ext cx="{cx}" cy="{cy}"/></p:xfrm>
      <a:graphic>
        <a:graphicData uri="http://schemas.openxmlformats.org/drawingml/2006/table">
          <a:tbl>
            <a:tblPr/>
            <a:tblGrid>{grid}</a:tblGrid>
            {body}
          </a:tbl>
        </a:graphicData>
      </a:graphic>
    </p:graphicFrame>"""


def footer(page, total=8):
    return textbox(
        90, "页码", 10400000, 6480000, 1400000, 280000,
        [paragraph(f"{page}  /  {total}", 1100, MUTED, align="r")],
    )


def header(title):
    return (
        shape(2, "顶栏", 0, 0, W, 980000, NAVY, [], radius=0)
        + textbox(3, "标题", 480000, 250000, 10800000, 520000, [paragraph(title, 2800, WHITE, True)])
    )


def slide_xml(parts):
    return f"""<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<p:sld xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main"
 xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"
 xmlns:p="http://schemas.openxmlformats.org/presentationml/2006/main">
  <p:cSld>
    <p:bg><p:bgPr><a:solidFill><a:srgbClr val="{PAPER}"/></a:solidFill><a:effectLst/></p:bgPr></p:bg>
    <p:spTree>
      <p:nvGrpSpPr>
        <p:cNvPr id="1" name=""/>
        <p:cNvGrpSpPr/>
        <p:nvPr/>
      </p:nvGrpSpPr>
      <p:grpSpPr>
        <a:xfrm><a:off x="0" y="0"/><a:ext cx="0" cy="0"/>
          <a:chOff x="0" y="0"/><a:chExt cx="0" cy="0"/></a:xfrm>
      </p:grpSpPr>
      {''.join(parts)}
    </p:spTree>
  </p:cSld>
  <p:clrMapOvr><a:masterClrMapping/></p:clrMapOvr>
</p:sld>"""


def slide_cover():
    return slide_xml([
        shape(2, "底", 0, 0, W, H, NAVY, [], radius=0),
        shape(3, "条", 0, 0, 220000, H, CLAY, [], radius=0),
        textbox(4, "眉题", 720000, 1450000, 10000000, 360000,
                [paragraph("例会汇报  ·  2026.10.30", 1600, "D7C4A8")]),
        textbox(5, "标题", 720000, 1900000, 10800000, 1800000, [
            paragraph("VR 示教与 VLA 闭环", 4400, WHITE, True),
            paragraph("现有控制链路的限制，及给香农的二次开发需求", 2600, "F3E6C8", True, spc=140000),
        ]),
        textbox(6, "结论", 720000, 4200000, 10400000, 1400000, [
            paragraph("现有 SDK 可执行预先规划的关节轨迹。", 2000, "E7EEF5"),
            paragraph("VR 实时伺服、拖动示教，以及后续 VLA 的动作下发，目前都过不去。", 2000, "E7EEF5", spc=80000),
            paragraph("VLA：视觉-语言-动作策略。模型在上位机推理，关节指令发给执行层。", 1600, "A9B8C7", spc=140000),
        ]),
        textbox(7, "脚", 720000, 6200000, 10000000, 320000,
                [paragraph("对照 OpenArm 实时伺服  ·  采集与策略闭环的接口需求", 1400, "A9B8C7")]),
    ])


def slide_framework():
    layers = [
        (NAVY, "01    规划层", "MoveIt 2", "关节空间规划，经 FollowJointTrajectory 下发。腰、左臂、右臂可分开规划。"),
        (TEAL, "02    控制层", "ros2_control · 100 Hz", "三个 JointTrajectoryController。指令只有关节位置，没有力矩。"),
        ("2E5984", "03    硬件插件", "脉冲换算与保护", "弧度换成脉冲，15 轴汇入同一接口。运动未使能时，指令不下总线。"),
        (CLAY, "04    执行层", "固高 GTN · EtherCAT", "控制卡做主站。轴 1 腰，2–8 左臂，9–15 右臂。运动由卡内 PT/PVT 执行。"),
    ]
    parts = [
        header("当前 SDK 的控制框架"),
        textbox(4, "说明", 480000, 1080000, 11000000, 320000, [
            paragraph("指令自上而下。上层给出关节轨迹，轴怎么动由最下面的固高卡决定。", 1600, MUTED),
        ]),
    ]
    y = 1480000
    h = 1050000
    gap = 70000
    sid = 10
    for color, name, title, body in layers:
        parts.append(shape(sid, name, 400000, y, 3200000, h, color, [
            paragraph(name, 2000, WHITE, True),
        ], anchor="ctr"))
        parts.append(shape(sid + 1, title, 3600000, y, 8200000, h, WHITE, [
            paragraph(title, 2000, NAVY, True),
            paragraph(body, 1500, INK, spc=80000),
        ], anchor="ctr"))
        y += h + gap
        sid += 2
    parts.append(textbox(40, "承接", 480000, 6120000, 11000000, 320000, [
        paragraph("后面要说的限制，集中在执行层：轨迹交给控制卡之后，上位机不能再改。", 1600, "8A3E16"),
    ]))
    parts.append(footer(2))
    return slide_xml(parts)


def slide_table():
    header_row = [
        ("能力", NAVY, WHITE, True, 1400),
        ("OpenArm", TEAL, WHITE, True, 1400),
        ("当前 SDK + 固高 GTN", CLAY, WHITE, True, 1400),
    ]
    body = [
        ("实时跟随", "周期下发位置，只保留最新目标", "轨迹写入控制卡，按 PT/PVT 表执行"),
        ("中途停止", "目标撤销后，关节立即停止", "队列里已缓存的点仍会走完"),
        ("拖动示教", "可切柔顺，外力可推动关节", "位置环锁死，外力拖不动"),
        ("接触让位", "力矩或阻抗可减小接触力", "继续跟踪位置，接触时顶死"),
        ("力矩回读", "驱动器周期回传力矩或电流", "现有 PDO 未映射力矩，读不到"),
        ("策略闭环", "新推理可覆盖尚未执行的动作", "已入队的点不能改写，旧动作会走完"),
    ]
    rows = [header_row]
    for i, (a, b, c) in enumerate(body):
        bg = WHITE if i % 2 == 0 else "F7F3EC"
        rows.append([
            (a, bg, INK, True, 1400),
            (b, TEAL_BG, TEAL, False, 1400),
            (c, CLAY_BG, "8A3E16", False, 1400),
        ])
    col_w = [2400000, 4500000, 4500000]
    row_h = [460000] + [680000] * 6
    return slide_xml([
        header("与 OpenArm 的控制差异"),
        textbox(4, "说明", 480000, 1080000, 11000000, 340000, [
            paragraph("同一处差异同时挡住 VR 遥操作和后续 VLA：OpenArm 每周期覆盖目标；本机提交给固高卡后不能改写。", 1500, MUTED),
        ]),
        table(5, 400000, 1480000, sum(col_w), sum(row_h), col_w, row_h, rows),
        footer(3),
    ])


def slide_examples():
    return slide_xml([
        header("两个典型场景"),
        shape(4, "左卡", 400000, 1280000, 5600000, 4900000, WHITE, []),
        shape(5, "左条", 460000, 1420000, 90000, 4620000, CLAY, [], radius=0),
        textbox(6, "左文", 700000, 1480000, 5100000, 4500000, [
            paragraph("遥操作中断", 2400, CLAY, True),
            paragraph("操作者松开手柄，不再下发新目标。", 1700, INK, spc=160000),
            paragraph("OpenArm：下一周期无新目标，关节立即停止。", 1700, TEAL, spc=140000),
            paragraph("当前：GTN 的 PT 队列里仍有已缓存点，手臂会把旧轨迹走完后才停。", 1700, INK, spc=140000),
            paragraph("立即停止只能走急停，当次采集中断。", 1700, "8A3E16", spc=140000),
        ]),
        shape(7, "右卡", 6200000, 1280000, 5600000, 4900000, WHITE, []),
        shape(8, "右条", 6260000, 1420000, 90000, 4620000, TEAL, [], radius=0),
        textbox(9, "右文", 6500000, 1480000, 5100000, 4500000, [
            paragraph("拖动示教与轻接触", 2400, TEAL, True),
            paragraph("示教前需拖动腕部到位，或轻触杯体。", 1700, INK, spc=160000),
            paragraph("OpenArm：可切出力矩或柔顺，外力可推动；接触时可降低输出力矩。", 1700, TEAL, spc=140000),
            paragraph("当前：位置环锁住关节，拖不动。障碍物阻挡时仍向目标位置顶。", 1700, INK, spc=140000),
            paragraph("接触力矩目前无法按周期记录。", 1700, "8A3E16", spc=140000),
        ]),
        footer(4),
    ])


def slide_vla():
    return slide_xml([
        header("VLA 闭环卡在同一处"),
        textbox(4, "引", 480000, 1100000, 11000000, 360000, [
            paragraph("策略不在控制卡上跑。上位机推理出关节目标或短动作块，再发给执行层。", 1600, MUTED),
        ]),
        shape(5, "左卡", 400000, 1580000, 5600000, 3900000, WHITE, []),
        textbox(6, "左文", 640000, 1760000, 5160000, 3540000, [
            paragraph("策略侧要什么", 2200, TEAL, True),
            paragraph("一次推理给出当前目标，或一小段未来动作。", 1600, INK, spc=140000),
            paragraph("下次推理必须盖住未执行部分，旧块不能走完。", 1600, INK, spc=100000),
            paragraph("观测要周期回读位置、速度、力矩，带时间戳。", 1600, INK, spc=100000),
            paragraph("单步超限时拒绝该点，不要因此把回合急停掉。", 1600, INK, spc=100000),
        ]),
        shape(7, "右卡", 6200000, 1580000, 5600000, 3900000, WHITE, []),
        textbox(8, "右文", 6440000, 1760000, 5160000, 3540000, [
            paragraph("当前执行层做不到", 2200, CLAY, True),
            paragraph("PT/PVT 写入后不能改，队列中的旧点会走完。", 1600, INK, spc=140000),
            paragraph("预缓冲约 80 ms，这段已承诺，新推理盖不住。", 1600, INK, spc=100000),
            paragraph("力矩未映射进 PDO，策略观测缺这一维。", 1600, INK, spc=100000),
            paragraph("不能单点拒绝并保持使能，异常时只能急停。", 1600, INK, spc=100000),
        ]),
        shape(9, "底句", 400000, 5680000, 11400000, 680000, GOLD, []),
        textbox(10, "底字", 640000, 5840000, 11000000, 400000, [
            paragraph("要的是可覆盖的实时执行接口。不是让香农训练 VLA。", 1800, NAVY, True),
        ]),
        footer(5),
    ])


def slide_needs():
    cards = [
        ("1", "实时跟随，可覆盖旧动作", "约 100 Hz 跟踪最新位置，或按推理周期下发短动作块。|新结果必须盖住未执行部分。|约 80 ms 的预缓冲现在不能改，要去掉。|单臂动时，另一臂与腰保持。"),
        ("2", "柔顺与接触让位", "提供可在线切换的柔顺模式：外力可拖动，松手后不因重力下坠。接触时按给定力矩或阻抗响应，避免位置死顶。切换无需重启控制栈。"),
        ("3", "状态与力矩回读", "不低于 100 Hz 回读关节位置、速度、力矩或等效电流、报警，并带时间戳，供 VLA 观测。若无力传感器，需说明仅为关节电流估计，并给出单位、方向和精度。"),
        ("4", "安全不能把回合打断", "柔顺模式下急停与抱闸仍然有效。断链时关节自行停止。柔顺开启时必须关闭卡内 PT/PVT。单步超速或超限位时拒绝该点并报警，不要执行，也不要因此整机急停。"),
    ]
    parts = [header("请香农补齐的能力")]
    positions = [(400000, 1220000), (6240000, 1220000), (400000, 3900000), (6240000, 3900000)]
    sid = 10
    for (num, title, body), (x, y) in zip(cards, positions):
        parts.append(shape(sid, title, x, y, 5560000, 2480000, WHITE, []))
        parts.append(textbox(sid + 1, title + "文", x + 220000, y + 160000, 5120000, 2160000, [
            paragraph(f"{num}   {title}", 2200, NAVY, True),
            *[paragraph(line, 1500, INK, spc=70000) for line in body.split("|")],
        ]))
        sid += 2
    parts.append(footer(6))
    return slide_xml(parts)


def slide_done():
    items = [
        "15 轴上电，模式与 PDO 配置不被驱动器拒绝。",
        "约 10 Hz 替换动作块，10 分钟内只执行最新块。",
        "柔顺模式下可拖动右臂，松手后不自由下落。",
        "停发或超限时拒绝该点，不执行已缓存轨迹。",
        "轻接触固定物时力矩上升，但不顶到驱动报警。",
        "位置、速度、力矩可连续回读，柔顺急停有效。",
    ]
    parts = [
        header("验收条件"),
        textbox(4, "引", 480000, 1120000, 11000000, 320000, [
            paragraph("真机示例需同时覆盖 VR 跟随和 VLA 动作块替换，满足下面 6 条。", 1600, MUTED),
        ]),
    ]
    for i, text in enumerate(items):
        col = i % 2
        row = i // 2
        x = 400000 + col * 5900000
        y = 1560000 + row * 1450000
        parts.append(shape(10 + i, f"条{i}", x, y, 5600000, 1280000, WHITE, []))
        parts.append(textbox(30 + i, f"字{i}", x + 240000, y + 280000, 5100000, 780000, [
            paragraph(f"0{i + 1}", 1400, CLAY, True),
            paragraph(text, 1700, INK, spc=40000),
        ]))
    parts.append(footer(7))
    return slide_xml(parts)


def slide_split():
    return slide_xml([
        header("分工"),
        shape(4, "左", 400000, 1280000, 5600000, 4300000, WHITE, []),
        shape(5, "左顶", 400000, 1280000, 5600000, 720000, NAVY, [], radius=0),
        textbox(6, "左题", 620000, 1460000, 5000000, 420000, [paragraph("香农", 2400, WHITE, True)]),
        textbox(7, "左文", 640000, 2180000, 5100000, 3100000, [
            paragraph("实时跟随，未执行动作可覆盖", 1700, INK, spc=20000),
            paragraph("柔顺或阻抗，接触可让位", 1700, INK, spc=100000),
            paragraph("位置、速度、力矩回读，带时间戳", 1700, INK, spc=100000),
            paragraph("超限拒绝该点，急停与抱闸仍有效", 1700, INK, spc=100000),
            paragraph("真机示例，说明超前量与力矩单位", 1700, INK, spc=100000),
        ]),
        shape(8, "右", 6200000, 1280000, 5600000, 4300000, WHITE, []),
        shape(9, "右顶", 6200000, 1280000, 5600000, 720000, TEAL, [], radius=0),
        textbox(10, "右题", 6420000, 1460000, 5000000, 420000, [paragraph("我方", 2400, WHITE, True)]),
        textbox(11, "右文", 6440000, 2180000, 5100000, 3100000, [
            paragraph("VR 头显与手柄", 1700, INK, spc=20000),
            paragraph("末端位姿到关节角", 1700, INK, spc=100000),
            paragraph("VLA 推理与相机", 1700, INK, spc=100000),
            paragraph("示教与策略数据记录", 1700, INK, spc=100000),
            paragraph("ROS 2 上位机", 1700, INK, spc=100000),
        ]),
        shape(12, "底句", 400000, 5740000, 11400000, 620000, GOLD, []),
        textbox(13, "底字", 640000, 5880000, 11000000, 380000, [
            paragraph("请香农改执行层：实时跟随且可覆盖旧动作、可柔顺、状态能回读。VR 与 VLA 由我方完成。", 1600, NAVY, True),
        ]),
        footer(8),
    ])


SLIDES = [slide_cover, slide_framework, slide_table, slide_examples, slide_vla, slide_needs, slide_done, slide_split]


CONTENT_TYPES = """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
  <Default Extension="xml" ContentType="application/xml"/>
  <Override PartName="/ppt/presentation.xml" ContentType="application/vnd.openxmlformats-officedocument.presentationml.presentation.main+xml"/>
  <Override PartName="/ppt/slideMasters/slideMaster1.xml" ContentType="application/vnd.openxmlformats-officedocument.presentationml.slideMaster+xml"/>
  <Override PartName="/ppt/slideLayouts/slideLayout1.xml" ContentType="application/vnd.openxmlformats-officedocument.presentationml.slideLayout+xml"/>
  <Override PartName="/ppt/theme/theme1.xml" ContentType="application/vnd.openxmlformats-officedocument.theme+xml"/>
  <Override PartName="/docProps/core.xml" ContentType="application/vnd.openxmlformats-package.core-properties+xml"/>
  <Override PartName="/docProps/app.xml" ContentType="application/vnd.openxmlformats-officedocument.extended-properties+xml"/>
  {slides}
</Types>"""

THEME = """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<a:theme xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main" name="Brief">
  <a:themeElements>
    <a:clrScheme name="Brief">
      <a:dk1><a:srgbClr val="1C2834"/></a:dk1>
      <a:lt1><a:srgbClr val="F4F1EB"/></a:lt1>
      <a:dk2><a:srgbClr val="14324F"/></a:dk2>
      <a:lt2><a:srgbClr val="FFFFFF"/></a:lt2>
      <a:accent1><a:srgbClr val="1F6B54"/></a:accent1>
      <a:accent2><a:srgbClr val="C4622D"/></a:accent2>
      <a:accent3><a:srgbClr val="F3E6C8"/></a:accent3>
      <a:accent4><a:srgbClr val="5E6A75"/></a:accent4>
      <a:accent5><a:srgbClr val="E7F2ED"/></a:accent5>
      <a:accent6><a:srgbClr val="F8EDE6"/></a:accent6>
      <a:hlink><a:srgbClr val="1F4E79"/></a:hlink>
      <a:folHlink><a:srgbClr val="8A3E16"/></a:folHlink>
    </a:clrScheme>
    <a:fontScheme name="Brief">
      <a:majorFont>
        <a:latin typeface="Microsoft YaHei"/>
        <a:ea typeface="微软雅黑"/>
        <a:cs typeface="Microsoft YaHei"/>
      </a:majorFont>
      <a:minorFont>
        <a:latin typeface="Microsoft YaHei"/>
        <a:ea typeface="微软雅黑"/>
        <a:cs typeface="Microsoft YaHei"/>
      </a:minorFont>
    </a:fontScheme>
    <a:fmtScheme name="Brief">
      <a:fillStyleLst><a:solidFill><a:schemeClr val="phClr"/></a:solidFill><a:solidFill><a:schemeClr val="phClr"/></a:solidFill><a:solidFill><a:schemeClr val="phClr"/></a:solidFill></a:fillStyleLst>
      <a:lnStyleLst><a:ln w="6350"><a:solidFill><a:schemeClr val="phClr"/></a:solidFill></a:ln><a:ln w="6350"><a:solidFill><a:schemeClr val="phClr"/></a:solidFill></a:ln><a:ln w="6350"><a:solidFill><a:schemeClr val="phClr"/></a:solidFill></a:ln></a:lnStyleLst>
      <a:effectStyleLst><a:effectStyle><a:effectLst/></a:effectStyle><a:effectStyle><a:effectLst/></a:effectStyle><a:effectStyle><a:effectLst/></a:effectStyle></a:effectStyleLst>
      <a:bgFillStyleLst><a:solidFill><a:schemeClr val="phClr"/></a:solidFill><a:solidFill><a:schemeClr val="phClr"/></a:solidFill><a:solidFill><a:schemeClr val="phClr"/></a:solidFill></a:bgFillStyleLst>
    </a:fmtScheme>
  </a:themeElements>
</a:theme>"""

MASTER = """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<p:sldMaster xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main"
 xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"
 xmlns:p="http://schemas.openxmlformats.org/presentationml/2006/main">
  <p:cSld name=""><p:spTree>
    <p:nvGrpSpPr><p:cNvPr id="1" name=""/><p:cNvGrpSpPr/><p:nvPr/></p:nvGrpSpPr>
    <p:grpSpPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="0" cy="0"/><a:chOff x="0" y="0"/><a:chExt cx="0" cy="0"/></a:xfrm></p:grpSpPr>
  </p:spTree></p:cSld>
  <p:clrMap bg1="lt1" tx1="dk1" bg2="lt2" tx2="dk2" accent1="accent1" accent2="accent2" accent3="accent3" accent4="accent4" accent5="accent5" accent6="accent6" hlink="hlink" folHlink="folHlink"/>
  <p:sldLayoutIdLst><p:sldLayoutId id="2147483649" r:id="rId1"/></p:sldLayoutIdLst>
</p:sldMaster>"""

LAYOUT = """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<p:sldLayout xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main"
 xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"
 xmlns:p="http://schemas.openxmlformats.org/presentationml/2006/main" type="blank" preserve="1">
  <p:cSld name="Blank"><p:spTree>
    <p:nvGrpSpPr><p:cNvPr id="1" name=""/><p:cNvGrpSpPr/><p:nvPr/></p:nvGrpSpPr>
    <p:grpSpPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="0" cy="0"/><a:chOff x="0" y="0"/><a:chExt cx="0" cy="0"/></a:xfrm></p:grpSpPr>
  </p:spTree></p:cSld>
  <p:clrMapOvr><a:masterClrMapping/></p:clrMapOvr>
</p:sldLayout>"""


def presentation():
    sld_ids = "".join(
        f'<p:sldId id="{256 + i}" r:id="rId{i + 2}"/>' for i in range(len(SLIDES))
    )
    return f"""<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<p:presentation xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main"
 xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"
 xmlns:p="http://schemas.openxmlformats.org/presentationml/2006/main" saveSubsetFonts="1">
  <p:sldMasterIdLst><p:sldMasterId id="2147483648" r:id="rId1"/></p:sldMasterIdLst>
  <p:sldIdLst>{sld_ids}</p:sldIdLst>
  <p:sldSz cx="{W}" cy="{H}" type="widescreen"/>
  <p:notesSz cx="6858000" cy="9144000"/>
</p:presentation>"""


def main():
    files = {
        "[Content_Types].xml": CONTENT_TYPES.format(
            slides="\n".join(
                f'<Override PartName="/ppt/slides/slide{i}.xml" ContentType="application/vnd.openxmlformats-officedocument.presentationml.slide+xml"/>'
                for i in range(1, len(SLIDES) + 1)
            )
        ),
        "_rels/.rels": """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="ppt/presentation.xml"/>
  <Relationship Id="rId2" Type="http://schemas.openxmlformats.org/package/2006/relationships/metadata/core-properties" Target="docProps/core.xml"/>
  <Relationship Id="rId3" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/extended-properties" Target="docProps/app.xml"/>
</Relationships>""",
        "docProps/core.xml": """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<cp:coreProperties xmlns:cp="http://schemas.openxmlformats.org/package/2006/metadata/core-properties"
 xmlns:dc="http://purl.org/dc/elements/1.1/">
  <dc:title>VR示教与VLA闭环例会汇报</dc:title>
  <dc:creator>x1 coffee sdk</dc:creator>
</cp:coreProperties>""",
        "docProps/app.xml": f"""<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Properties xmlns="http://schemas.openxmlformats.org/officeDocument/2006/extended-properties">
  <Slides>{len(SLIDES)}</Slides>
</Properties>""",
        "ppt/presentation.xml": presentation(),
        "ppt/_rels/presentation.xml.rels": """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/slideMaster" Target="slideMasters/slideMaster1.xml"/>
  """
        + "\n".join(
            f'<Relationship Id="rId{i + 2}" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/slide" Target="slides/slide{i + 1}.xml"/>'
            for i in range(len(SLIDES))
        )
        + "\n</Relationships>",
        "ppt/slideMasters/slideMaster1.xml": MASTER,
        "ppt/slideMasters/_rels/slideMaster1.xml.rels": """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/slideLayout" Target="../slideLayouts/slideLayout1.xml"/>
  <Relationship Id="rId2" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/theme" Target="../theme/theme1.xml"/>
</Relationships>""",
        "ppt/slideLayouts/slideLayout1.xml": LAYOUT,
        "ppt/slideLayouts/_rels/slideLayout1.xml.rels": """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/slideMaster" Target="../slideMasters/slideMaster1.xml"/>
</Relationships>""",
        "ppt/theme/theme1.xml": THEME,
    }
    for i, builder in enumerate(SLIDES, start=1):
        files[f"ppt/slides/slide{i}.xml"] = builder()
        files[f"ppt/slides/_rels/slide{i}.xml.rels"] = """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/slideLayout" Target="../slideLayouts/slideLayout1.xml"/>
</Relationships>"""
    OUT.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(OUT, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        for name, content in files.items():
            zf.writestr(name, content.encode("utf-8"))
    print(OUT)


if __name__ == "__main__":
    main()
