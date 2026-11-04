#!/usr/bin/env python3
"""第一周例会说明：SDK 框架、与 OpenArm 的局限、给香农的初步对接方向。"""

import html
import zipfile
from pathlib import Path

OUT = Path("/home/szxn/x1_coffee_robot_sdk/SDK框架熟悉与香农对接_初稿.docx")

NAVY = "14324F"
INK = "1C2834"
MUTED = "5E6A75"
RULE = "D9D2C5"
HEAD_BG = "14324F"
ALT = "F7F4EE"


def esc(text: str) -> str:
    return html.escape(text, quote=False)


def run(text, size=24, bold=False, color=INK, font="微软雅黑"):
    b = "<w:b/>" if bold else ""
    return (
        f'<w:r><w:rPr>{b}'
        f'<w:rFonts w:ascii="Microsoft YaHei" w:hAnsi="Microsoft YaHei" '
        f'w:eastAsia="{font}" w:cs="Microsoft YaHei"/>'
        f'<w:color w:val="{color}"/>'
        f'<w:sz w:val="{size}"/><w:szCs w:val="{size}"/>'
        f"</w:rPr><w:t xml:space=\"preserve\">{esc(text)}</w:t></w:r>"
    )


def paragraph(text, style="Body", size=24, bold=False, color=INK, before=0, after=120, center=False):
    jc = '<w:jc w:val="center"/>' if center else ""
    return (
        f'<w:p><w:pPr><w:pStyle w:val="{style}"/>'
        f'<w:spacing w:before="{before}" w:after="{after}" w:line="276" w:lineRule="auto"/>'
        f"{jc}</w:pPr>{run(text, size, bold, color)}</w:p>"
    )


def rich_paragraph(runs, style="Body", before=0, after=120):
    body = "".join(run(*item) if len(item) == 4 else run(*item) for item in runs)
    return (
        f'<w:p><w:pPr><w:pStyle w:val="{style}"/>'
        f'<w:spacing w:before="{before}" w:after="{after}" w:line="276" w:lineRule="auto"/>'
        f"</w:pPr>{body}</w:p>"
    )


def cell(text, width, fill, color, bold=False, size=21):
    return f"""<w:tc>
      <w:tcPr>
        <w:tcW w:w="{width}" w:type="dxa"/>
        <w:shd w:val="clear" w:color="auto" w:fill="{fill}"/>
        <w:vAlign w:val="center"/>
        <w:tcMar>
          <w:top w:w="70" w:type="dxa"/>
          <w:left w:w="100" w:type="dxa"/>
          <w:bottom w:w="70" w:type="dxa"/>
          <w:right w:w="100" w:type="dxa"/>
        </w:tcMar>
      </w:tcPr>
      <w:p>
        <w:pPr><w:spacing w:before="0" w:after="0" w:line="240" w:lineRule="auto"/></w:pPr>
        {run(text, size, bold, color)}
      </w:p>
    </w:tc>"""


def table(rows):
    widths = (1680, 3900, 4058)
    total = sum(widths)
    grid = "".join(f'<w:gridCol w:w="{w}"/>' for w in widths)
    trs = []
    for i, row in enumerate(rows):
        if i == 0:
            fills = (HEAD_BG, HEAD_BG, HEAD_BG)
            colors = ("FFFFFF", "FFFFFF", "FFFFFF")
            bold = True
        else:
            bg = "FFFFFF" if i % 2 else ALT
            fills = (bg, bg, bg)
            colors = (INK, INK, INK)
            bold = False
        tcs = "".join(
            cell(text, widths[c], fills[c], colors[c], bold if c == 0 or i == 0 else False)
            for c, text in enumerate(row)
        )
        trs.append(f"<w:tr>{tcs}</w:tr>")
    return f"""<w:tbl>
      <w:tblPr>
        <w:tblW w:w="{total}" w:type="dxa"/>
        <w:tblLayout w:type="fixed"/>
        <w:tblBorders>
          <w:top w:val="single" w:sz="4" w:space="0" w:color="{RULE}"/>
          <w:left w:val="single" w:sz="4" w:space="0" w:color="{RULE}"/>
          <w:bottom w:val="single" w:sz="4" w:space="0" w:color="{RULE}"/>
          <w:right w:val="single" w:sz="4" w:space="0" w:color="{RULE}"/>
          <w:insideH w:val="single" w:sz="4" w:space="0" w:color="{RULE}"/>
          <w:insideV w:val="single" w:sz="4" w:space="0" w:color="{RULE}"/>
        </w:tblBorders>
      </w:tblPr>
      <w:tblGrid>{grid}</w:tblGrid>
      {''.join(trs)}
    </w:tbl>
    {paragraph("", after=160)}"""


BODY = [
    paragraph("当前 SDK 控制框架与香农对接需求（初稿）", "Title", 36, True, NAVY, before=0, after=40),
    paragraph("第一周阅读记录  ·  2026.10.30 例会", "Sub", 21, False, MUTED, after=200),
    paragraph(
        "本周先把现有双臂 SDK 的控制链路读通，并对照 OpenArm 的 ROS 2 / MoveIt 2 教程，看后面做 VR 示教采集和 VLA 闭环时，限制在什么地方。没有改控制代码，也没有在真机上新发运动指令。下面是读代码、教程和已有固高、eRob 资料后的理解。",
        after=80,
    ),
    paragraph(
        "给香农的部分只是方向性初稿，用来会上先对齐。指令周期、允许的超前时间、力矩单位这些接口细节，还要继续查资料，补到能直接对接的程度。",
        after=200,
    ),
    paragraph("一、两边的控制框架", "H1", 30, True, NAVY, before=200, after=80),
    paragraph("两边上面几层都是 MoveIt 规划、ros2_control 执行轨迹。差别在硬件插件下面：OpenArm 每个周期把位置直接送给关节驱动器；当前 SDK 把轨迹交给固高卡，由卡按表执行。教程里还有应用层和机械本体，下面只对和运动有关的四层。", after=80),
    paragraph("1. OpenArm 教程里的框架", "H2", 24, True, NAVY, before=140, after=40),
    paragraph("规划层是 MoveIt 2。它做路径规划和碰撞检测，通过 FollowJointTrajectory 把轨迹交给控制器。左右臂可以一起规划，再拆开下发。本体是两条 7 轴臂，没有腰。"),
    paragraph("控制层是 ros2_control。左右臂各有一个 JointTrajectoryController，另外还有夹爪控制器。控制器按周期把关节位置写到命令接口。"),
    paragraph("硬件插件在真机上是 OpenArm_v10HW。每个控制周期做一次读写，把位置命令打成 CAN-FD 帧。左右臂各走一条 CAN-FD 总线。"),
    paragraph("再往下是关节驱动器自己的电流环和位置跟随。上位机这一拍写入的位置，驱动器就跟这一拍。中间没有另一块运动控制卡把未来一段轨迹存下来。", after=40),
    paragraph("教程里的真机部分，确认的是硬件插件能加载、CAN-FD 能配起来。下面拿来对比的是这套架构，不是我们在 OpenArm 真机上做过的实验。"),
    paragraph("2. 当前 SDK 的框架", "H2", 24, True, NAVY, before=160, after=40),
    paragraph("规划层同样是 MoveIt 2，也通过 FollowJointTrajectory 下发。腰、左臂、右臂可以分开规划。"),
    paragraph("控制层是 ros2_control，controller_manager 以 100 Hz 运行。腰、左臂、右臂各有一个 JointTrajectoryController。指令接口只有关节位置，没有力矩。"),
    paragraph("硬件插件把关节弧度换成脉冲，15 个轴汇入同一个接口。运动没有使能时，写进来的指令不会发到总线上。"),
    paragraph("执行层是固高 GTN 卡，作为 EtherCAT 主站，经厂商库 libgxn 驱动。轴 1 是腰，轴 2 到 8 是左臂，轴 9 到 15 是右臂。让轴按时间运动的，是卡内的 PT/PVT。"),
    paragraph("仓库里还有一层 OpenArm 兼容接口。它是 ROS 侧的位置指令流和关节命名对照，不是教程里那套 CAN 硬件。", after=120),
    paragraph("3. 对在一起看", "H2", 24, True, NAVY, before=80, after=80),
    table([
        ("层次", "OpenArm 教程", "当前 SDK"),
        ("规划", "MoveIt 2，FollowJointTrajectory。两条 7 轴臂", "MoveIt 2，同一类接口。腰加两条 7 轴臂，共 15 轴"),
        ("控制", "ros2_control。左右臂轨迹控制器，按周期写位置", "ros2_control，100 Hz。腰、左、右三个轨迹控制器，只有位置"),
        ("硬件插件", "OpenArm_v10HW，每周期把位置打成 CAN-FD", "自研插件，弧度换成脉冲。未使能时不下总线"),
        ("执行", "驱动器直接跟本周期的位置命令", "固高 GTN 做 EtherCAT 主站，用 PT/PVT 执行已提交的轨迹"),
    ]),
    paragraph("二、当前框架的局限", "H1", 30, True, NAVY, before=200, after=80),
    paragraph("上一节的差别落到使用上，就是执行层不能改已经交出去的轨迹。当前 SDK 把一段轨迹交给固高卡，由卡内 PT 队列或 PVT 表按时间执行。点一旦写进去，还没走完的部分不能改。"),
    paragraph("当前动态 PT 的配置是周期 10 ms、预缓冲 8 个点，大约 80 ms 的运动在下发时就已经定下来了。"),
    paragraph("周期数据里目前也没有力矩。现有 EtherCAT 过程数据映射了目标位置、控制字、实际位置和状态字，没有把运行模式和力矩映射进来。这套 SDK 没有做力矩环。固高手册里有周期同步力矩（CST）这类接口，当前程序没有接到那条路上。控制卡本身不带阻抗或柔顺模型，柔顺需要在上位机计算，再由执行层接住。"),
    paragraph("10 月 27 日的记录里，键盘点动用的 PVT 在真机上确认过。MoveIt 通过动态 PT 执行，还没有确认手臂真正跟上。和香农对实时跟随时，这一点需要单独看。", after=160),
    table([
        ("能力", "OpenArm 教程中的做法", "当前 SDK + 固高 GTN"),
        ("实时跟随", "每个周期下发位置，只保留最新目标", "轨迹写入控制卡，按 PT/PVT 表执行"),
        ("中途停止", "目标撤销后，关节立即停止", "队列里已缓存的点仍会走完"),
        ("拖动示教", "可以切到柔顺，外力推得动关节", "位置环锁住，外力拖不动"),
        ("接触让位", "力矩或阻抗可以减小接触力", "继续跟踪位置，接触时顶住"),
        ("力矩回读", "驱动器按周期回传力矩或电流", "现有过程数据没有力矩，周期内读不到"),
        ("策略闭环", "新的推理结果可以覆盖还没执行的动作", "已入队的点不能改，旧动作会走完"),
    ]),
    paragraph("三、这些限制会碰到的使用场景", "H1", 30, True, NAVY, before=80, after=80),
    paragraph("1. VR 遥操作", "H2", 24, True, NAVY, before=120, after=40),
    paragraph("采集时手柄连续给出目标。操作者松开手柄后，希望手臂马上停住。按现在的队列，已经交给卡的点还会再走一段。要立刻停，只能走急停，这一次采集就中断了。"),
    paragraph("2. 拖动示教和轻接触", "H2", 24, True, NAVY, before=140, after=40),
    paragraph("示教前需要把腕部拖到杯架附近，或者手臂轻轻碰到杯子时能让一下。现在关节锁在位置环上，拖不动；前面有东西挡着时，仍往目标位置顶。顶了多大力，现有周期数据里记不下来。"),
    paragraph("3. 后续的 VLA 闭环", "H2", 24, True, NAVY, before=140, after=40),
    paragraph("VLA（视觉-语言-动作）在上位机推理，不在控制卡上跑。策略输出当前关节目标，或者未来零点几秒的一段动作，再发给执行层。下一次推理需要盖住还没执行的部分。"),
    paragraph("当前 PT/PVT 写入后不能改，大约 80 ms 的预缓冲已经定下来，新的动作盖不住旧的。力矩没有进周期数据，策略观测少这一维。单步超速或超出限位时，也没有“拒绝这一点、保持使能、这一回合继续”的处理，异常时更容易整段停掉。", after=200),
    paragraph("四、初步希望香农配合的方向", "H1", 30, True, NAVY, before=200, after=80),
    paragraph("下面四条是按现有代码和资料收拢的缺口，方便会上先对齐方向。还不是最终接口清单。正式对接前，需要把周期、超前时间、单位和异常行为写具体。"),
    paragraph("1. 实时跟随，并且能覆盖还没执行的动作", "H2", 24, True, NAVY, before=140, after=40),
    paragraph("希望执行侧能按大约 100 Hz 跟踪最新位置，也能按推理周期接收一段短动作。新的结果盖住未执行部分，旧点不要继续走完。只动一条臂时，另一条臂和腰保持不动。"),
    paragraph("2. 柔顺，接触时能够让开", "H2", 24, True, NAVY, before=140, after=40),
    paragraph("希望有可以在线切换的柔顺模式：外力能拖动，松手后不要因为重力掉下去。碰到物体时按给定的力矩或阻抗响应，不要死死顶在位置目标上。两种模式切换时，不需要重启整套控制程序。柔顺打开时，卡内原来的 PT/PVT 自动插补需要停掉，避免和上位机指令叠在一起。"),
    paragraph("3. 按周期把状态和力矩读回来", "H2", 24, True, NAVY, before=140, after=40),
    paragraph("希望不低于 100 Hz 读到关节位置、速度、力矩或等效电流、报警，并带时间戳，后面 VLA 的观测要用。如果腕部没有力传感器，只能用关节电流估计，需要说明估计方式和精度，并给出单位和方向。"),
    paragraph("4. 这几种用法下，安全仍然成立", "H2", 24, True, NAVY, before=140, after=40),
    paragraph("柔顺模式下，急停和抱闸仍然有效。上位机和控制卡通信断开时，关节应自行停止。单步超出速度或限位时，希望拒绝该点并报警，不要把这个点执行掉，也不要因此把整个回合停掉。", after=200),
    paragraph("五、需求写细之后，可以用来对照的检查", "H1", 30, True, NAVY, before=200, after=80),
    paragraph("这些不是本周要香农承诺的验收单。需求写细、有了真机小例子之后，可以用下面几条看是否做通：", after=60),
    paragraph("·  15 轴能够上电，模式和过程数据配置不被驱动器拒绝。", after=40),
    paragraph("·  大约 10 Hz 不断替换动作段，一段时间内手臂只执行最新的一段。", after=40),
    paragraph("·  柔顺模式下可以拖动右臂，松手后不自由下落。", after=40),
    paragraph("·  停止下发或单步超限时，不继续执行已经缓存的轨迹。", after=40),
    paragraph("·  轻接触固定物时，力矩读数上升，但不顶到驱动报警。", after=40),
    paragraph("·  位置、速度、力矩能连续回读；柔顺模式下急停仍然有效。", after=120),
    paragraph("如果能附一个真机可运行的小例子，并说明力矩的单位、方向，以及执行侧实际提前定下来的时间有多长，后面对接会清楚很多。", after=200),
    paragraph("六、接下来要补的内容", "H1", 30, True, NAVY, before=200, after=80),
    paragraph("本周只把框架和缺口对齐。要和香农正式对接，还需要继续查资料，把上面四条收成具体接口：", after=60),
    paragraph("·  固高手册里的 EtherCAT 过程数据、SDO，以及位置、速度、力矩这几种轴模式实际怎么切换。", after=40),
    paragraph("·  eRob 手册里，周期同步力矩对过程数据映射和数据长度的要求。", after=40),
    paragraph("·  香农客户资料里已经提供的运动接口、二次开发做到哪一层，以及哪些必须由香农改卡侧或驱动配置。", after=40),
    paragraph("·  对接表里写清：指令内容、周期、延迟或超前时间、单臂和腰的关系、单位和方向、异常时的行为、真机示例要演示什么。", after=120),
    paragraph("分工上，VR 头显和手柄、末端位姿换算成关节角、示教和策略的数据记录、VLA 推理和相机、ROS 2 上位机，由我方来做。需要和香农确认的是执行层：能不能实时跟随并覆盖旧动作，能不能柔顺，状态和力矩能不能读回来。", after=80),
]


STYLES = f"""<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<w:styles xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main">
  <w:docDefaults>
    <w:rPrDefault><w:rPr>
      <w:rFonts w:ascii="Microsoft YaHei" w:hAnsi="Microsoft YaHei" w:eastAsia="微软雅黑" w:cs="Microsoft YaHei"/>
      <w:sz w:val="24"/><w:szCs w:val="24"/>
      <w:color w:val="{INK}"/>
    </w:rPr></w:rPrDefault>
    <w:pPrDefault><w:pPr>
      <w:spacing w:after="120" w:line="276" w:lineRule="auto"/>
    </w:pPr></w:pPrDefault>
  </w:docDefaults>
  <w:style w:type="paragraph" w:default="1" w:styleId="Body">
    <w:name w:val="Body"/>
    <w:qFormat/>
  </w:style>
  <w:style w:type="paragraph" w:styleId="Title">
    <w:name w:val="Title"/>
    <w:basedOn w:val="Body"/>
  </w:style>
  <w:style w:type="paragraph" w:styleId="Sub">
    <w:name w:val="Subtitle"/>
    <w:basedOn w:val="Body"/>
  </w:style>
  <w:style w:type="paragraph" w:styleId="H1">
    <w:name w:val="heading 1"/>
    <w:basedOn w:val="Body"/>
  </w:style>
  <w:style w:type="paragraph" w:styleId="H2">
    <w:name w:val="heading 2"/>
    <w:basedOn w:val="Body"/>
  </w:style>
</w:styles>"""


DOCUMENT = f"""<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<w:document xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main"
 xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">
  <w:body>
    {''.join(BODY)}
    <w:sectPr>
      <w:footerReference w:type="default" r:id="rId2"/>
      <w:pgSz w:w="11906" w:h="16838"/>
      <w:pgMar w:top="1134" w:right="1134" w:bottom="1134" w:left="1134" w:header="567" w:footer="680"/>
      <w:cols w:space="425"/>
    </w:sectPr>
  </w:body>
</w:document>"""


FOOTER = f"""<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<w:ftr xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main">
  <w:p>
    <w:pPr>
      <w:pBdr><w:top w:val="single" w:sz="6" w:space="6" w:color="{RULE}"/></w:pBdr>
      <w:spacing w:before="60" w:after="0"/>
      <w:jc w:val="right"/>
    </w:pPr>
    {run("初稿  ·  第一周阅读记录，供例会对齐", 18, False, MUTED)}
  </w:p>
</w:ftr>"""


def main():
    files = {
        "[Content_Types].xml": """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
  <Default Extension="xml" ContentType="application/xml"/>
  <Override PartName="/word/document.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.document.main+xml"/>
  <Override PartName="/word/styles.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.styles+xml"/>
  <Override PartName="/word/footer1.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.footer+xml"/>
  <Override PartName="/docProps/core.xml" ContentType="application/vnd.openxmlformats-package.core-properties+xml"/>
  <Override PartName="/docProps/app.xml" ContentType="application/vnd.openxmlformats-officedocument.extended-properties+xml"/>
</Types>""",
        "_rels/.rels": """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="word/document.xml"/>
  <Relationship Id="rId2" Type="http://schemas.openxmlformats.org/package/2006/relationships/metadata/core-properties" Target="docProps/core.xml"/>
  <Relationship Id="rId3" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/extended-properties" Target="docProps/app.xml"/>
</Relationships>""",
        "docProps/core.xml": """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<cp:coreProperties xmlns:cp="http://schemas.openxmlformats.org/package/2006/metadata/core-properties"
 xmlns:dc="http://purl.org/dc/elements/1.1/">
  <dc:title>当前 SDK 控制框架与香农对接需求（初稿）</dc:title>
  <dc:creator>x1 coffee sdk</dc:creator>
</cp:coreProperties>""",
        "docProps/app.xml": """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Properties xmlns="http://schemas.openxmlformats.org/officeDocument/2006/extended-properties">
  <Application>x1 coffee sdk</Application>
</Properties>""",
        "word/document.xml": DOCUMENT,
        "word/styles.xml": STYLES,
        "word/footer1.xml": FOOTER,
        "word/_rels/document.xml.rels": """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles" Target="styles.xml"/>
  <Relationship Id="rId2" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/footer" Target="footer1.xml"/>
</Relationships>""",
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(OUT, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        for name, content in files.items():
            zf.writestr(name, content.encode("utf-8"))
    print(OUT)


if __name__ == "__main__":
    main()
