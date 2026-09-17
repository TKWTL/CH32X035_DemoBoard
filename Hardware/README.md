# 硬件

CH32X035 DemoBoard（CH32X035C8T6 USB-PD 受电端开发板，EPR 最高 28 V）的硬件设计文件。

## 目录结构

```text
Hardware/
├─ source/                        设计源工程（EasyEDA Pro / 嘉立创EDA专业版）
│  └─ CH32X035_DemoBoard.epro2
├─ Preview/                       预览与交换文件
│  ├─ Schematics_CH32X035_DemoBoard.pdf   原理图
│  ├─ PCB_CH32X035_DemoBoard.pdf           PCB 预览
│  └─ Netlist_CH32X035_DemoBoard.tel      网表
├─ Manufacturing/                 生产文件
│  ├─ Gerber/
│  │  ├─ Gerber_CH32X035_DemoBoard_二层版.zip
│  │  └─ Gerber_CH32X035_DemoBoard_四层版.zip
│  ├─ BOM/
│  │  ├─ BOM_CH32X035_DemoBoard.csv
│  │  └─ InteractiveBOM_CH32X035_DemoBoard.html   交互式 BOM（浏览器打开，点选定位）
│  └─ CPL/
│     └─ PickAndPlace_CH32X035_DemoBoard.csv
└─ README.md
```

## 打样 / 配单 / 贴片

1. **打样**：按需选择 **二层版** 或 **四层版** Gerber 包（含钻孔文件），解压后整体打包给板厂（嘉立创、捷配等）。
2. **配单**：按 `Manufacturing/BOM/BOM_CH32X035_DemoBoard.csv` 采购物料。
3. **贴片**：SMT 可直接使用 `Manufacturing/CPL/PickAndPlace_CH32X035_DemoBoard.csv`（贴片坐标/器件位号）。
4. **交互式 BOM**：浏览器打开 `Manufacturing/BOM/InteractiveBOM_CH32X035_DemoBoard.html`，可在 PCB 视图上点选/高亮元件与位号。

## 备注

- 焊接建议：先完成电源部分并验证，再焊主控与外设；Type-C 母座与芯片注意方向；上电前先确认无电源短路。
- 板卡照片与实机显示截图在 `../Images/`；引脚/跳线/连接器对照见 `../Firmware/README.md` 的「板级接口与引脚映射」。
- **36 V 挡位改动**：将 VBUS 保护 TVS **D2** 由 SMAJ28A 更换为 SMAJ36A，并将固件 EPR 上限宏 `PD_POLICY_EPR_MAX_FIXED_MV` 调至 `36000U`（详见仓库根 README「更新说明」）。
