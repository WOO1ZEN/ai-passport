<p align="right">
  <strong>简体中文</strong> · <a href="README.md">English</a>
</p>

# 应用字体:思源黑体 16px 子集

`app_font_16.c` 是 BLE 信号扫描应用唯一使用的 16px 界面字体。它覆盖可打印
ASCII 与应用可能显示的全部中文字符,因此每个界面标签都有真实字形,不会出现
缺字方块。

## 来源与授权

| 项目 | 值 |
| --- | --- |
| 字体家族 | Source Han Sans SC Normal(思源黑体简体中文常规体,与 Noto Sans CJK 同源设计) |
| 字重 | Normal(常规) |
| 授权 | SIL Open Font License 1.1(OFL);许可全文位于 `managed_components/lvgl__lvgl/scripts/built_in_font/font_license/SourceHanSansSC/LICENSE.txt` |
| 源文件 | `managed_components/lvgl__lvgl/scripts/built_in_font/SourceHanSansSC-Normal.otf` —— 由锁定的 `lvgl/lvgl` 9.5.0 依赖自带,无需额外下载 |
| 再分发 | OFL 允许嵌入栅格化子集;16 MB 的源 OTF **不** 提交进本仓库 |

## 生成产物

| 项目 | 值 |
| --- | --- |
| 生成文件 | `assets/fonts/app_font_16.c`(不要手改) |
| 符号 | `const lv_font_t app_font_16`(声明在 `main/app_font.h`) |
| 字号 / 位深 / 格式 | 16 px、4 bpp、`LV_FONT_FMT_TXT_PLAIN`、不压缩、不做字距调整 |
| 行高 / 基线 | 20 px / 5 px(与 LVGL 内置思源黑体字体一致) |
| 覆盖范围 | 可打印 ASCII U+0020–U+007E,加上 `app_font_16.chars` 列出的 117 个中文字符(共 212 个字形、213 条 `glyph_dsc`,含保留的 0 号条目) |
| Flash 占用 | 生成的 C 源文件约 104 KB;编译进固件的数据约 15 KB(字体表 + 字形位图),数据为 `const`,常驻 Flash |
| RAM 占用 | 0 字节静态 RAM;LVGL 直接从 Flash 读取字形位图 |

## 字符清单

`app_font_16.chars` 逐行列出界面可能显示的全部非 ASCII 字符;ASCII 固定包含
U+0020–U+007E。界面文案变化时先更新该文件,再重新生成字体,最后运行
`tests/test_app_font_coverage.py`(`./tools/validate.sh --static` 的一部分):
只要任一清单字符或 ASCII 码点缺失,该检查就会失败。

动态文本(BLE 设备名)不受这份固定清单约束:界面会把字体里没有字形的字符
替换成 `?`,而不是让 LVGL 画占位方块。相关约定见
`docs/development/engineering/lvgl-chinese-fonts.md` 的"动态内容"一节。

## 重新生成

```bash
./tools/gen_app_font.sh
```

脚本依赖 `npx`,使用固定版本的转换器 `lv_font_conv@1.5.3`,并固定在 Node.js 18
下运行:

```bash
npx -y -p node@18 -p lv_font_conv@1.5.3 lv_font_conv \
  --font managed_components/lvgl__lvgl/scripts/built_in_font/SourceHanSansSC-Normal.otf \
  --symbols "<可打印 ASCII + 字符清单>" \
  --size 16 --bpp 4 --no-compress --no-kerning \
  --format lvgl --lv-font-name app_font_16 --lv-include lvgl.h \
  --output assets/fonts/app_font_16.c
```

指定 Node.js 18 不是随意的:较新的 Node.js 运行时会让该转换器写出损坏的
PLAIN 位图。参见
[CJK 子集经验记录](../../docs/reference/shinku-chen/lvgl-cjk-font-subsets.zh_CN.md)。
提交前还按 LVGL 9.5 的 PLAIN 4bpp 语义重新解析了 cmap 与字形表并渲染了预览,
即这份产物是"校验过的",而不是"假设正确的"。
