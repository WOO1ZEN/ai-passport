<p align="right">
  <a href="README.zh_CN.md">简体中文</a> · <strong>English</strong>
</p>

# Application font: Source Han Sans SC 16 px subset

`app_font_16.c` is the single 16 px UI font used by the BLE signal scanner
application. It covers printable ASCII plus the exact set of Chinese characters
the application can display, so every label has a real glyph.

## Source and license

| Item | Value |
| --- | --- |
| Family | Source Han Sans SC Normal (Source Han Sans / Noto Sans CJK, same design) |
| Weight | Normal (regular) |
| License | SIL Open Font License 1.1 (OFL); license text at `managed_components/lvgl__lvgl/scripts/built_in_font/font_license/SourceHanSansSC/LICENSE.txt` |
| Source file | `managed_components/lvgl__lvgl/scripts/built_in_font/SourceHanSansSC-Normal.otf` — shipped by the pinned `lvgl/lvgl` 9.5.0 dependency, so no separate download is needed |
| Redistribution | The OFL permits embedding a rasterized subset. The 16 MB source OTF is **not** committed to this repository. |

## Generated artifact

| Item | Value |
| --- | --- |
| Generated file | `assets/fonts/app_font_16.c` (do not edit by hand) |
| Symbol | `const lv_font_t app_font_16` (declared in `main/app_font.h`) |
| Size / bpp / format | 16 px, 4 bpp, `LV_FONT_FMT_TXT_PLAIN`, no compression, no kerning |
| Line height / base line | 20 px / 5 px (identical to LVGL's built-in Source Han Sans SC fonts) |
| Coverage | printable ASCII U+0020–U+007E + 117 CJK code points listed in `app_font_16.chars` (212 glyphs total, 213 `glyph_dsc` entries including the reserved id 0) |
| Flash cost | Generated C source is ~104 KB; the compiled font data is ~15 KB (glyph tables + bitmaps) and is `const`, so it lives in Flash |
| RAM cost | 0 bytes of static RAM; LVGL reads glyph bitmaps directly from Flash |

## Character inventory

`app_font_16.chars` lists every non-ASCII code point the UI may display, one
character per line. ASCII is always included as U+0020–U+007E. When a UI string
changes, update that file, regenerate, and rerun `tests/test_app_font_coverage.py`
(part of `./tools/validate.sh --static`), which fails if any inventory character
or ASCII code point is missing from the generated font.

Dynamic text (BLE device names) is not covered by this fixed inventory; the UI
replaces any character whose glyph is missing with `?` instead of drawing
LVGL placeholder boxes. See the "Dynamic content" note in
`docs/development/engineering/lvgl-chinese-fonts.md`.

## Regeneration

```bash
./tools/gen_app_font.sh
```

The script needs `npx` and uses a pinned converter, `lv_font_conv@1.5.3`, run
under Node.js 18:

```bash
npx -y -p node@18 -p lv_font_conv@1.5.3 lv_font_conv \
  --font managed_components/lvgl__lvgl/scripts/built_in_font/SourceHanSansSC-Normal.otf \
  --symbols "<printable ASCII + inventory>" \
  --size 16 --bpp 4 --no-compress --no-kerning \
  --format lvgl --lv-font-name app_font_16 --lv-include lvgl.h \
  --output assets/fonts/app_font_16.c
```

Node.js 18 is required, not incidental: newer Node.js runtimes have produced
corrupt PLAIN bitmap output with this converter. See
[the CJK subset experience note](../../docs/reference/shinku-chen/lvgl-cjk-font-subsets.md).
The generated file was additionally verified by re-parsing its cmap/glyph tables
with LVGL 9.5 PLAIN 4 bpp semantics and rendering a preview, so the committed
artifact is checked rather than trusted.
