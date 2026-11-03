// main/app_font.h —— 应用专用 16px 中文字体。
//
// 字体由 assets/fonts/app_font_16.c 提供:Source Han Sans SC Normal(OFL 1.1)的
// 静态子集,覆盖 ASCII 可打印字符与应用全部中文界面文案(174 个码点)。
// 生成方式与授权记录见 assets/fonts/README.md;不要手改生成文件。
#pragma once

#include "lvgl.h"

LV_FONT_DECLARE(app_font_16);
