#!/usr/bin/env bash
# Regenerate the application's 16 px Chinese font subset.
#
# Input : managed_components/lvgl__lvgl/scripts/built_in_font/SourceHanSansSC-Normal.otf
#         (Source Han Sans SC, SIL OFL 1.1, shipped by the pinned lvgl/lvgl 9.5.0 dependency)
#         assets/fonts/app_font_16.chars  (application CJK inventory, one char per line)
# Output: assets/fonts/app_font_16.c      (symbol: app_font_16)
#
# Node.js 18 is required: newer runtimes have produced corrupt PLAIN bitmap output
# with lv_font_conv. See docs/reference/shinku-chen/lvgl-cjk-font-subsets.md.
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
font="${repo_root}/managed_components/lvgl__lvgl/scripts/built_in_font/SourceHanSansSC-Normal.otf"
chars_file="${repo_root}/assets/fonts/app_font_16.chars"
output="${repo_root}/assets/fonts/app_font_16.c"

if [[ ! -f "${font}" ]]; then
    echo "ERROR: ${font} not found." >&2
    echo "Run './tools/validate.sh --firmware' (or 'idf.py build') once so the pinned" >&2
    echo "lvgl/lvgl component is downloaded, then rerun this script." >&2
    exit 1
fi

symbols="$(python3 - "${chars_file}" <<'PY'
import sys
chars = [line.strip() for line in open(sys.argv[1], encoding="utf-8") if line.strip()]
print("".join(chr(cp) for cp in range(0x20, 0x7F)) + "".join(chars), end="")
PY
)"

npx -y -p node@18 -p lv_font_conv@1.5.3 lv_font_conv \
    --font "${font}" \
    --symbols "${symbols}" \
    --size 16 --bpp 4 --no-compress --no-kerning \
    --format lvgl --lv-font-name app_font_16 --lv-include lvgl.h \
    --output "${output}"

echo "Generated ${output}"
echo "Verify coverage with: python3 tests/test_app_font_coverage.py"
