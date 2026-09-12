#!/bin/sh
# 新设备先在仓库根目录运行（同名文件远端优先）：
# rclone bisync ali-oss:yay-waves/til/ ./assets --workdir .oss-assets/bisync --resync --resync-mode path1 --compare size,modtime --fast-list
set -eu
repo=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd -P)
[ -d "$repo/assets" ] && [ ! -L "$repo/assets" ] || { echo 'assets must be an ordinary local directory.' >&2; exit 1; }
if command -v mountpoint >/dev/null 2>&1 && mountpoint -q "$repo/assets"; then
    echo 'Unmount assets first.' >&2; exit 1
fi
git -C "$repo" add -A
result=0
git -C "$repo" diff --cached --quiet || result=$?
case "$result" in
    0) ;;
    1) git -C "$repo" commit -m "$(date +%y%m%d)" ;;
    *) exit "$result" ;;
esac
git -C "$repo" push
rclone bisync 'ali-oss:yay-waves/til/' "$repo/assets" --workdir "$repo/.oss-assets/bisync" --compare size,modtime --fast-list --verbose
