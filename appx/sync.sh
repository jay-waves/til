#!/bin/sh
# 新设备先在仓库根目录运行（同名文件远端优先）：
# rclone bisync ali-oss:yay-waves/til/ ./assets --resync --resync-mode path1 --compare size,modtime --fast-list
set -eu
repo=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd -P)
[ -d "$repo/assets" ] && [ ! -L "$repo/assets" ] || { echo 'assets must be an ordinary local directory.' >&2; exit 1; }
if command -v mountpoint >/dev/null 2>&1 && mountpoint -q "$repo/assets"; then
    echo 'Unmount assets first.' >&2; exit 1
fi
log="${TMPDIR:-/tmp}/rclone-bisync.$$.log"
trap 'rm -f "$log"' EXIT
rclone bisync 'ali-oss:yay-waves/til/' "$repo/assets" --compare size,modtime --fast-list --verbose >"$log" 2>&1 &
sync_pid=$!
git_result=0
(
    set -e
    git -C "$repo" add -A
    result=0
    git -C "$repo" diff --cached --quiet || result=$?
    case "$result" in
        0) ;;
        1) git -C "$repo" commit -m "$(date +%y%m%d)" ;;
        *) exit "$result" ;;
    esac
    git -C "$repo" push
) || git_result=$?
sync_result=0
wait "$sync_pid" || sync_result=$?
cat "$log"
[ "$git_result" -eq 0 ] || exit "$git_result"
exit "$sync_result"
