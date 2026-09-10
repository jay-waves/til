#!/bin/sh
# POSIX sh; requires rclone, jq and a working FUSE installation.
set -eu
action=${1:-start}
case "$action" in start|status|refresh|warm|stop) ;; *) echo "Usage: sh $0 [start|status|refresh|warm|stop]" >&2; exit 2 ;; esac
repo=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd -P)
mount_dir=$repo/assets
state=$repo/.oss-assets/unix
remote=ali-oss:yay-waves/til/
umask 077
command -v rclone >/dev/null
command -v jq >/dev/null
mkdir -p "$state"

rc() { rclone rc --url http://127.0.0.1:5579 "$@"; }
status() {
    response=$(rc vfs/stats) || return 1
    printf '%s\n' "$response" | jq -e --arg fs "${remote%/}" 'if .fs == $fs then . else error("RC belongs to another remote") end'
}
refresh() {
    status >/dev/null
    response=$(rc vfs/refresh recursive=true) || return 1
    printf '%s\n' "$response" | jq -e '(.result | type == "object") and (.result | all(. == "OK"))' >/dev/null
}

if [ "$action" = start ]; then
    mkdir "$state/start.lock" || { echo 'Another start is in progress; check unix/start.lock.' >&2; exit 1; }
    trap 'rmdir "$state/start.lock"' 0
    trap 'exit 1' 1 2 15
    if [ ! -f "$state/rc-password" ]; then
        # Private random RC password; OSS credentials remain in rclone's own config.
        od -An -N32 -tx1 /dev/urandom | tr -d ' \n' > "$state/rc-password"
    fi
fi
RCLONE_RC_USER=til-assets
RCLONE_RC_PASS=$(cat "$state/rc-password")
export RCLONE_RC_USER RCLONE_RC_PASS

case "$action" in
    start)
        if ! status >/dev/null 2>&1; then
            [ ! -L "$mount_dir" ] || { echo 'assets is a symbolic link.' >&2; exit 1; }
            # rmdir fails for a nonempty directory or existing mount; never hide local files.
            if [ -d "$mount_dir" ]; then rmdir "$mount_dir"; fi
            mkdir "$mount_dir"
            rclone mount "$remote" "$mount_dir" \
                --vfs-cache-mode=full --vfs-cache-max-age=8760h \
                --dir-cache-time=720h --poll-interval=0 --vfs-fast-fingerprint \
                --contimeout=10s --timeout=30s \
                --rc --rc-addr=127.0.0.1:5579 \
                --log-level=NOTICE --log-file-max-size=10M --log-file-max-backups=2 \
                --cache-dir "$state/cache" --log-file "$state/mount.log" --daemon
            refresh
        fi
        printf 'Mounted: %s\n' "$mount_dir"
        ;;
    status) status ;;
    refresh) refresh; echo 'Directory refreshed.' ;;
    warm)
        refresh
        # Read every byte through the mount; per-file failures propagate through find.
        find "$mount_dir" -type f -exec sh -c 'for file do cat "$file" > /dev/null || exit 1; done' sh {} +
        echo 'All files cached.'
        ;;
    stop)
        response=$(status) || exit 1
        printf '%s\n' "$response" | jq -e '.diskCache.uploadsQueued == 0 and .diskCache.uploadsInProgress == 0 and .inUse <= 1' >/dev/null || {
            echo 'Mount is busy or unavailable; close files and wait for uploads before stopping.' >&2; exit 1;
        }
        rc core/quit >/dev/null
        echo 'Mount shutdown requested; cache retained.'
        ;;
esac
