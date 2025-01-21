# minimal bashrc for Ubuntu
# don't put duplicate lines or lines starting with space in the history.
setopt histignorespace
setopt histignoredups

# append to the history file, don't overwrite it
setopt append_history

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=1000
HISTFILESIZE=2000

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# set variable identifying the chroot you work in (used in the prompt below)
if [ -z "${debian_chroot:-}" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    #alias dir='dir --color=auto'
    #alias vdir='vdir --color=auto'
    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# colored GCC warnings and errors
export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'

# some more ls aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

# Add an "alert" alias for long running commands.  Use like so:
#   sleep 10; alert
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'


# ------------- command and prompt --------------------------
# color support
# export COLORTERM=truecolor
# export TERM="xterm-256color

# prompt customize for bash: bash-it or starship
eval "$(starship init zsh)"

# command syntax highlight
source /usr/share/zsh-syntax-highlighting/zsh-syntax-highlighting.zsh

# ------------- network --------------------------

# global proxy, using clash on port 7890.
host_ip=$(cat /etc/resolv.conf|grep nameserver|awk '{print $2}')
clash_port=7890
proxy="http://$host_ip:$clash_port"
export all_proxy=$proxy
export https_proxy=$proxy
export http_proxy=$proxy
export ALL_PROXY=$proxy
export HTTPS_PROXY=$proxy
export HTTP_PROXY=$proxy

# software proxy
git config --global http.proxy "socks5://$host_ip:$clash_port"
git config --global http.proxy "socks5://$host_ip:$clash_port"

# ------------- compiler and lsp --------------------------

# language server
#. "$HOME/.cargo/env"

# compiler
export MAKEFLAGS="-j4"
export CC=/usr/bin/clang
export CXX=/usr/bin/clang++

# ---------- user software settings -----------------------

# add ./local to path
export PATH="$HOME/.local/bin:$PATH"

# keyboard map: CapsLock <-> Escape
# xmodmap -e "keycode 9 = Caps_Lock"
# xmodmap -e "keycode 66 = Escape"

# helix
# export HELIX_RUNTIME=/home/dirge/bin/helix-2305/runtime

# git config
export GIT_TOKEN="..."
git config --global credential.helper 'cache --timeout=3600'
git config --global user.name "jay-waves"
git config --global user.email "yujiawei@buaa.edu.cn"
