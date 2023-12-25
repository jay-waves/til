### zsh
- starship
- ohmyzsh

.zshrc:
```bash
# ---------- user alias -----------------------
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'
alias bat='batcat'

# Add an "alert" alias for long running commands.  Use like so: `sleep 10; alert`
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'

# ------------- command and prompt --------------------------

# zoxide
eval "$(zoxide init zsh)"

# starship
eval "$(starship init zsh)"

# ohmyzsh plugins
# git completion, auto suggesitons, syntax hightlight
plugins=(git starship timer zsh-autosuggestions zsh-syntax-highlighting)

# ------------- network --------------------------

# global proxy
host_ip=$(cat /etc/resolv.conf|grep nameserver|awk '{print $2}')
export all_proxy="http://$host_ip:7890"
export https_proxy="http://$host_ip:7890"
export http_proxy="http://$host_ip:7890"
export ALL_PROXY="http://$host_ip:7890"
export HTTPS_PROXY="http://$host_ip:7890"
export HTTP_PROXY="http://$host_ip:7890"

# software proxy
git config --global http.proxy "socks5://$host_ip:7890"
git config --global http.proxy "socks5://$host_ip:7890"


# ------------- compiler and lsp --------------------------

# language server
#. "$HOME/.cargo/env"

# compiler
export MAKEFLAGS="-j4"


# ---------- user software settings -----------------------

# add ./local to path
export PATH=$HOME/bin:/usr/local/bin:$HOME/.local/bin:$PATH

# helix
export HELIX_RUNTIME=/home/dirge/bin/helix-2305/runtime

# git config:
export GITHUB_TOKEN="..."
git config --global credential.helper 'cache --timeout=3600'
git config --global user.name "luminous-whispers"
git config --global user.email "2291303762@qq.com"

# ros env:
# source /opt/ros/humble/setup.bash
# source ~/code/ros_ws/install/setup.bash

# ---------- user cmd history -----------------------

# don't put duplicate lines or lines starting with space in the history.
setopt histignorespace
setopt histignoredups

# append to the history file, don't overwrite it
setopt append_history

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=2000
HISTFILESIZE=2000

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# set variable identifying the chroot you work in (used in the prompt below)
if [ -z "${debian_chroot:-}" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

# ---------- color and theme -----------------------

# don't put duplicate lines or lines starting with space in the history.
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
# theme, use starship instead of ohmyzsh
# ZSH_THEME="robbyrussell"

# colored GCC warnings and errors
export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'

# color support
# export COLORTERM=truecolor
# export TERM="xterm-256color"

# ---------- ohmyzsh configuration -----------------------

export ZSH="$HOME/.oh-my-zsh" # ohmyzsh path
CASE_SENSITIVE="true"
COMPLETION_WAITING_DOTS="%F{red}waiting...%f"
zstyle ':omz:update' mode disabled  # disable automatic updates
source $ZSH/oh-my-zsh.sh
```