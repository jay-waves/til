## 增强

* startship  
* zoxide 模糊跳转
* fzf 模糊搜索，集成到 `ctrl+R`
* tldr 

## 键映射

配置键映射: 
```bash
# in .zshrc
# 将 CapsLock 和 Escape 互换
xmodmap -e "keycode 9 = Caps_Lock"
xmodmap -e "keycode 66 = Escape"
``` 

## 配置网路代理

`~/.bashrc`:

```bash
proxy="http://127.0.0.1:7890" 
export all_proxy=$proxy
export https_proxy=$proxy
export http_proxy=$proxy
export ALL_PROXY=$proxy
export HTTPS_PROXY=$proxy
export HTTP_PROXY=$proxy
```

配置部分软件的代理

```bash
# software proxy
git config --global http.proxy "socks5://$host_ip:7890"
git config --global https.proxy "socks5://$host_ip:7890"
```

## 给命令上色

```sh
green='\e[32m'
blue='\e[34m'
clear='\e[0m'

color_green(){
	echo -ne $green$1$clear
}

color_blue(){
	echo -ne $blue$1$clear
}
```

## 起别名

Linux 有四种可执行命令:
- bin
- shell builtins 
- shell functions 
- alias

### 给命令起别名

Run `nano ~/.bash_profile` and add the following line:

```bash
alias dockerlogin='ssh www-data@adnan.local -p2222'  # add your alias in .bash_profile
```

`$ alias new_expr=old_instru_expr`

```bash
e.g.
$ alias lm='ls -al | more'
```

`alias`: 列出当前所有 alias

`unalias old_instru_alias`: 取消某 alias

### 给路径起别名

```bash
export hot_path = '/xxx/xxx/xxx/very/long/and/sophisticated/path'

cd $hot_path
```