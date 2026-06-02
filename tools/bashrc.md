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
