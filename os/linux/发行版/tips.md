
## 常见开发软件

```bash
sudo apt install zsh sudo vim git \
	net-tools curl iproute2 traceroute wget nfs-utils \
	openssl-devel openssh-server \

# 编译工具
sudo apt install llvm build-essential gcc cmake ninja
```

## 增强 shell 

### zsh

加强 Shell: zsh + oh-my-zsh + [StarShip](https://starship.rs)
```bash
chsh -s /bin/zsh

# 

# startship
curl -sS https://starship.rs/install.sh | sh

# 进入 .config 修改 starship 的配置
mkdir -p ~/.config && touch ~/.config/starship.toml
cat <<EOF > ~/.config/starship.toml
[custom.my_custom_char]
command="echo '@debian'"
when="true"
EOF
```

安装 oh-my-zsh. 安装后 oh-my-zsh 会新建一个 `.zshrc`, 原文件被命名为 `.zshrc.pre-oh-my-zsh`, 改回来即可.

```sh
ZSH= sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"

# 安装 oh-my-zsh 的插件
git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting

git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions

```

### bash

必备 fzf, 增强 Ctrl+R 功能.

```
sudo apt install fzf
source /usr/share/fzf/shell/key-bindings.bash
```

推荐使用 fish. 

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

**完整的配置见 [.zshrc](../../../src/settings/unix.zshrc)**

## Reference

Linux from scratch (LSB)