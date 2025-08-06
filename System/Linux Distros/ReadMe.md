
## Linux 分发版配置

软件安装:
```bash
sudo apt install zsh sudo vim git \
	net-tools curl traceroute wget nfs-utils \
	openssl-devel openssh-server \

# 编译工具
sudo apt install llvm build-essential gcc cmake ninja 
```

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

配置键映射: 
```bash
# in .zshrc
# 将 CapsLock 和 Escape 互换
xmodmap -e "keycode 9 = Caps_Lock"
xmodmap -e "keycode 66 = Escape"
```

配置部分软件的代理

```bash
# software proxy
git config --global http.proxy "socks5://$host_ip:7890"
git config --global http.proxy "socks5://$host_ip:7890"
```

**完整的配置见 [.zshrc](../../src/settings/unix.zshrc)**

## Reference

Linux from scratch (LSB)