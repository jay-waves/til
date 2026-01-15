详见:
- [.barshrc](../../src/settings/unix.bashrc)
- [.zshrc](../../src/settings/unix.zshrc)
- [powershell $PROFILE](../../src/settings/$PROFILE.ps1)

这些脚本会在 shell 加载时被执行, 可用于系统配置.

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

## 添加环境变量

一些程序存储在非标准位置, 我们将其添加到 `PATH` 中.
- `~/.local/bin`
- `~/opt/go/bin`

我的习惯是, 一些命令行工具放在 `~/.local` 下, 其他包管理及其产物放在 `/opt` 下.

```bash
# add ./local to path
export PATH=/usr/local/bin:$HOME/.local/bin:$PATH

# add vcpkg
export VCPKG_ROOT=/opt/vcpkg
export PATH=$VCPKG_ROOT:$PATH

# go, /opt/go
export GOPATH=/opt/go/usr
export PATH=/opt/go/bin:$GOPATH/bin:$PATH

# rust
export CARGO_HOME=/opt/rust/cargo
export PATH=/opt/rust/bin:$CARGO_HOME/bin:$PATH
```

## 配置网络

详见 [代理配置](../../networks/VPN/Proxy.md).