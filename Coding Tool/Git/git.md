删除已commit文件: `git rm --cached desktop.ini`

## 网络代理

linux 中, 修改 `.bashrc`; windows 中, 修改 `.gitconfig`

```bash
git config --global http.proxy "socks5://127.0.0.1:1080"
git config --global https.proxy "socks5://127.0.0.1:1080"

# 查看配置:
git config --list
```

使用该git配置, 指定使用IPv4 (而不是IPv6), 可以解决 git 的 `TLS Connection Failed` 问题. 见 [StackOverflow](https://stackoverflow.com/questions/51635536/the-tls-connection-was-non-properly-terminated). 这里的`--global`选项, 可能影响到Windows主机.

## 登录

github 目前已不支持密码登录, 需要 token.

repo 全权限 class token: `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` 这个寿命比较短

repo 全权限 fine-grained token, 无账户权限: `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`

使用 HTTPs 协议时, git 每次都要求登录密钥. 嫌麻烦, 可开启身份缓存: `git config --global credential.helper cache`. 默认15分钟, 自动缓存时间则用: `git config --global credential.helper 'cache --timeout=3600'`

windows上, 可以使用 wincred 记住密码: `git config --global credential.helper wincred`

## 管理分支

创建分支: `git checkout -b <new_branch>`

将新分支推送: `git push origin <new_branch>`. 推送后, 去 github 主页进行 pull request

在提交之前, 经常 `git pull origin main` 来确保本地和远程的主版本同步, 避免合并推送时出现冲突.

查看分支: `git branch [-r]` -r 查看远程分支

删除分支: `git branch -d <name>`, 远程: `git push origin --delete <name>`

### 基于当前分支的修改创建分支

`git stash`

`git checkout -b <new-heature>`

`git stash pop` 应用stash到当前分支, 并删除stash区. 如果不想删除, 用`git stash apply`. stash 用于保存不想提交的修改, add(stage)用于保存准备提交的修改.

### 基于新分支创建 pull request

fork别人仓库后, 创建一个新分支. 

推送修改: `git push origin <new_local_branch>[:<remote-branch-name>]`

上github, 建立一个pull request即可.

如果向合并入主分支, 需要先确保和origin/main同步: `git pull origin main`

### 修改分支名

`git branch -m <old_name> <new_name>`

远程分支改名:
- 直接删除原远程分支 `git push origin --delete <old_name>
- 推送新分支 `git push origin <new_name>`
- 设置上游链接 `git branch -u origin/<new_name>`

### 远程仓库改名

查看远程仓库
`git remote -v` 

更新远程仓库的 url:
`git remote set-url origin https://github.com/username/new_repo_name.git`

## 减少 .git 体积

.git 体积过于庞大, 通常是因为仓库内有二进制文件 (blomb), 备份体积膨胀得很快.

查看提交记录: `git log --oneline [--graph]`. 使用工具 `git-sizer` 查看仓库存储.

清理下垃圾: `git gc`, `git prune`

使用历史修改工具: `git-filter-repo`, 删除不用的和比较大的二进制文件.

**从网上克隆大项目时, 若不开发, 可以使用浅克隆, 减少 .git 体积:** `git clone --depth 1 <repos_url>`

## .gitignore

若发现文件没有被正常 ignore:
1. 检查 ignore 状态: `git check-ignore -v /path/to/file`
2. 删除已提交的文件缓存(这不会删除原文件, 只会让git不再追踪该文件) `git rm --cached -r /path/to/..`
3. 重新提交 `git commit -m "remove ignred files"`, 这会更新缓存.

```shell
# 匹配所有文件夹(包括子文件夹)下的文件, git会自动扫描
__pycache__/

# 指定目录后, 只会匹配该路径下的文件
./__pycache__/
```

## 从其他分支获取文件

从 main 分支获取 target_file 到当前分支暂存区.
```shell
git checkout main target_file
```

提交暂存区, 该文件就会被合并到分支:
```shell
git commit -m "checkout target_file from main"
```

如果在错误分支提交, 可以使用 `git reset --hard ORIG_HEAD` 来取消 merge 操作的改动.

### 合并其他分支

`git fetch rogin` 获取远程改动, 不合并

`git merge origin/main` 将远程主分支和当前分支合并

如果已经在 main 上合并了远程分支, 那么可以 `git merge main` 即可.

如果不想直接创建一个新提交, 就用 `git merge --squash`

### 打标签

`git tag v1.1`

`git push origin v1.1`, 或 `git push origin/branch v1.1`. 当前分支的默认远程分支并不会更改!! 而是远程创建了一个新tag分支(而不是branch). tag虽然和branch都是git的引用, 但有一些差别:
- tag 一般不变, branch 会一直活跃
- tag 用于标记特定时刻, branch 用于跟踪开发

之后可以在github基于该 tag 创建 release

### 创建追踪主分支的功能分支, 用于创建PR

#工作流

```shell
# 切回主分支, 拉取更新
git checkout main
git pull origin main

# 创建新分支, 基于main
git checkout -b new-feature main

# main 有更新, 切回main更新, 并merge到当前分支
git checkout main
git pull origin main
git checkout new-feature
git merge main

# 修改好后, 推送到远程, 建立PR
git push origin new-feature
git push origin local_branch:remote_branch
```