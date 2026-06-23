### 查看修改

同分支上次提交和最新提交的区别:

```bash
git diff HEAD~1

git diff HEAD~3 --stat  # 祖上三代
```

查看暂存区和上次提交的差异:
```bash
git diff --cached
```

查看工作目录和最新提交的差异:
```bash
git diff HEAD
```

## 暂存区

### 暂时回退版本

```bash
git checkout <commit_hash>  # 创建一个临时分支, detached HEAD

git checkout <commit_hash> -- /path/to/files # 仅回退几个文件的版本

git switch main             # 回到主分支 (最新修改)

git swtich -c temp_branch   # 在 detached HEAD 状态下创建一个新分支并保存修改
```

### 撤销提交

撤销提交, 并保留修改在暂存区:
```shell
git log
git reset --soft HEAD~1
```

撤销提交, 撤销暂存区, 但保留文件修改: 这也是默认模式
```shell
git reset --mixed HEAD~1
```

撤销提交, 并撤销修改: (不会有自上次提交后的文件修改保留)
```shell
git reset --hard HEAD~1
```

**如果不慎未保存修改, 可以使用 `git reflog` 来查看本地分支的所有变动引用, 可以用于恢复错误操作.**

**注意**, `HEAD~1` 指向上一个提交, 仅有一个提交或没有提交时不适用, 此时应该使用:
```shell
git update-ref -d HEAD
```

### 撤销暂存(stage)

未编写好 `.gitignore`, 就将所有文件 `add` (比如将编译目录 `build/` 错误提交了), 此时使用:

```shell
git reset HEAD build/
```

此时, 可进一步撤销修改 (回退到上一提交版本):

```shell
git checkout -- build/
```

## 提交

## .git 

### 清理某个文件的历史

使用工具 git-filter-repo (推荐使用 pip 安装):
1. 先备份副本容灾
2. 确保所有内容已被提交
3. 使用 git-filter-repo 修改 git 历史, 主要用于删除历史中大文件缓存.
4. `git filter-repo --analyze` 生成历史分析文件, 在 `.git/filter-repo` 中.
5. 程序本身是一个 `py` 脚本, 也可以直接运行.
6. 修改完历史后, 使用 `git push origin main --force` 推送. 注意, 单人仓库才可以这样做.

git 每条历史的哈希都是由缓存文件计算的, 所以要删除缓存中的文件, 就需要把整个git历史都修改掉.

常用命令: 
- `git filter-repo --inver-paths --path-glob 'attach/garbage.jbin' --force`
- `git filter-repo --strip-blobs-bigger-than 10M` 注意这也会删除现存文件....

覆盖远程仓库后, 使用 `api.github.com/repos/jay-waves/til` 来查看仓库元信息变化.

### 减少 .git 体积

.git 体积过于庞大, 通常是因为仓库内有二进制文件 (blomb), 备份体积膨胀得很快.

查看提交记录: `git log --oneline [--graph]`. 使用工具 `git-sizer` 查看仓库存储.

清理下垃圾: `git gc`, `git prune`

使用历史修改工具: `git-filter-repo`, 删除不用的和比较大的二进制文件.

### .gitignore

-  特殊标记：
1. `!` 取消忽略
2. `/dir`与`dir`区别
	- `/dir` 仅代表忽略根目录下的dir目录
	- `dir` 代表忽略任何子目录下的dir目录。但是dir后不能接任何东西，否则失效
3. `**`忽略任何文件（常用）
4. `#`注释
