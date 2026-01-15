-  特殊标记：
1. `!` 取消忽略
2. `/dir`与`dir`区别
	- `/dir` 仅代表忽略根目录下的dir目录
	- `dir` 代表忽略任何子目录下的dir目录。但是dir后不能接任何东西，否则失效
3. `**`忽略任何文件（常用）
4. `#`注释

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

### 处理已提交后, 又想 ignore 的文件

1. 删除已提交文件: `git rm --cached desktop.ini`
2. 在 `.gitignore` 中添加要忽略的目标文件, 重新提交.

### 本地停止追踪

不修改远程, 不修改 `.gitignore`, 忽略所有在本地的修改.

```bash
git update-index --assume-unchanged xxx\xxx\xxx.file
```