

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