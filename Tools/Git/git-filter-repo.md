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