BUILD.gn

```gn
executable("main") {
	sources = [
		"main.cpp",
	]

	deps = [
		"//lib2",
	]
}
```

`deps` 中声明依赖的对象, `sources` 中声明源文件. 语法接近 json.

**gn支持调试**. 

gn 需要搭配 [ninja](ninja.md) 使用:

```bash
gn gen -C out/a_build
ninja -C out/a_build
./out/a_build/hello
```

1. 在当前目录查找 `.gn`, 找不到就沿目录向上, 知道找到 `.gn`.
2. 执行构建 `.gn` 中声明的配置文件 `BUILDCONFIG.gn`, 获取默认的工具链文件的位置
3. 执行 toolchain, 确定编译和链接规则
4. 找到根目录下的 `BUILD.gn`, 并递归加载子目录下的所有 `BUILD.gn`, 解析依赖项
5. 解决目标依赖关系后, 将 `xx.ninja` 文件写入构建结果目录
6. 解决所有目标后, 写出根 `build.ninja` 文件
7. 手动执行 `ninja -C`, 根据 `build.ninja` 文件编译目标