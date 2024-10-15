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