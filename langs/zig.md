zig 将 rust `Result<T, E>` 式的错误处理控制流进一步化简

> #TODO 关注一下 1.0 版本发布，现在太不稳定了

```zig
const std = @import("std");

fn read() !i32 {
	const input = "not_a_num";
	const num = std::fmt.parseInt(i32, input, 10) 
			catch return error.InvalidInput;
	return num;
}

pub fn main() void {
	const result = readNumber();
	if (result) |val| {
		std.debug.print("Numbfer {}\n", .{val});
	} else |err| {
		std.debug.print("Error occured");
	}
}
```

用 `defer` 延迟清理资源，并用 `try` 自动向上传递错误

```zig
pub fn main() !void {
	var file = try std.fs.cwd().openFile("data.txt", .{});
	defer file.close();
	
	const buffer = try file.readToEndAlloc(std.heap.page_allocator, 4096);
	defer std.heap.page_allocator.free(buffer);

}
```

## 模式匹配

仅支持简单模式匹配，用 `|x|` 来捕获匹配的值。

optional 类型：

```zig 
const maybe: ?i32 = 42;

if (maybe) |val| {
	// use val
} else {
	// may is null
}
```

error union 或普通 union 类型匹配：

```zig
const my = union(enum) {
	i: i32,
	f: f32,
};

var u: my = .{ .i = 10 };

switch (u) {
	.i => |val| std.debug.print(val);
	.f => |val| std.debug.print(val); 
}
```