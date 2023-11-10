coverage 编译时生成 .gcno 文件 (gcov notation), 记录源代码 (单个文件) 的块图和基本信息; 运行时, 会参考 .gcda 文件 (gcov data) 收集代码覆盖信息, 生成实际执行的代码路径信息, 记录到 .gcda. 

这两个文件是通用的, 但是有不同的分析工具. 如 lcov, gcovr, 

### 使用 lcov

1. 编译

nav2 所用命令:
```shell
colcon build \
	--symlink-install \
	--parallel-workers 1 \
	--packages-select nav2_amcl \
	--cmake-args \
		-DCMAKE_BUILD_TYPE=Debug \
		-DBUILD_TESTING=OFF \
		-DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ \
		-DCMAKE_CXX_STANDARD=17 \
		-DCMAKE_CXX_FLAGS="${CMAKE_CXX_FLAGS} \
			-w -Wno-error \
			--coverage -DCOVERAGE_RUN=1" \
		-DCMAKE_C_FLAGS="${CMAKE_C_FLAGS}  \
			-w -Wno-error \
			--coverage -DCOVERAGE_RUN=1"
```

`--coverage` 是 `-fprofile-arcs -ftest-coverage` 的平替. `-DCOVERAGE_RUN` 使程序不统计测试代码的覆盖率.

原教程中命令:

```shell
$ colcon build --mixin coverage
$ find ./build -name "*.gcno"
# a gcno for each source file that was built
$ rm $(find ./build -name "CMake*CompilerId.gcno")
# These are created by the CMake compiler checker
$ mkdir lcov
```

2.  (optional) 用 lcov 生成零基准报告 baseline zero-coverage, 作为参照系

```shell
$ lcov \
    --base-directory .  \ 
    --initial \                              # 生成基准零覆盖率报告
    --capture \                              # 捕获覆盖率数据
    --directory ./build \                    # 在 ./build 目录下查找 .gcno .gcda
    --output-file ./lcov/zero-coverage \     
    --no-external \                          # 不包括外部代码
    --rc lcov_branch_coverage=1              # 分支覆盖率报告
```

3.  执行测试, 或单纯运行程序.

```shell
$ colcon test
$ find ./build -name "*.gcda"
# a gcda for each source file whose code was executed
```

4.  用 lcov 生成报告

> lcov 参考[官方仓库](https://github.com/linux-test-project/lcov)
```shell
$ lcov \
    --base-directory . \
    --capture \
    --directory ./build \
    --output-file lcov/coverage \
    --no-external \
    --rc lcov_branch_coverage=1
```

5.  (optional) Combine zero-coverage (from step 2) with coverage output (from step 4)

```shell
$ lcov \
    --add-tracefile lcov/zero-coverage \
    --add-tracefile lcov/coverage \
    --output-file lcov/total_coverage \
    --rc lcov_branch_coverage=1
```

6.  Use `genhtml` to generate browsable coverage information

```shell
$ genhtml \
    --output-directory ./lcov/html \
    "./lcov/total_coverage" \
    --prefix . \
    --legend \
    --demangle-cpp \
    --rc genhtml_branch_coverage=1

# just open is
chromium-browser ./lcov/html/index.html
```

> 参考 [Add a mixin for GCC coverage flags by jpsamper2009 · Pull Request 8 · colconcolcon-mixin-repository](https://github.com/colcon/colcon-mixin-repository/pull/8)

### 使用 llvm-cov

### 使用 gcovr

> 参考 [gcovr docs](https://gcovr.com/en/stable)
```shell
# clang 编译需要使用 llvm-cov 工具
# gcc 则使用 gcov
gcovr --gcov-executable "llvm-cov gcov"

# 输出 json 格式, --json-pretty 更易读, --json-summary 
gcovr --json coverage.json --json-pretty

gcovr --output coverage_summary.json --json-summary-pretty

# 排除不可达代码
gcovr --exclude-unreachable-branches

# 排除分支代码
gcovr --exclude-throw-branches
```

