- Address Sanitizer: `libasan5`
- Leak Sanitizer: `liblsan0`
- Undefined Behavior Snitizer: `libubsan1`

## MSan 

`MemorySanitizer` (aka. MSan) detects uninitialized memory reads for c.

Uninitialized values occur when stack- or heap-allocated memory is read before written. 

MSan detects cases where uninitialized values affect program execution, which means MSan tracks the spread of uninitialized data in memory (in *bit-exact*), and reports a warning when a code branch is taken (or not) depending on such data, otherwise keeps silent.

**use**: `-fsanitize=memory -fPIE -pie`
- `-fno-omit-frame-pointer`, get stack traces.
- `-fsanitize-memory-track-origins`, tracks the original allocation of uninitialized data with additional 1.5x-2.5x slowdown.

## TSan

`ThreadSanitizer` (aka. TSan) detects data-race bugs for c. **Data race** , which is banned by C++11 as *Undefined Behavior*,  occurs when two threads access the same variable concurrently and at least one of the accesses is write.

*use*: compile with flags `-fsanitizer=thread`.
- `-o2` optimize code performance.
- `-g` get file names and line numbers in the warning msgs.

详见 [Thread Sanitizer](Thread%20Sanitizer.md)

## UBSan

UndefinedBehaviorSanitizer

## Asan

[AddressSanitizer (Aka. ASan)](Address%20Sanitizer.md) detects bugs for C++ like:
- **Use after free** (dangling pointer dereference)
- **Heap/Stack/Global buffer overflow**
- Use after return, due to error scope. with flags: `ASAN_OPTIONS=detect_stack_use_after_return=1`.
- **Use after scope**. with flags: `-fsanitize-address-use-after-scope`
- Initialization order bugs
- Memory leaks

### how to use

use `clang` to compile and link 
- `-fsanitize=address` enable ASan
- `-01` optimize code performance
- `-fno-omit-frame-pointer` get nicer stack traces in error msgs.

## LSan

## Sanitizer Coverage

Sanitizer 自带的覆盖率统计工具, 见:
- [Sanitizer Coverage](Sanitizer%20Coverage.md)
- [AFL](AFL.md)
- [Coverage](Coverage.md)

