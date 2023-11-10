AddressSanitizer (Aka. ASan) detects bugs for C++ like:
- **Use after free** (dangling pointer dereference)
- **Heap/Stack/Global buffer overflow**
- Use after return, due to error scope. with flags: `ASAN_OPTIONS=detect_stack_use_after_return=1`.
- **Use after scope**. with flags: `-fsanitize-address-use-after-scope`
- Initialization order bugs
- Memory leaks

### use

use `clang` to compile and link 
- `-fsanitize=address` enable ASan
- `-01` optimize code performance
- `-fno-omit-frame-pointer` get nicer stack traces in error msgs.

### principle

The run-time library replaces the `malloc` and `free` functions. The memory around malloc-ed regions (red zones) is poisoned. The `free`\-ed memory is placed in quarantine and also poisoned. Every memory access in the program is transformed by the compiler in the following way:

Before:

```
*address = ...;  // or: ... = *address;
```

After:

```
if (IsPoisoned(address)) {
  ReportError(address, kAccessSize, kIsWrite);
}
*address = ...;  // or: ... = *address;
```

...