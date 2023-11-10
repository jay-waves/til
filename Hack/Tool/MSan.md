`MemorySanitizer` (aka. MSan) detects uninitialized memory reads for c.

Uninitialized values occur when stack- or heap-allocated memory is read before written. 

MSan detects cases where uninitialized values affect program execution, which means MSan tracks the spread of uninitialized data in memory (in *bit-exact*), and reports a warning when a code branch is taken (or not) depending on such data, otherwise keeps silent.

**use**: `-fsanitize=memory -fPIE -pie`
- `-fno-omit-frame-pointer`, get stack traces.
- `-fsanitize-memory-track-origins`, tracks the original allocation of uninitialized data with additional 1.5x-2.5x slowdown.