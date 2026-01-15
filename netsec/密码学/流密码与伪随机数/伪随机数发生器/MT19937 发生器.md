---
source: https://en.wikipedia.org/wiki/Mersenne_Twister#TinyMT
code: [src/cryptography/mt19937.py,]
---


> 详细信息见[梅森旋转算法wiki]()

### PseudoCode

MT19937 所用常数: (注意, 这里的 $A_{32}$ 中 $32$ 是位宽的意思)

$$\begin{align}
(w,n,m,r) & = (32, 624, 397, 31) \\
a & =9908B0DF_{32} \\
(u,d) & =(11, FFFFFFFF_{32}) \\
(s,b) & =(7, 9D2C5680_{32}) \\
(t,c) & =(15, EFC60000_{32}) \\
l & =18
\end{align}$$

```python
# Create a length n array to store the state of the generator
 int[0..n-1] MT
 int index := n+1
 const int lower_mask = (1 << r) - 1 # That is, the binary number of r 1's
 const int upper_mask = lowest w bits of (not lower_mask)
 
# Initialize the generator from a seed
function seed_mt(int seed) {
     index := n
     MT[0] := seed
     for i from 1 to (n - 1) { # loop over each element
         MT[i] := lowest w bits of (f * (MT[i-1] ^ (MT[i-1] >> (w-2))) + i)
     }
}
 
# Extract a tempered value based on MT[index]
# calling twist() every n numbers
function extract_number() {
    if index >= n {
         if index > n {
           error "Generator was never seeded"
           # Alternatively, seed with constant value; 5489 is used in reference C code
         }
         twist()
    }
    \
     int y := MT[index]
     y := y ^ ((y >> u) & d)
     y := y ^ ((y << s) & b)
     y := y ^ ((y << t) & c)
     y := y ^ (y >> l)
     \
     index := index + 1
     return lowest w bits of (y)
}
 
# Generate the next_n_values from the series x_i
function twist() {
    for i from 0 to (n-1) {
         int x := (MT[i] & upper_mask)
                   | (MT[(i+1) mod n] & lower_mask)
         int xA := x >> 1
         if (x mod 2) != 0 { # lowest bit of x is 1
             xA := xA ^ a
         }
         MT[i] := MT[(i + m) mod n] ^ xA
    }
    index := 0
}
```
