### Functions

Function calls **bind tighter** than operators

| Haskell       | Python, Java or C |
| ------------- | ----------------- |
| `a + b`       | `a + b`           |
| `f a + g b`   | `f(a) + g(b)`     |
| `f (a + g b)` | `f(a+g(b))`       |

function application **asscociates left**, group expression:

| Haskell       | Python, Java or C |
| ------------- | ----------------- |
| `g h f 1`     | `g(h,f,1)`        |
| `g h (f 1)`   | `g(h,f(1))`       |
| `g (h f 1)`   | `g(h(f,1))`       |
| `g (h (f 1))` | `g(h(f(1)))`      |

### Basic Types

| Type                  | Literals                        | Use                         | Operations                  |
| --------------------- | ------------------------------- | --------------------------- | --------------------------- |
| `Int`                 | `1`, `2`, `-3`                  | Number type (signed, 64bit) | `+`, `-`, `*`, `div`, `mod` |
| `Integer`             | `1`, `-2`, `900000000000000000` | Unbounded number type       | `+`, `-`, `*`, `div`, `mod` |
| `Double`              | `0.1`, `1.2e5`                  | Floating point numbers      | `+`, `-`, `*`, `/`, `sqrt`  |
| `Bool`                | `True`, `False`                 | Truth values                | `&&`, `not`,...             |
| `String` aka `[Char]` | `"abcd"`, `""`                  | Strings of characters       | `reverse`, `++`             |
|                       |                                 |                             |                             |

Names of types (values) start with a capital letter, but Variables and functions start with a lower case (like `reverse, not, x`).


function types: `->`
- one argus functions: `argumentType -> returnType`
- ...of two argus: `argument1Type -> argument2Type -> returnType`

### Program Structure

in file: `Gold.hs`

```haskell
module Gold where

-- The golden ratio
phi :: Double
phi = (sqrt 5 + 1) / 2

polynomial :: Double -> Double
polynomial x = x^2 - x - 1

f x = polynomial (polynomial x)

main = do
  print (polynomial phi)
  print (f phi)
```

#### div 

`` `div` `` 仅用于整数型, 如 Int, Integer.

```haskell
Prelude> 7 div 2
3
```

> 注意,  `` ` `` 用于将函数变为中缀表达式. 如 `div 10 3` 改写为 `` 10 `div` 3``

`/` 仅用于浮点数型, 如 Double.

```haskell
Prelude> 7.0 / 2.0
3.5
```







