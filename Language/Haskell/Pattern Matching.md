```haskell
factorial :: Int -> Int
factorial 0 = 1
factorial n = n * factorial (n - 1)
```

```haskell
[] -- 空列表
x:xs -- 头部元素 x 和 尾部列表 xs
head [1, 2, 3] -- 返回 1
tail [1, 2, 3] -- 返回 [2, 3]
length [1, 2, 3] -- 返回 3
null []        -- 返回 True
reverse [1, 2, 3] -- 反转列表 [3, 2, 1]
```

```haskell
map (*2) [1, 2, 3]              -- [2, 4, 6]
filter (>1) [1, 2, 3]           -- [2, 3]
foldl (+) 0 [1, 2, 3]           -- 6
foldr (+) 0 [1, 2, 3]           -- 6
```

```haskell
(.) :: (b -> c) -> (a -> b) -> a -> c
(f . g) x = f(g x)
```

```haskell
let add x y = x + y
let add5 = add 5
add5 3            -- 8
```