`if` in Haskell,  corresponds to `?:` in C:

```Java
//Java
int price = product.equals("milk") ? 1:2;
```

```Haskell
price = if product == "milk" then 1 else 2
```

**Always a `else` is needed in Haskell**, because `if` 'has' a value.

```haskell
> True || False
True

> "bike" /= "bike" -- /=, not equal
False
```

