## Base58

### Purpose

Base58 is an encoding scheme used to convert binary data into an alphanumeric string. It is specifically designed to avoid ambiguous characters and improve readability, making it ideal for applications where data is manually entered or communicated, such as cryptocurrency addresses and short unique identifiers. Base58 is most commonly associated with Bitcoin and other cryptocurrencies.

```
Why base-58 instead of standard base-64 encoding?
- Don't want 0OIl characters that look the same in some fonts and
  could be used to create visually identical looking account numbers.
- A string with non-alphanumeric characters is not as easily accepted 
  as an account number.
- E-mail usually won't line-break if there's no punctuation to break at.
- Doubleclicking selects the whole number as one word if it's all
  alphanumeric.

by Satoshi Nakamoto, bitcoin/base58.h
```

### Encoding Rules

Base58 uses a set of 58 characters, deliberately excluding characters that are visually similar or could be confused from [Base64](Base64.md). The character set includes:
- Uppercase letters: `A-Z` (except for `O`)
- Lowercase letters: `a-z` (except for `l`)
- Digits: `1-9` (excluding `0`)

Specifically, the following characters are omitted to avoid confusion:
- The digit `0` (zero) and the uppercase letter `O` (oh)
- The lowercase letter `l` (ell) and the digit `1` (one)
- The uppercase letter `I` (eye)

### Examples

1. input: decimal 100
2. binary data: `1100100`
3. base58 enoding: `2j`

