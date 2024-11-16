> Base Encoding (RFC 3548, 2003), MIME (RFC 2045, 1996):
> 
> "Base Encoding of data is used in many situations to store or transfer
> data in environments that, perhaps for legacy resons, are restricted 
> to only US-ASCII data. Base encoding can also be used in new applications 
> that do not have legacy restrictions, simply because it possible to 
> manipulate objects with text editors."

Base 编码将数据编码为可见字符串, 防止老旧信道将二进制数据默认识别为 ASCII 控制字符. 比如传输 `0x04`, 在 ASCII 里表示的意思是 End of Transmission; 又比如电子邮件中, 不同操作系统可能会自动修改邮件中的换行符 `\n` 或 `\r\n`, 可能造成附件被篡改. 本质也是类似反斜杠 `\t` 这样的转义方式.

## Base64

包含 64 个字符和用于填充的等号 `=`. 每三个字节二进制数据变为四字节的 ASCII 字符串, 将导致约 33% 的数据膨胀.

**Encoding Rules**: 
It divides the input data into groups of three bytes (24 bits), then splits these 24 bits into four groups of six bits each (padding with `0` if no aligned to 3bytes). Each six-bit group is then mapped to one of $2^6=64$ characters from the Base64 character set.

The Base64 character set includes:
- Uppercase letters: `A-Z`
- Lowercase letters: `a-z`
- Digits: `0-9`
- Special characters: `+` and `/`

If the input data's length is not a multiple of three, the output is padded with one or two `=` characters to ensure the encoded data's length is a multiple of four.

**Examples**:
1. input: "Hello"
2. ascii bianry: `01001000 01100101 01101100 01101100 01101111`
3. padding: `01001000 01100101 01101100 01101100 01101111 00000000`
4. base64 encoded: `SGVsbG8`
5. adding suffix: `SGVsbG8=`

## Base58

Base58 是比特币和虚拟货币应用中常见的编码形式. 从 Base64 中删除了一些易混淆的字符.

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

**Encoding Rules:**  
Base58 uses a set of 58 characters, deliberately excluding characters that are visually similar or could be confused from Base64. The character set includes:
- Uppercase letters: `A-Z` (except for `O`)
- Lowercase letters: `a-z` (except for `l`)
- Digits: `1-9` (excluding `0`)

Specifically, the following characters are omitted to avoid confusion:
- The digit `0` (zero) and the uppercase letter `O` (oh)
- The lowercase letter `l` (ell) and the digit `1` (one)
- The uppercase letter `I` (eye)

**Examples:**  
1. input: decimal 100
2. binary data: `1100100`
3. base58 enoding: `2j`

