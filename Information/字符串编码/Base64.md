### Base64

**Purpose**:
Base64 is an encoding scheme used to represent binary data in a textual format. It is commonly used to encode data for safe transmission over media that are designed to handle text, such as email or HTTP.

**Encoding Rules**:
Base64 encoding converts binary data into a sequence of 64 characters, plus a padding character (`=`). It divides the input data into groups of three bytes (24 bits), then splits these 24 bits into four groups of six bits each (padding with `0` if no aligned to 3bytes). Each six-bit group is then mapped to one of $2^6=64$ characters from the Base64 character set.

The Base64 character set includes:
- Uppercase letters: A-Z
- Lowercase letters: a-z
- Digits: 0-9
- Special characters: + and /

If the input data's length is not a multiple of three, the output is padded with one or two `=` characters to ensure the encoded data's length is a multiple of four.

**Examples**:
1. input: "Hello"
2. ascii bianry: `01001000 01100101 01101100 01101100 01101111`
3. padding: `01001000 01100101 01101100 01101100 01101111 00000000`
4. base64 encoded: `SGVsbG8`
5. adding suffix: `SGVsbG8=`