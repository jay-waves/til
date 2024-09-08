## URL Encode

URL 编码, 也叫百分号编码. 用于在 URL (Uniform Resource Locator) 中表示特殊字符和需转义字符的编码格式. 

**Rules:**  
In URL encoding, each character is converted to its corresponding ASCII/UTF-8 value in hexadecimal, each two hexadecimal digit is prefixed by a percent sign `%`. Characters that must be URL-encoded include:
- **Reserved characters**: Characters that have special meanings in URLs, such as `?`, `/`, `#`, `&`, and `=`.
- **Non-ASCII characters**: Characters not present in the standard ASCII character set, such as Unicode characters. 
- **Unsafe characters**: Characters that are considered unsafe for URL transmission, such as spaces (encoded as `%20`).

**Examples:**  
Example1:
1. Original URL: `https://example.com/path/to page?query=hello world`
2. URL encoded: `https://example.com/path/to%20page?query=hello%20world`

Example2:
1. Original string: `Café`
2. URL encoded: `Caf%C3%A9`

Examples3:
1. Original string: `你`
2. Unicode: `%u4F60`
3. UTF-8 encoded: `E4 B8 80`
4. URL encoded: `%E4%B8%80`

Example3:  

x-www-form-urlencoded format use `?` to substitute `%20` in query url

```
https://google.com/serach?q=hello+world
```