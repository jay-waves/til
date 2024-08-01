## URL Encode

### Purpose

URL encoding, also known as percent encoding, is a mechanism used to encode information in a Uniform Resource Locator (URL). Since URLs often contain characters that are not allowed in URLs or have special meanings, URL encoding converts these characters into safe formats.

### Rules

In URL encoding, each character is converted to its corresponding ASCII/UTF-8 value in hexadecimal, each two hexadecimal digit is prefixed by a percent sign `%`. Characters that must be URL-encoded include:
- **Reserved characters**: Characters that have special meanings in URLs, such as `?`, `/`, `#`, `&`, and `=`.
- **Non-ASCII characters**: Characters not present in the standard ASCII character set, such as Unicode characters. 
- **Unsafe characters**: Characters that are considered unsafe for URL transmission, such as spaces (encoded as `%20`).

### Examples

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

Example3: x-www-form-urlencoded format use `?` to substitute `%20` in query url

```
https://google.com/serach?q=hello+world
```