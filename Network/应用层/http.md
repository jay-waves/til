HTTP, hyper text transfer protocol, 超文本传输协议. 端口为 80, 加密端口 (https) 为 443.

软件作为客户端 (Client) 和服务器 (Server) 通信, 称为 C/S 架构, 可以使用 RPC 协议.

浏览器 (Browser) 并不是和自家服务器 (Server) 通信, 而是和各个网络公司网页进行通信, 所以在 1990s 诞生了 HTTP 协议, 用于 B/S 架构下的通信.

目前多端开发中, B/S 和 C/S 在逐渐融合, 通信使用同一的 HTTP, 避免重复开发.

### 方法

HTTP 定义了多种请求方法 (http 动词), 表示对服务器资源的不同操作. 常见请求方法是 `GET` 和 `POST`.

- `Get` 最常见, 如打开网页操作.
- `Post` 用于提交表单到服务器, 如登录操作.
- `Put` 发送数据更新
- `Delete`
- `Patch` 更新资源
- `Head` 类似于Get, 但没有响应体

### 常见状态码

| 状态码 | 名称                       | 描述                                                                 |
| ------ | -------------------------- | -------------------------------------------------------------------- |
| 100    | Continue                   | 继续                                                                 |
| 101    | Switching Protocols        | 协议切换, 如服务器根据客户端请求升级到高级协议                       |
| 1xx    |                            | **信息性状态码**                                                     |
| **200**    | OK                         | 请求成功                                                             |
| 201    | Created                    | 已创建, 成功请求并创建了新资源                                       |
| 202    | Accepted                   | 已接收请求, 可能未处理完成                                           |
| 2xx    |                            | **成功状态码**                                                       |
| 301    | Moved Permanently          | 永久移动, 请求的资源已移动到新URI, 浏览器可能会自动重定向.           |
| 302    | Found                      | 资源被临时移动, 回复中 `Location` 字段可能包括新地址                                                       |
| 305    | Use Proxy                  | 请求的资源必须使用代理访问.                                          |
| 3xx    |                            | **重定向状态码**                                                     |
| 400    | Bad Request                | 客户端请求语法错误, 服务器不识别.                                    |
| **401**    | Unauthorized               | 请求需要用户提供身份认证                                             |
| **403**    | Forbidden                  | 服务器可以理解请求, 但是拒绝执行.                                    |
| **404**    | Not Found                  | 服务器无法根据客户端请求找到资源.                                    |
| 405    | Method Not Allowed         | 客户端请求的方法被禁止                                               |
| **408**    | Request Time-out           | 服务器等待客户端发送的请求时间过长, 超时.                            |
| 4xx    |                            | **客户端错误状态**                                                   |
| **500**    | Internal Server Error      | 服务器内部错误, 无法完成请求                                         |
| 501    | Not Implemented            | 服务器不支持请求的功能, 无法完成请求                                 |
| 502    | Bad Gateway                | 作为网关/代理的服务器尝试执行请求时, 从远程服务器接收到一个无效响应. |
| 505    | HTTP Version Not Supported |                                                                      |
| 5xx    |                            | **服务器错误状态码**                                                                     |


### 请求头

| 字段名           | 说明                       | 常见值                                                  |
| ---------------- | -------------------------- | ------------------------------------------------------- |
| `Host`           | 请求的主机名和端口号       | `Host: www.example.com`                                 |
| `User-Agent`     | 客户端浏览器标识符         | `User-Agent: Mozilla/5.0 (Windows NT 10.0; Win64; x64)` |
| `Accept`         | 客户端可处理的内容类型     | `Accept: text/html,application/xhtml+xml`               |
| `Content-Type`   | 请求内容的类型 (用于 POST) | `Content-Type: applcation/json`                         |
| `Content-Length` | 请求主体的长度 (字节数)    | `Content-Length: 348`                                   |
| `Authorization`  | 身份验证凭证               | `Authorization: Basic YWxhZGRpbjpvcGVuc2VzYW1l`         |
| `XFF`            | X-Forwarded-For, 经过代理或负载均衡时, 用此字段保存客户端的真实源 IP 地址.                           |                                                         |


```http
GET /search?q=openai HTTP/1.1
Host: www.example.com
User-Agent: Mozilla/5.0 (Windows NT 10.0; Win64; x64)
Accept: text/html,application/xhtml+xml
```

```htt
POST /submit-form HTTP/1.1
Host: www.example.com
User-Agent: Mozilla/5.0 (Windows NT 10.0; Win64; x64)
Content-Type: application/x-www-form-urlencoded
Content-Length: 27
name=John+Doe&age=30
```
