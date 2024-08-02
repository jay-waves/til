---
	url: https://datatracker.ietf.org/doc/html/rfc6749
date: 24-08-02
---

## 令牌与密码的区别

令牌 (token) 和密码 (password) 皆用于认证, 但有所区别:
1. 令牌是短期的, 到期失效, 且用户无法自行修改. 而密码是长期的
2. 颁发给他人的令牌是可以被数据所有者撤销的, 立即失效.
3. 令牌有权限范围 (scope), 而密码一般拥有完整权限.

令牌是颁发给他人的, 如 Github 用户将令牌颁发给第三方应用, 用于获取用户*授予权限的*部分数据. 之后第三方应用就能持续获取这部分数据, 直到*令牌到期或用户撤销*.

## OAuth2.0

OAuth2.0 是 [RFC6749](https://datatracker.ietf.org/doc/html/rfc6749) 定义的令牌授权机制, 用于将用户和客户端 (第三方应用) 的权限隔离开:

> OAuth addresses these issues by introducing an authorization layer
   and separating the role of the client from that of the resource
   owner.  In OAuth, the client requests access to resources controlled
   by the resource owner and hosted by the resource server, and is
   issued a different set of credentials than those of the resource
   owner.
>
> Instead of using the resource owner's credentials to access protected
   resources, the client obtains an access token -- a string denoting a
   specific scope, lifetime, and other access attributes. Access tokens
   are issued to third-party clients by an authorization server with the
   approval of the resource owner.  The client uses the access token to
   access the protected resources hosted by the resource server.

这里的 "Client" 指第三方应用程序, "User Agent" 一般指用户浏览器, "Resource Owner" 就是指用户了. 注意整个过程处于 TLS 保护下.

```
     +--------+                               +---------------+
     |        |--(A)- Authorization Request ->|   Resource    |
     |        |                               |     Owner     |
     |        |<-(B)-- Authorization Grant ---|               |
     |        |                               +---------------+
     |        |
     |        |                               +---------------+
     |        |--(C)-- Authorization Grant -->| Authorization |
     | Client |                               |     Server    |
     |        |<-(D)----- Access Token -------|               |
     |        |                               +---------------+
     |        |
     |        |                               +---------------+
     |        |--(E)----- Access Token ------>|    Resource   |
     |        |                               |     Server    |
     |        |<-(F)--- Protected Resource ---|               |
     +--------+                               +---------------+

                     Figure 1: Abstract Protocol Flow
```

其中 "Authorization Grant" 共有四种方式:
- authorization-code
- implicit
- password
- client credentials

### Authorizaiton Code Grant

```
     +----------+
     | Resource |
     |   Owner  |
     |          |
     +----------+
          ^
          |
         (B)
     +----|-----+          Client Identifier      +---------------+
     |         -+----(A)-- & Redirection URI ---->|               |
     |  User-   |                                 | Authorization |
     |  Agent  -+----(B)-- User authenticates --->|     Server    |
     |          |                                 |               |
     |         -+----(C)-- Authorization Code ---<|               |
     +-|----|---+                                 +---------------+
       |    |                                         ^      v
      (A)  (C)                                        |      |
       |    |                                         |      |
       ^    v                                         |      |
     +---------+                                      |      |
     |         |>---(D)-- Authorization Code ---------'      |
     |  Client |          & Redirection URI                  |
     |         |                                             |
     |         |<---(E)----- Access Token -------------------'
     +---------+       (w/ Optional Refresh Token)

Note: The lines illustrating steps (A), (B), and (C) are broken into
   two parts as they pass through the user-agent.

                     Figure 3: Authorization Code Flow
```

authorization request: 这里可选参数 `scope` 用于规定令牌的权限. 客户端需要提前去认证服务器注册 `redirect_uri`, 获得分配的 `client_id` 和 `client_secret`.  

```http
GET /authorize?response_type=code
	&client_id=s6BhdRkqt3
	&state=xyz
	&redirect_uri=https%3A%2F%2Fclient%2Eexample%2Ecom%2Fcb HTTP/1.1
Host: server.example.com
```

authorization response: 注意这里的 `state` 和请求时一致, `code` 指授权码, 一般有效期不超过10分钟, 客户端只能使用该码一次用于请求令牌.
```http
HTTP/1.1 302 Found
Location: https://client.example.com/cb?code=SplxlOBeZQQYbYS6WxSbIA
				 &state=xyz
```

access token request: 这里的 `Authorization` 字段指客户端凭证, 格式为 `base64_encode(client_id:client_secret)`

```http
POST /token HTTP/1.1
Host: server.example.com
Authorization: Basic czZCaGRSa3F0MzpnWDFmQmF0M2JW
Content-Type: application/x-www-form-urlencoded

grant_type=authorization_code
	&code=SplxlOBeZQQYbYS6WxSbIA
	&redirect_uri=https%3A%2F%2Fclient%2Eexample%2Ecom%2Fcb
```

access token response: `expires_in` 标识令牌的过期时间.
```http
HTTP/1.1 200 OK
Content-Type: application/json;charset=UTF-8
Cache-Control: no-store
Pragma: no-cache

{
	"access_token":"2YotnFZFEjr1zCsicMWpAA",
	"token_type":"example",
	"expires_in":3600,
	"refresh_token":"tGzv3JOkF0XG5Qx2TlKWIA",
	"example_parameter":"example_value"
}
```

#### PKCE

PKCE（Proof Key for Code Exchange）机制是增强 OAuth 2.0 授权码模式（Authorization Code Flow）安全性的补充措施. 用于防范授权码拦截和重放攻击. 使用 PKCE 后, 就无需再提供客户端秘密 `Authorization`.

客户端生成 `code_verifier`, 再通过算法 (如 SHA256) 生成 `code_challenge`. 
客户端请求授权码时, 将挑战值发送给授权服务器:

```http
GET /authorize?
    response_type=code
    &client_id=s6BhdRkqt3
    &redirect_uri=https://client.example.com/cb
    &code_challenge=xxxxxxxxxxxxxxxxxxxxxx
    &code_challenge_method=S256&
    state=xyz
```

客户端收到授权码后, 请求令牌, 此时一并提供 `code_verifier`, 注意不再提供 `Authorization` 客户端凭证.

```http
POST /token
Host: server.example.com
Content-Type: application/x-www-form-urlencoded

&grant_type=authorization_code
&code=SplxlOBeZQQYbYS6WxSbIA
&redirect_uri=https://client.example.com/cb
&client_id=s6BhdRkqt3
&code_verifier=xxxxxxxxxxxxxxxxxxxxxxx
```

服务器收到请求后, 验证 `code_veifier` 和 `code_challenge_method`. 

### Implicit

主要用于无法安全存储客户端凭证 (Authorization=`client_id:client_secret`) 的公共客户端, 如纯前端应用和移动应用. 隐式模式通将令牌直接传递给客户端, 降低了潜在的中间人攻击. 但也有明显安全缺陷.

使用 AuthorizationCode模式 + PKCE 机制, 理论能够安全替代 Implicit 模式.

```
     +----------+
     | Resource |
     |  Owner   |
     |          |
     +----------+
          ^
          |
         (B)
     +----|-----+          Client Identifier     +---------------+
     |         -+----(A)-- & Redirection URI --->|               |
     |  User-   |                                | Authorization |
     |  Agent  -|----(B)-- User authenticates -->|     Server    |
     |          |                                |               |
     |          |<---(C)--- Redirection URI ----<|               |
     |          |          with Access Token     +---------------+
     |          |            in Fragment
     |          |                                +---------------+
     |          |----(D)--- Redirection URI ---->|   Web-Hosted  |
     |          |          without Fragment      |     Client    |
     |          |                                |    Resource   |
     |     (F)  |<---(E)------- Script ---------<|               |
     |          |                                +---------------+
     +-|--------+
       |    |
      (A)  (G) Access Token
       |    |
       ^    v
     +---------+
     |         |
     |  Client |
     |         |
     +---------+

   Note: The lines illustrating steps (A) and (B) are broken into two
   parts as they pass through the user-agent.

                       Figure 4: Implicit Grant Flow
```

隐式授权模式将访问令牌直接暴露在 URL 中, 即使受 TLS 保护, 也可能被浏览器历史和恶意脚本等获取. 因此该模式不支持刷新令牌 (refresh token), 即过期后必须重新由用户授权, 而不能自动授权, 有效期也较短.

client request:
```http
GET /authorize?response_type=token
		&client_id=s6BhdRkqt3&state=xyz
		&redirect_uri=https%3A%2F%2Fclient%2Eexample%2Ecom%2Fcb HTTP/1.1
Host: server.example.com
```

authorization server response:
```http
HTTP/1.1 302 Found
Location: http://client.example.com/cb
				 #access_token=2YotnFZFEjr1zCsicMWpAA
				 &state=xyz
				 &token_type=example
				 &expires_in=3600
```

### Password

高度信用某个应用时, 可以直接将密码交给应用. 该应用直接使用你的密码获取临时令牌.

```
     +----------+
     | Resource |
     |  Owner   |
     |          |
     +----------+
          v
          |    Resource Owner
         (A) Password Credentials
          |
          v
     +---------+                                  +---------------+
     |         |>--(B)---- Resource Owner ------->|               |
     |         |         Password Credentials     | Authorization |
     | Client  |                                  |     Server    |
     |         |<--(C)---- Access Token ---------<|               |
     |         |    (w/ Optional Refresh Token)   |               |
     +---------+                                  +---------------+

            Figure 5: Resource Owner Password Credentials Flow
```

```http
POST /token HTTP/1.1
Host: server.example.com
Authorization: Basic czZCaGRSa3F0MzpnWDFmQmF0M2JW
Content-Type: application/x-www-form-urlencoded

grant_type=password&username=johndoe&password=A3ddj3w
```

### Client Credentials

适用于机器间通信, 客户端软件有一个独有的密码 `client_secret`, 用于服务器认证其身份. 该过程不涉及用户的参数或授权, 而是以客户端自己的名义向服务商认证, 严格说不属于 OAuth 定义.

```
     +---------+                                  +---------------+
     |         |                                  |               |
     |         |>--(A)- Client Authentication --->| Authorization |
     | Client  |                                  |     Server    |
     |         |<--(B)---- Access Token ---------<|               |
     |         |                                  |               |
     +---------+                                  +---------------+

                     Figure 6: Client Credentials Flow
```

```http
https://oauth.b.com/token?
  grant_type=client_credentials&
  client_id=CLIENT_ID&
  client_secret=CLIENT_SECRET
```