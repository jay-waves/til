RPC（Remote Procedure Call）


* 客户端：客户、客户端代理（Stub）、网络 
* 服务端：服务端代理（Stub）、服务端
* 交互方式：序列化后的请求体 `RpcRequest`，以及序列化后的响应体 `RpcResponse`

### gRPC 

Google 开发的，需要和 ProtoBuf 搭配使用。


### 和 HTTP 的区别？

1. RPC 有状态；HTTP 无状态，通过 Cookie 来携带状态。
2. HTTP 通过 DNS 来获取服务器地址；RPC 需要专用的服务发现中间件，但可以基于 DNS 做。、
3. RPC 直接序列化函数体，而 HTTP 需要冗余的字符串消息头和序列化消息（JSON）。