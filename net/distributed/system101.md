## API Architecture 

| Style     |  Format  | Use Cases                                     |
| --------- | ------------ | --------------------------------------------- |
| SOAP      |    XML          | XML_based, for enterprise applications        |
| RESTful   |  XML, JSON, HTML, text            | Resource-based, for web servers               |
| GraphQL   |   JSON            | Queryy language, reduce network load          |
| gRPC      | XML, JSON, Protobuf, ...              | High performance, for microservices           |
| WebSocket |              | Bi-directional, for low-latency data exchange |
| Webhook   |              | Asynchronous, for event-driven                                              |

### API Performance Improvement 

- pagination, 分页技术
- aysnc logging, 异步日志. 避免同步日志造成过大 I/O 开销.
- caching, 缓存.
- Payload Compression. 压缩, 用于节省带宽, 但也增大了收发两端处理开销.
- Connection Pool

### API First Development

Code First:
1. write code.
2. Integrate Code <--- run tests, write api docs.
3. Deploy, release new version 

API First: (TDD, Test Driven Design)
1. Design API 
2. Review API, Mock API
3. Develeop API, Write Tests --> Write Code 
4. Deploy API, release new version.

### Reverse Proxy 

Forward Proxy:
1. 保护内网用户
2. 限制访问特定内容
```
[LAN ---> Froward Proxy] ---> Internet ---> Web Servers 
```

Reverse Proxy:
1. 转发用户请求给服务器
2. 保护服务器, 负载均衡或缓存.
3. 负责加密和握手.
```
User ---> Inetrnet ---> [Reverse Proxy ---> Web Servers]

[LAN]
```

### CI

*CI/CD (Continous Integration/Continous Delivery)*

1. 开发者提交代码到 Git 
2. CI 服务检测到代码修改, 自动构建
3. 代码测试 (单元测试 + 集成测试), 并报告测试结果.
4. 若测试成功, 当前环境被保存. 否则环境回退到上一个版本.
5. CD 将代码修改部署到实际产品中.

## Architecture Patterns 

- View: 展示内容, 并接受用户输入
- Model: 控制商业数据
- 

MVC (Model-View-Controller):
```
         +-----notify---------> Controller -------update-----+
         |                                                   |
User -> View  <----------- get data  ---------------------- Model
```

MVP (Model-View-Presenter):
```
         +--------notify--------+     +------update---------+
         |                      |     |                     |
         |                      v     |                     v
User -> View <------update----- Presenter <---get date---- Model
```

MVVM (Model View View-Model):
```
         +------data binding--------+     +------update---------+
         |                          |     |                     |
         |                          v     |                     v
User -> View <------notify-------- View Model <----notify----- Model
```

VIPER (View, Interactor, Presenter, Entity, Router):
```
                                      Router 
                                        |
         +------data binding--------+   |    +-------update---------+
         |                          |   |    |                      |
         |                          v   v    |                      v
User -> View <------notify--------  Presenter  <-----notify------ Interactor
                                                                  |      ^
                                                               notify    |
                                                                  |    manage
                                                                  v      |
                                                                   Entity
```

