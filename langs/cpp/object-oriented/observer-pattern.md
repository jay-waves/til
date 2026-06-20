观察者模式 (Observer Pattern) 定义了*一对多的依赖关系*, 当 "一" 的状态发生改变时, 所有依赖于 "一" 的对象都会得到通知并更新. 在分布式系统中极其常见，如 PUB/SUB 结构.

观察者模式不适合处理强顺序的事件管线，可能有循环依赖、同步错误等问题。这时候用责任链模式（管线模式、事件总线）更好。

```ascii
┌──────────────┐
│ Order Service│
└──────┬───────┘
       │ publish event
       │ topic = order.created
       ▼
┌──────────────────────────────┐
│        Message Broker         │
│                              │
│  ┌────────────────────────┐  │
│  │ topic: order.created   │  │
│  │ event queue / log      │  │
│  └───────────┬────────────┘  │
└──────────────┼───────────────┘
               │
      ┌────────┼────────┬─────────────┐
      │        │        │             │
      ▼        ▼        ▼             ▼
┌──────────┐ ┌───────┐ ┌──────────┐ ┌───────────┐
│Inventory │ │ Email │ │ Analytics│ │ Fraud     │
│ Service  │ │Service│ │ Service  │ │ Service   │
└──────────┘ └───────┘ └──────────┘ └───────────┘
      │        │        │             │
      ▼        ▼        ▼             ▼
 reserve   send mail  track KPI   risk check
 stock
 ```