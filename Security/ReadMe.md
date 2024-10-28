## 参考

- Internet Security Glossary, [RFC 2828](https://www.ietf.org/rfc/rfc2828.txt) (v1, 2000), [RFC 4949](https://datatracker.ietf.org/doc/html/rfc4949) (v2, 2007)
- [NIST SP 800.53](https://csrc.nist.gov/pubs/sp/800/53/r5/upd1/final). [NIST SP 800-92](https://csrc.nist.gov/pubs/sp/800/92/final). NIST Cybersecurity Framwork (NIST CSF)
- Information Security, cybersecurity and privacy protection: Evaluation criteria for IT security, [ISO/IEC 15408](https://www.iso.org/standard/72891.html) (Edition 4, 2022). Information security management systems, [ISO/IEC 27001](https://www.iso.org/standard/27001) (Edition 3, 2022). Information security controls, [ISO/IEC 27002](https://www.iso.org/standard/75652.html) (Edition 3, 2022). 
- Information Security, cybersecurity and privacy protection -- 
- [ITU-T X.800](https://www.itu.int/ITU-T/recommendations/rec.aspx?rec=3102&lang=en), 1991. ISO 7498-2, 1988.

## 安全目标

基础安全目标 (CIA 不可能三角, 信息安全三要素):
- 保密性 (Confidentiality):  只对授权的实体可见, 包含数据保密性与隐私保密性
- 完整性 (Integrity): 信息未被未经授权地修改或破坏, 包含信息完整性与系统完整性
- 可用性 (Availability): 对授权用户不能拒绝服务或服务中断

其他重要安全目标如下.
- 不可否认性 (也叫不可抵赖性, Non-Repudiation): 发送者或接收者不能否认曾经的收发信息行为.
- 可控性 (也叫可追溯性, Accountability): 确保对系统或信息的访问和修改行为可以被审计和追踪.
- 真实性 (Authenticity): 确保信息的来源是可信的, 并且传输过程未被篡改.
- 新鲜性 (Freshness): 保证数据是新的, 而不是重放的历史消息.
- 隐私 (Privacy)
- 透明性 (Transparency)
- 可靠性 (Reliability)
- 合规性 (Compliance)
- 抗篡改性 (Tamper Resistance)

其他安全属性:
- 授权 (Authorization)
- 审计性 (Accountablility)
- 可追溯性 (Traceability)
- 最小化特权 (Least Privilege)
- 权责分离 (Separation of Duties)
- 数据最小化 (Data Minimization)
- 抗篡改 (Anti-tampering)

???

## OSI 安全框架

[X.800, 1991] 对 OSI 安全框架进行了系统定义, 包括:

- 安全攻击: 任何危及信息系统安全的行为
- 安全机制: 用来检测攻击, 阻止攻击或从攻击状态恢复到正常状态的过程.
- 安全服务: 加强数据处理系统和信息传输的安全性的一种处理过程或通信的服务, 目的在于利用一种或多种**安全机制**进行反攻击.

### 安全威胁和攻击

[RFC 4949, 2007, P22] 对 **Attack** 的定义为:

> An intentional act by which an entity attempts to evade 
> security services and violate the security policy of a system.
> That is, an actual assault on system security that derives from an
> intelligent threat.

[RFC 4949, 2007, P23] 对 **Attack** 进行了分类:
- 按意图分类:
	- 主动攻击 (Active Attack). 尝试直接修改和影响系统的资源和操作.
	- 被动攻击 (Passive Attack). 利用系统的信息, 但是并不直接影响系统资源, 即不影响正常通信也不篡改消息. 被动攻击的目的是获取实施离线攻击 (Off-line Attack) 所需的信息, 离线攻击从目标系统获取信息, 然后在另一个攻击者选择的系统中去分析. **被动攻击难以检测, 易于预防**, 也称为*窃听 (Eavesdropping)*.
- 按攻击的发起地点分类:
	- 内部攻击 (Inside Attack), 被授权的内部人员, 将其权限用于其他非授权目的. 
	- 外部攻击 (Outside Attack), 未授权或不合法的用户, 从互联网攻击系统内部.
- 按攻击的传送方式分类:
	- 直接攻击 (Direct Attack)
	- 间接攻击 (Indirect Attack), 攻击者将数据包发送给第三方, 委托第三方通过某种方式攻击.

```
      + - - - - - - - - - - - - +  + - - - - +  + - - - - - - - - - - -+
      | An Attack:              |  |Counter- |  | A System Resource:   |
      | i.e., A Threat Action   |  | measure |  | Target of the Attack |
      | +----------+            |  |         |  | +-----------------+  |
      | | Attacker |<==================||<=========                 |  |
      | |   i.e.,  |   Passive  |  |         |  | |  Vulnerability  |  |
      | | A Threat |<=================>||<========>                 |  |
      | |  Agent   |  or Active |  |         |  | +-------|||-------+  |
      | +----------+   Attack   |  |         |  |         VVV          |
      |                         |  |         |  | Threat Consequences  |
      + - - - - - - - - - - - - +  + - - - - +  + - - - - - - - - - - -+
```

具体的攻击手段可以参考笔记[安全攻击](安全攻击.md).

### 安全威胁

[RFC 4949, 2007, P251] 对 **Risk** 的定义为:

> An expectation of loss expressed as the probability that a
> particular threat will exploit a particular vulnerability with a
> particular harmful result.

根据 [NIST SP 800-30 Rev.1, 2012], 处理 **Risk** 有四种方式:
1. Risk Avoidance, 通过填补漏洞来消除风险.
2. Risk Transference, 将风险转移到其他系统或实体. 如商业保险和外包服务.
3. Risk Limitations, 实施控制来最小化风险和损失.
4. Risk Assumption, 评估并接受可能的风险.

从经济和技术的角度看, 完全避免和转义风险都是不可行的. 即使部署了所有可用对策, 仍会有参与风险, 因此, 分析风险时会将风险按成本和关键性顺序列出, 针对性应用对策. 完整和定量的风险分析很难, 机构会避免列出可能的安全威胁, 而只制定具体的安全手段.

基本安全威胁: (和基本安全目标相对应)
1. 信息泄露 (Disclosure)
2. 完整性破坏 
3. 拒绝服务   
4. 非法使用 

主要的可实现威胁:
- 渗入类威胁:
	1. 假冒
	2. 旁路控制
	3. 授权侵犯, 即内部攻击.
- 植入类型威胁:
	1. 特洛伊木马 (Trojan Horse)
	2. 缺陷和后门 (Flaws & Trap Door)

潜在威胁:
1. 窃听, Eavesdropping
2. 流量分析, Traffic Analysis
3. 人员操作不慎导致的信息泄露
4. 媒体废弃物导致的信息泄露

### 安全服务及安全机制

[RFC 4949, 2007] 描述道: 

> Security services implement security policies, and are implemented by security mechanisms.

```
		 What Security Services
         Should Be Provided?        +- - - - - - - - - - - - -+
         ^  +- - - - - - - - - - - -| Mission Functions View  |
         |  | Security Policy       |- - - - - - - - - - - - -+
         |  +- - - - - - - - - - - -| Domain Practices View   |
         |  | Security Model        |- - - - - - - - - - - - -+
         |  +- - - - - - - - - - - -| Enclave Services View   |
         |  | Security Architecture |- - - - - - - - - - - - -+
         |  +- - - - - - - - - - - -| Agent Mechanisms View   |
         |  | Security Mechanism    |- - - - - - - - - - - - -+
         v  +- - - - - - - - - - - -| Platform Devices View   |
         How Are Security           +- - - - - - - - - - - - -+
         Services Implemented?
```

[X.800, 1991] 定义了五类安全服务 (security services) 安全服务:
1. 数据保密性, Data Confidentiality.
	- Data confidentiality, 数据内容保密性.
		- Connection confidentiality. 有连接通信 ([TCP](../Network/传输层/TCP.md)), 对整个会话状态和数据分组进行加密. 如 [SSL/TLS](../Network/VPN/SSL.md) 技术.
		- Connectionless confidentiality. 无连接通信 ([UDP](../Network/传输层/UDP.md)), 对单个数据包独立加密. 如 [IPSec ESP](../Network/VPN/IPSec.md) 技术.
		- Selective field confidentiality. 只在必要字段
	- Traffic flow confidentiality, TFC, 流量保密性.
2. 数据完整性, Data Integrity.
	- Connection integrity, 整个连接通信中, 数据单元按顺序传输, 并且不被篡改.
		- with recovery, 通过握手和数据确认机制, 遇到完整性错误时, 请求重传来恢复完整性.
		- without recovery, 遇到完整性错误时, 直接丢弃. 无连接通信默认该方式.
	- Connectionless integrity, 独立数据包的完整性. 如 [IPSec AH](../Network/VPN/IPSec.md) 技术. 
	- Selective field integrity
1. 认证, Authentication, 身份验证. 注意身份认证, 不能保证不可否认, 也不能保证数据完整性.
	- Peer entity authentication. 对等实体身份验证.
	- Data origin authentication. 信息源身份验证.
2. 不可否认性, Nonrepudiation.
	- Non-repudiation with proof of origin. 消息发送者否认发送行为.
	- Non-repudiation with proof of delivery (receipt). 消息接收者否认收到拥有数据.
3. 访问控制, Access Control, 和**授权**休戚相关.


安全服务在技术层面通过**特定安全机制 (security machanisms)** 来实现安全策略:


| <table><tr><th></th><th>安全机制</th></tr><tr><td>安全服务</td><td></td></tr></table> | 加密 | 数字签名 | 访问控制 | 数据完整性 | 认证交换 | 流量填充 | 路由控制 | 公证 (Notarization) |
| ----------------------------------------- | ---- | -------- | -------- | ---------- | -------- | -------- | -------- | ---- |
| 同等实体认证                              | Y    | Y        |          |            | Y        |          |          |      |
| 数据源认证                                | Y    | Y        |          |            |          |          |          |      |
| 访问控制                                  |      |          | Y        |            |          |          |          |      |
| 保密性                                    | Y    |          |          |            |          |          | Y        |      |
| 流量保密性                                | Y    |          |          |            |          | Y        | Y        |      |
| 数据完整性                                | Y    | Y        |          | Y          |          |          |          |      |
| 不可否认性                                |      | Y        |          | Y          |          |          |          | Y    |
| 可用性                                    |      |          |          | Y          | Y         |          |          |      |

[X.800, 1991] 还列有一些不属于特定安全服务的安全机制, 如*事件检测*, *安全审计 (Security Audit Trail)*, *灾难恢复 (Security Recovery)*. 这些安全机制的重要性和所需的安全等级相关.

[X.800, 1991] 和 [RFC 4949, 2007] 都定义了安全服务所处的[网络层次](../Network/网络体系结构.md), 不过随着技术进步和定义变化, 下表在不同标准间区别较大, 仅作参考.

<table border="1" cellspacing="0" cellpadding="5">
    <caption><strong>the relationship of security services and layers</strong></caption>
    <thead>
        <tr>
	        <th rowspan="2" colspan="2">Service</th>
            <th colspan="7">Network Layer</th>
        </tr>
        <tr>
            <th>Physical</th>
            <th>Data Link </th>
            <th>Network</th>
            <th>Transport</th>
            <th>Session</th>
            <th>Presentation</th>
            <th>Application</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td rowspan="2">Authentication</td><td>Peer Entity</td>
            <td></td><td></td><td>Y</td><td>Y</td><td></td><td></td><td>Y</td>
        </tr>
        <tr>
            <td>Data Origin</td>
            <td></td><td></td><td>Y</td><td>Y</td><td></td><td></td><td>Y</td>
        </tr>
        <tr>
            <td>Access Control</td><td></td>
            <td></td><td></td><td>Y</td><td>Y</td><td></td><td></td><td>Y</td>
        </tr>
        <tr>
            <td rowspan="4">Confidentiality</td><td>Stream</td>
            <td>Y</td><td>Y</td><td>Y</td><td>Y</td><td></td><td>Y</td><td>Y</td>
        </tr>
        <tr>
            <td>Datagram</td>
            <td>Y</td><td>Y</td><td>Y</td><td>Y</td><td></td><td>Y</td><td>Y</td>
        </tr>
        <tr>
            <td>Selective Field</td>
            <td></td><td></td><td>Y</td><td></td><td></td><td>Y</td><td>Y</td>
        </tr>
        <tr>
            <td>Traffic Flow</td>
            <td>Y</td><td></td><td>Y</td><td></td><td></td><td></td><td>Y</td>
        </tr>
        <tr>
            <td rowspan="5">Integrity</td><td>Stream with Recovery</td>
            <td></td><td></td><td></td><td>Y</td><td></td><td></td><td>Y</td>
        </tr>
        <tr>
            <td>Stream without Recovery</td>
            <td></td><td></td><td>Y</td><td>Y</td><td></td><td></td><td>Y</td>
        </tr>
        <tr>
            <td>Stream Selective Field</td>
            <td></td><td></td><td></td><td></td><td></td><td></td><td>Y</td>
        </tr>
        <tr>
            <td>Datagram</td>
            <td>Y</td><td>Y</td><td>Y</td><td>Y</td><td></td><td></td><td>Y</td>
        </tr>
        <tr>
            <td>Datagram Selective Field</td>
            <td></td><td></td><td></td><td></td><td></td><td></td><td>Y</td>
        </tr>
        <tr>
            <td rowspan="2">Non-repudiation</td><td>of Origin</td>
            <td></td><td></td><td></td><td></td><td></td><td></td><td>Y</td>
        </tr>
        <tr>
            <td>of Delivery (Receipt)</td>
            <td></td><td></td><td></td><td></td><td></td><td></td><td>Y</td>
        </tr>
    </tbody>
</table>

