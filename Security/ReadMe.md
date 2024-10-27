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

安全威胁的基本目标是使: 信息泄露, 完整性破坏, 拒绝服务, 非法使用. 和基本安全目标相对应.

**被动攻击**, 又称为**窃听** (Eavesdropping), 指以获取信息为下目的, 攻击系统**保密性**, 但不影响正常通信, 不篡改消息的攻击. 难以检测, 易于预防. 主要分为以下两类:
- 信息内容泄露, Release Message Contents 
- 流量分析, Traffic Analysis

**主动攻击**, 指对数据流进行篡改和假冒. 难以预防, 易于检测. 分类:
- 拒绝服务, Denial of Service: 攻击**可用性**
- 消息篡改, Modification: 攻击**完整性**
- 伪装, Masquerade: 攻击**真实性**
- 重放, Replay. 防御手段有序列号机制, 挑战应答机制, 时间戳(全局时钟)机制.

口令碰撞和窃取, 欺骗, 缺陷和后门 (trapdoor), 认证失效, 协议缺陷, 信息泄露, 病毒和蠕虫, 拒绝服务, 木马 (Trojan Horse), 内部攻击者.

[安全漏洞](安全漏洞.md)

???

### 安全服务及安全机制

[X.800, 1991] 定义了五类安全服务 (security services) 安全服务:
1. 数据保密性, Data Confidentiality.
	- Data confidentiality, 数据内容保密性.
		- Connection confidentiality. 有连接通信 ([TCP](../Network/传输层/TCP.md)), 对整个会话状态和数据分组进行加密. 如 [SSL/TLS](../Network/VPN/SSL.md) 技术.
		- Connectionless confidentiality. 无连接通信 ([UDP](../Network/传输层/UDP.md)), 对单个数据包独立加密. 如 [IPSec ESP](../Network/VPN/IPSec.md) 技术.
		- Selective field confidentiality. 只在必要字段
	- Traffic flow confidentiality, 流量保密性.
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
	- Non-repudiation with proof of origin.
	- Non-repudiation with proof of delivery.
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

安全服务所处的[网络层次](../Network/网络体系结构.md):

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
            <td rowspan="2">Authenticatio</td><td>Peer entity authentication</td>
            <td></td><td></td><td>Y</td><td>Y</td><td></td><td></td><td>Y</td>
        </tr>
        <tr>
            <td>Data origin authentication</td>
            <td></td><td></td><td>Y</td><td>Y</td><td></td><td></td><td>Y</td>
        </tr>
        <tr>
            <td>Access Control</td><td></td>
            <td></td><td></td><td>Y</td><td>Y</td><td></td><td></td><td>Y</td>
        </tr>
        <tr>
            <td rowspan="4">Confidentiality</td><td>Connection confidentiality</td>
            <td>Y</td><td>Y</td><td>Y</td><td>Y</td><td></td><td>Y</td><td>Y</td>
        </tr>
        <tr>
            <td>Connectionless confidentiality</td>
            <td></td><td>Y</td><td>Y</td><td>Y</td><td></td><td>Y</td><td>Y</td>
        </tr>
        <tr>
            <td>Selective field confidentiality</td>
            <td></td><td></td><td></td><td></td><td></td><td>Y</td><td>Y</td>
        </tr>
        <tr>
            <td>Traffic flow confidentiality</td>
            <td>Y</td><td></td><td>Y</td><td></td><td></td><td></td><td>Y</td>
        </tr>
        <tr>
            <td rowspan="5">Integrity</td><td>Connection Integrity with recovery</td>
            <td></td><td></td><td></td><td>Y</td><td></td><td></td><td>Y</td>
        </tr>
        <tr>
            <td>Connection integrity without recovery</td>
            <td></td><td></td><td>Y</td><td>Y</td><td></td><td></td><td>Y</td>
        </tr>
        <tr>
            <td>Selective field connection integrity</td>
            <td></td><td></td><td></td><td></td><td></td><td></td><td>Y</td>
        </tr>
        <tr>
            <td>Connectionless integrity</td>
            <td></td><td></td><td>Y</td><td>Y</td><td></td><td></td><td>Y</td>
        </tr>
        <tr>
            <td>Selective field connectionless integrity</td>
            <td></td><td></td><td></td><td></td><td></td><td></td><td>Y</td>
        </tr>
        <tr>
            <td rowspan="2">Non-repudiation</td><td>Non-repudiation Origin</td>
            <td></td><td></td><td></td><td></td><td></td><td></td><td>Y</td>
        </tr>
        <tr>
            <td>Non-repudiation Delivery</td>
            <td></td><td></td><td></td><td></td><td></td><td></td><td>Y</td>
        </tr>
    </tbody>
</table>
