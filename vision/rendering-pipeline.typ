
#import "../appx/theme.typ": tufte, note, mermaid
#show: tufte

#let bmat(..args) = math.mat(delim: "[", ..args)
#let vec(x) = math.bold(math.upright(x))

#mermaid(
  ```mermaid
    graph TD;
    Application-->Geometry;
    Geometry-->Rasterizer;
  ```.text, 
  width: 30%
)

```mermaid
flowchart TB
    subgraph Provisioning["Proxy Provisioning"]
        Providers["Subscription Servers"]
        Subconverter["Subconverter"]
        Nodes["Proxy Providers"]

        Providers --> Subconverter
        Subconverter -->|"proxy-providers"| Nodes
    end

    subgraph RuleProvisioning["Rule Provisioning"]
        RuleProviders["Remote Servers"]
        RuleProvider["Rule Providers"]

        RuleProviders -->|"rule-providers"| RuleProvider
    end

    subgraph Routing["Traffic Routing"]
        User["User Traffic"]
        Inbound["Mihomo Inbound: TUN / Mixed"]
        Metadata["Destination: Domain / IP / Process"]
        Rules["Routing Rules"]
        Groups["Proxy Groups"]
        SelectedNode["Selected Proxy Node"]
        Remote["Remote Proxy Server"]
        Destination["Destination Website"]
        Reject["Rejected"]
        DNS["DNS Resolver"]

        User --> Inbound
        Inbound --> Metadata
        Metadata --> Rules

        Rules -->|"PROXY"| Groups
        Groups --> SelectedNode
        SelectedNode --> Remote
        Remote --> Destination

        Rules -->|"DIRECT"| Destination
        Rules -->|"REJECT"| Reject

        Inbound -.->|"DNS queries"| DNS
        DNS -.->|"Domain / IP result"| Metadata
    end

    RuleProvider -.-> |"third-party rules"| Rules
    Nodes -.->|"Available nodes"| Groups

```
