### 1.1 Normal-Form Pepresentation of Games 

* strictly dominated strategies
* iterated elimination of strictly dominated strategies 
* Nash equilibrium

### 1.2 Prisoner's Dilemma

### Definition: Nash Equilibrium

In the $n$-player normal-form game $G = \{ S_1, \ldots, S_n; u_1, \ldots, u_n \}$, a strategy profile $(s_{1}^*, \ldots, s_{n}^*)$ is a **Nash equilibrium** if, for each player $i$, the strategy $s_i^*$ is (at least tied for) player $i$’s best response to the strategies chosen by the other $n-1$ players $(s_1^*, \ldots, s_{i-1}^*, s_{i+1}^*, \ldots, s_n^*)$. That is,

$$  
u_i(s_1^*, \ldots, s_{i-1}^*, s_i^*, s_{i+1}^*, \ldots, s_n^*)  
\;\ge\;  
u_i(s_1^*, \ldots, s_{i-1}^*, s_i, s_{i+1}^*, \ldots, s_n^*)  
$$

for every feasible strategy $s_i \in S_i$.

Equivalently, $s_i^*$ solves

$$  
\max_{s_i \in S_i} \; 
u_i(s_1^*, \ldots, s_{i-1}^*, s_i, s_{i+1}^*, \ldots, s_n^*).  
$$

