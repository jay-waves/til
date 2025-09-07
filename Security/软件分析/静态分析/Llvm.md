Llvm 自带的 Analysis Pass:
- call graph analysis
- alias analysis
- data flow 
- control flow 
- interprocedural analysis 

CFG (Loop, SCEV, LCSSA) --> SCCP --> AAManager / MemorySSA / LVI --> Interprocedural Taint Analysis