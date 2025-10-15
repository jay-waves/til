SELinux (Security-Enhanced Linux) 由 NSA 和 RedHat 在 2000 年开发, 在 Linux2.6 中合并到主线 LSM (Linux Security Module) 接口中. 在传统的[自主访问控制 (DAC)](../../../Security/安全模型.md) 基础上, 加入[强制访问控制 (MAC)](../../../Security/安全模型.md).

对于每个对象 (进程, 文件), 都有一个安全标签: `user:role:type:level`. 
- user:role (用于 RBAC, 基于用户角色的限制)
- type: 用于强制访问控制 (Type Enforcement)
- level: 用来表示机密等级 (Multi-Level Security)

```
> ls -Zd /srv/myweb
unconfined_u:object_r:httpd_sys_content_t:s0 /srv/myweb
```

注意, 访问控制而不是*资源控制*, 资源控制请使用 cgroup.