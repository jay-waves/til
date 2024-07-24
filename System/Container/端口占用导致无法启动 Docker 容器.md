https://stackoverflow.com/questions/65272764/ports-are-not-available-listen-tcp-0-0-0-0-50070-bind-an-attempt-was-made-to

管理员模式运行CMD, 重启一下Winnat, 把端口腾出来. 可能是Clash把本地端口占满了没释放.

```
net stop winnat
net start winnat
```