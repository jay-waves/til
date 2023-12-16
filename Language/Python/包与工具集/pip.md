使用`pip install [software]`时报错:
`WARNING: There was an error checking the latest version of pip.`

问题可能是:
1. pip忘记更新, 每次安装前先执行: `python -m pip install --upgrade pip`
2. 没关VPN, 连着VPN不会连接成功.