# 机载 PC SSH 远程连接配置指南

## 第一步：在机载 PC 上安装 SSH 服务

在机载 PC 的终端里执行：

```bash
sudo apt update
sudo apt install openssh-server -y
sudo systemctl enable --now ssh
```

查一下机载 PC 的 IP 地址：

```bash
ip addr show | grep "inet "
```

会看到类似 `192.168.1.100` 或 `10.167.191.188` 这样的地址，记下来。

## 第二步：在本地机器上配置 SSH

打开本地机器终端，编辑 SSH 配置文件：

```bash
mkdir -p ~/.ssh
nano ~/.ssh/config
```

写入以下内容（把 IP 和用户名换成你自己的）：

```
Host onboard
    HostName <机载PC的IP>
    User <机载PC上的用户名>
    Port 22
```

保存退出（`Ctrl+O` 回车，`Ctrl+X`）。

## 第三步：配置免密登录

在本地终端执行：

```bash
ssh-keygen -t ed25519 -f ~/.ssh/id_ed25519 -N ""
ssh-copy-id onboard
```

`ssh-copy-id` 这一步会问你要一次机载 PC 的密码，输入后之后就再也不用了。

## 第四步：验证连接

```bash
ssh onboard
```

进去了就是机载 PC 的终端。

### 常用命令速查

| 命令 | 作用 |
|------|------|
| `ssh onboard` | 登录到机载 PC |
| `ssh onboard "ls ~/"` | 不登录，远程执行一条命令 |
| `exit` 或 `Ctrl+D` | 断开连接 |
| `scp file.txt onboard:~/` | 把本地文件传到远程 |
| `scp onboard:~/file.txt ./` | 把远程文件拉到本地 |

## 第五步：VS Code / Cursor / Trae 连接

1. 安装 **Remote-SSH** 扩展
2. 点左下角绿色 `><` 图标
3. `Connect to Host...` → 选择 `onboard`
4. 连上后 `Open Folder` → 输入远程项目路径（如 `/home/<用户名>/wulin_r2`）

之后开发就和在本地写代码一样，文件浏览、终端、编译都在机载 PC 上执行，只传文本不传画面。

## 注意事项

- 如果机载 PC 用的是 WiFi DHCP 分配的 IP，路由器重启后 IP 可能会变。连不上时去机载 PC 上重新执行 `ip addr show` 确认 IP，更新 `~/.ssh/config` 里的 `HostName`。