# Ubuntu 安装与使用 Claude Code 指南

## 第一步：安装 Node.js 环境

Claude Code 需要 Node.js 22+。Ubuntu 自带的版本通常不够新，推荐用 nvm 安装：

```bash
# 安装 nvm
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.3/install.sh | bash

# 重新加载 shell 环境（或关掉终端重开）
source ~/.bashrc

# 安装 Node.js 22 LTS
nvm install 22
nvm use 22
```

验证安装：

```bash
node --version   # 应该显示 v22.x.x
npm --version    # 应该显示 10.x.x
```

## 第二步：安装 Claude Code

```bash
npm install -g @anthropic-ai/claude-code
```

安装完后验证：

```bash
claude --version
```

在~/.claude.json 里添加配置 注意逗号 英文的逗号 "hasCompletedOnboarding": true
在~/.claude/setting.json里
环境配置
"env":
{ "ANTHROPIC_BASE_URL": "https://api.deepseek.com/anthropic", "ANTHROPIC_AUTH_TOKEN": "你的deepseekapikey", "ANTHROPIC_MODEL": "deepseek-v4-pro[1m]", "ANTHROPIC_DEFAULT_OPUS_MODEL": "deepseek-v4-pro[1m]", "ANTHROPIC_DEFAULT_SONNET_MODEL": "deepseek-v4-pro[1m]", "ANTHROPIC_DEFAULT_HAIKU_MODEL": "deepseek-v4-flash", "CLAUDE_CODE_SUBAGENT_MODEL": "deepseek-v4-flash", "CLAUDE_CODE_EFFORT_LEVEL": "max" }
## 第三步：获取 API Key

Claude Code 通过 Anthropic API 调用模型，需要一个 API Key。

### 方式一：官方 Anthropic API（推荐）

1. 打开 [https://console.anthropic.com](https://console.anthropic.com)
2. 注册 / 登录账号
3. 进入 **API Keys** 页面，点击 **Create Key**
4. 复制生成的 key（格式为 `sk-ant-api03-...`）

### 方式二：第三方代理 API

如果你用的是代理服务（如 xunsuan 等），需要拿到服务商给你的：
- API Key（通常 `sk-` 开头）
- API Base URL

## 第四步：配置环境变量

把你的环境变量写入 `~/.bashrc`，这样每次开机都能用：

```bash
nano ~/.bashrc
```

在文件末尾添加：

```bash
# --- Claude Code ---
export ANTHROPIC_AUTH_TOKEN="sk-ant-api03-你的API密钥"
```

如果用的是第三方代理，还需要加一行：

```bash
export ANTHROPIC_BASE_URL="https://你的代理地址"
```

保存后生效：

```bash
source ~/.bashrc
```

> **安全提示**：不要把 API Key 硬编码在项目的 `settings.json` 里，存在 `~/.bashrc` 用环境变量引用更安全。`~/.bashrc` 不会被 git 追踪。

验证环境变量：

```bash
echo $ANTHROPIC_AUTH_TOKEN
```

## 第五步：启动和使用

进入你的项目目录，直接运行：

```bash
cd ~/你的项目路径
claude
```

进去之后就是一个交互式对话界面，你可以：

- 直接问代码问题
- 让它帮你写代码、修 bug
- 让它搜索和解释代码库

### 基本操作

| 操作 | 按键 / 命令 |
|------|------------|
| 发送消息 | `Enter` |
| 换行 | `Ctrl+Enter` 或 `Shift+Enter` |
| 退出 | `/exit` 或 `Ctrl+D` |
| 切换模型 | `/model` |
| 查看权限配置 | `/permissions` |
| 切换暗/亮主题 | `/config` |
| 清除对话历史 | `/clear` |
| 查看帮助 | `/help` |
| 接受建议的修改 | `y` |
| 拒绝修改 | `n` |

## 第六步：项目配置（可选）

在项目根目录创建 `.claude/settings.json`，可以对当前项目做定制：

```bash
mkdir -p .claude
```

```json
{
  "env": {
    "ANTHROPIC_AUTH_TOKEN": "$ANTHROPIC_AUTH_TOKEN",
    "ANTHROPIC_BASE_URL": "$ANTHROPIC_BASE_URL"
  },
  "permissions": {
    "allow": [
      "Bash(git:*)",
      "Read",
      "Edit",
      "Write",
      "WebFetch",
      "WebSearch"
    ],
    "deny": [
      "Bash(sudo:*)",
      "Bash(rm -rf:*)"
    ]
  }
}
```

> `allow` 里的命令会被自动批准、不再弹确认框；`deny` 里的命令会被直接拒绝。根据你的使用习惯调整。

## 常用工作流

### 让 Claude 了解你的项目

```bash
claude
> 帮我生成 CLAUDE.md 文档
```

Claude 会扫描项目结构，写一个 `CLAUDE.md`，后续每次会话自动加载，回答会更准确。

### 写代码

```
> 在 src/utils 里写一个日志工具函数，支持 INFO/WARN/ERROR 三个级别
```

### 修 bug

```
> 运行 src/xxx.cpp 的时候 segfault 了，帮我排查一下
```

### 代码审查

```
> review 我最近的改动
```

### 搜代码

```
> 项目里所有调用 publish() 的地方在哪里？
```

## 更新 Claude Code

```bash
npm update -g @anthropic-ai/claude-code
```

查看当前版本和最新版本：

```bash
npm list -g @anthropic-ai/claude-code
npm view @anthropic-ai/claude-code version
```

## 注意事项

- Claude Code 会在项目里直接修改文件，建议**重要改动前先 git commit**，方便回滚。
- `.claude/` 目录会自动生成，建议加入 `.gitignore`（但 `settings.json` 可能需要团队共享的话就别加）。
- 如果遇到网络问题（大陆访问 Anthropic API），需要配置代理服务，把 `ANTHROPIC_BASE_URL` 指向代理地址。
- 免费额度和付费策略以 [Anthropic 官网](https://docs.anthropic.com/en/docs/claude-code/pricing) 为准。
