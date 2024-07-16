Lightweight Communications and Marshalling (LCM)

LCM is a set of libraries and tools for message passing and data marshalling, targeted at real-time systems where high-bandwidth and low latency are critical. It provides a publish/subscribe message passing model and automatic marshalling/unmarshalling code generation with bindings for applications in a variety of programming languages.

> LCM 是一套用于消息传递和数据转储的库和工具，主要针对高带宽和低延迟的实时系统。它提供了一个发布/订阅消息传递模型，并能自动生成数据转储/解转储代码，还能与各种编程语言的应用程序绑定。

# Quick Links

- [LCM downloads](https://github.com/lcm-proj/lcm/releases)
- [Website and documentation](http://lcm-proj.github.io)

# Features

- Low-latency inter-process communication
- Efficient broadcast mechanism using UDP Multicast
- Type-safe message marshalling
- User-friendly logging and playback
- No centralized "database" or "hub" -- peers communicate directly
- No daemons
- Few dependencies

> - 低延迟进程间通信
> - 使用 UDP 组播的高效广播机制
> - 类型安全的信息传递
> - 用户友好的日志记录和回放
> - 无集中式 "数据库 "或 "中心" -- 对等程序直接通信
> - 无守护进程
> - 依赖性小

## Supported platforms and languages

- Platforms:
  - GNU/Linux
  - OS X
  - Windows
  - Any POSIX-1.2001 system (e.g., Cygwin, Solaris, BSD, etc.)
- Languages
  - C
  - C++
  - C#
  - Go
  - Java
  - Lua
  - MATLAB
  - Python
