"""NexArm 主从遥操作链路与延迟测量。"""

# 两块板的 CH340 没有序列号，只能按 USB 口区分；换口后要改这里（scripts/probe_ports.py 可查哪个口是哪条臂）。
# 2026-11-15 起：主臂插主板 11 号口，从臂插主板 4 号口后面的板载集线器；两个相机在另一个外接集线器上。
LEADER_PORT = "/dev/serial/by-path/pci-0000:00:14.0-usb-0:11:1.0-port0"
FOLLOWER_PORT = "/dev/serial/by-path/pci-0000:00:14.0-usb-0:4.4:1.0-port0"
