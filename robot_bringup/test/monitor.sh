#!/bin/bash

# 1. 把你需要监控的 Topic 全列在这里，想加几个加几个
TOPICS=(
    "/t265_front/pose"
    "/t265_front/imu"
    "/t265_front/image0"
    "/t265_front/image1"
    "/camera_front/image"
)

SESSION="ros2_hz_dashboard"

# 2. 如果之前有同名监控任务没关干净，直接强杀，保证干净启动
tmux kill-session -t $SESSION 2>/dev/null

# 3. 创建一个新的后台 Tmux 会话
tmux new-session -d -s $SESSION

# 4. 在第一个原生窗口跑第一个 Topic
tmux send-keys -t $SESSION "ros2 topic hz ${TOPICS[0]}" C-m

# 5. 遍历剩下的 Topic，不断切分屏幕
for i in "${!TOPICS[@]}"; do
    if [ $i -eq 0 ]; then continue; fi
    # 切分出一个新窗口
    tmux split-window -t $SESSION
    # 发送原生测频指令
    tmux send-keys -t $SESSION "ros2 topic hz ${TOPICS[$i]}" C-m
    # 🌟 核心魔法：每次切分后，自动重新排版成最完美的“平铺矩阵（Tiled）”
    tmux select-layout -t $SESSION tiled
done

# 6. 把咱们的视角直接切进这个做好的监控大屏里！
tmux attach-session -t $SESSION