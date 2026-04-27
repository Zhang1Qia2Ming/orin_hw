#pragma once

#include <atomic>
#include <Eigen/Dense>

struct PoseSnapshot {
    uint64_t timestamp_ns;
    Eigen::Vector3f translation;
    Eigen::Quaternionf rotation;
};

class LockFreePoseBuffer {
public:
    LockFreePoseBuffer() {
        head_.store(0);
        for(int i=0; i<BUFFER_SIZE; ++i) {
            buffer_[i].timestamp_ns = 0;
        }
    }

    // ================= IMU 线程调用：高频压入 =================
    void Push(uint64_t timestamp_ns, const Eigen::Affine3f& pose) {
        // 计算下一个写入索引
        size_t next_idx = (head_.load(std::memory_order_relaxed) + 1) % BUFFER_SIZE;
        
        // 🚨 必须先写数据，再更新 Head 指针！
        buffer_[next_idx].timestamp_ns = timestamp_ns;
        buffer_[next_idx].translation = pose.translation();
        buffer_[next_idx].rotation = Eigen::Quaternionf(pose.linear());
        
        // 使用 release 语义：保证在 head 更新前，上面的数据写入对其他线程绝对可见
        head_.store(next_idx, std::memory_order_release);
    }

    // ================= Lidar 线程调用：查询与球面插值 =================
    // 输入一个纳秒时间戳，返回该微秒的绝对平滑姿态
    Eigen::Affine3f QueryInterpolatedPose(uint64_t target_ts) {
        // 使用 acquire 语义读取最新的 head，构成内存屏障
        size_t current_head = head_.load(std::memory_order_acquire);
        
        if (buffer_[current_head].timestamp_ns == 0) {
            return Eigen::Affine3f::Identity(); // 还没数据
        }

        // 1. 查找 target_ts 所在的区间 [t_prev, t_next]
        size_t idx = current_head;
        size_t prev_idx = idx;
        
        // 往前找，直到找到一个时间戳 <= target_ts 的快照
        for (size_t i = 0; i < BUFFER_SIZE; ++i) {
            if (buffer_[idx].timestamp_ns <= target_ts) {
                break;
            }
            prev_idx = idx; // 记录它的前一个(时间戳大于 target 的那个)
            idx = (idx - 1 + BUFFER_SIZE) % BUFFER_SIZE; // 环形减1
            
            // 如果找了一圈发现最老的数据都比 target_ts 新，说明 Lidar 数据太旧了，脱节了
            if (buffer_[idx].timestamp_ns == 0 || i == BUFFER_SIZE - 1) {
                return ConstructAffine(buffer_[current_head]); // 直接返回最新值
            }
        }

        // 如果正好精确命中
        if (buffer_[idx].timestamp_ns == target_ts || idx == prev_idx) {
            return ConstructAffine(buffer_[idx]);
        }

        // 2. 球面线性插值 (Slerp) 运算
        const auto& p1 = buffer_[idx];      // 过去的时间点
        const auto& p2 = buffer_[prev_idx]; // 未来的时间点
        
        // 防止除零保护
        double dt = static_cast<double>(p2.timestamp_ns - p1.timestamp_ns);
        if (dt <= 0) return ConstructAffine(p1);

        // 计算插值系数 alpha [0, 1]
        double alpha = static_cast<double>(target_ts - p1.timestamp_ns) / dt;

        // 旋转：四元数球面插值
        Eigen::Quaternionf q_interp = p1.rotation.slerp(alpha, p2.rotation);
        
        // 平移：线性插值
        Eigen::Vector3f t_interp = p1.translation + alpha * (p2.translation - p1.translation);

        // 组装返回
        Eigen::Affine3f result = Eigen::Affine3f::Identity();
        result.translation() = t_interp;
        result.linear() = q_interp.toRotationMatrix();
        return result;
    }

private:
    static constexpr size_t BUFFER_SIZE = 1024; // 1024 帧 200Hz 数据，约 5 秒的历史长度
    PoseSnapshot buffer_[BUFFER_SIZE];
    std::atomic<size_t> head_;

    // 辅助函数：把结构体转成 Affine3f
    inline Eigen::Affine3f ConstructAffine(const PoseSnapshot& snap) {
        Eigen::Affine3f pose = Eigen::Affine3f::Identity();
        pose.translation() = snap.translation;
        pose.linear() = snap.rotation.toRotationMatrix();
        return pose;
    }
};