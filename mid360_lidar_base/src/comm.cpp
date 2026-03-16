#include "mid360_lidar_base/comm.hpp"

#include <string>
#include <cstring>


namespace sensor_base {

/** Common function ---------- */




/** queue operate function start */
bool InitQueue(sensor_base::LidarDataQueue *queue, uint32_t queue_size) {
    if(queue == nullptr) {
        return false;
    }

    if(!IsPowerOf2(queue_size)) {
        queue_size = RoundupPowerOf2(queue_size);
        printf("Init queue, real query size:%u.\n", queue_size);
    }

    if(queue->storage_packet) {
        delete[] queue->storage_packet;
        queue->storage_packet = nullptr;
    }

    queue->storage_packet = new StoragePacket[queue_size];
    if (queue->storage_packet == nullptr) {
        // ROS_WARN("RosDriver Queue: Initialization failed - failed to allocate memory.");
        return false;
    }

    queue->rd_idx = 0;
    queue->wr_idx = 0;
    queue->size = queue_size;
    queue->mask = queue_size - 1;

    return true;
}

bool DeInitQueue(LidarDataQueue *queue) {
    if(queue == nullptr) {
        return false;
    }

    if(queue->storage_packet) {
        delete[] queue->storage_packet;
    }

    queue->rd_idx = 0;
    queue->wr_idx = 0;
    queue->size = 0;
    queue->mask = 0;

    return true;
}

void ResetQueue(LidarDataQueue *queue) {
    queue->rd_idx = 0;
    queue->wr_idx = 0;
}

bool QueuePrePop(LidarDataQueue *queue, StoragePacket *storage_packet) {
    if(queue == nullptr || storage_packet == nullptr) {
        return false;
    }

    if(QueueIsEmpty(queue)) {
        return false;
    }

    uint32_t rd_idx = queue->rd_idx;
    storage_packet->base_time = queue->storage_packet[rd_idx].base_time;
    storage_packet->points_num = queue->storage_packet[rd_idx].points_num;
    storage_packet->points.resize(storage_packet->points_num);

    memcpy(storage_packet->points.data(), queue->storage_packet[rd_idx].points.data(), (storage_packet->points_num) * sizeof(PointXyzlt));

    return true;
}

void QueuePopUpdate(LidarDataQueue *queue) {
    queue->rd_idx++;
}

bool QueuePop(LidarDataQueue *queue, StoragePacket *storage_packet) {
    if(!QueuePrePop(queue, storage_packet)) {
        return false;
    }
    QueuePopUpdate(queue);

    return true;
}

uint32_t QueueUsedSize(LidarDataQueue *queue) {
    return (queue->wr_idx - queue->rd_idx);
}

uint32_t QueueUnusedSize(LidarDataQueue *queue) {
    return (queue->size - QueueUsedSize(queue));
}

bool QueueIsFull(LidarDataQueue *queue) {
    return ((queue->wr_idx - queue->rd_idx) > queue->mask);
}

bool QueueIsEmpty(LidarDataQueue *queue) {
    return (queue->wr_idx == queue->rd_idx);
}

uint32_t QueuePushAny(LidarDataQueue *queue, uint8_t *data, const uint64_t base_time) {
    uint32_t wr_idx = queue->wr_idx & queue->mask;
    PointPacket* lidar_point_data = reinterpret_cast<PointPacket*>(data);
    queue->storage_packet[wr_idx].base_time = base_time;
    queue->storage_packet[wr_idx].points_num = lidar_point_data->points_num;

    queue->storage_packet[wr_idx].points.clear();
    queue->storage_packet[wr_idx].points.resize(lidar_point_data->points_num);
    memcpy(queue->storage_packet[wr_idx].points.data(), lidar_point_data->points, sizeof(PointXyzlt) * (lidar_point_data->points_num));

    queue->wr_idx++;
    return 1;
}
/** queue operate function end */


/** imu data queue operate function start */
void LidarImuDataQueue::Push(ImuData* imu_data) {
  ImuData data;
  data.lidar_type = imu_data->lidar_type;
  data.handle = imu_data->handle;
  data.time_stamp = imu_data->time_stamp;

  data.gyro_x = imu_data->gyro_x;
  data.gyro_y = imu_data->gyro_y;
  data.gyro_z = imu_data->gyro_z;

  data.acc_x = imu_data->acc_x;
  data.acc_y = imu_data->acc_y;
  data.acc_z = imu_data->acc_z;

  std::lock_guard<std::mutex> lock(mutex_);
  imu_data_queue_.push_back(std::move(data));
}

bool LidarImuDataQueue::Pop(ImuData& imu_data) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (imu_data_queue_.empty()) {
    return false;
  }
  imu_data = imu_data_queue_.front();
  imu_data_queue_.pop_front();
  return true;
}

bool LidarImuDataQueue::Empty() {
  std::lock_guard<std::mutex> lock(mutex_);
  return imu_data_queue_.empty();
}

void LidarImuDataQueue::Clear() {
  std::list<ImuData> tmp_imu_data_queue;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    imu_data_queue_.swap(tmp_imu_data_queue);
  }
}


} // namespace sensor_base


// bool QueuePrePop(LidarDataQueue *queue, StoragePacket *storage_packet);
// void QueuePopUpdate(LidarDataQueue *queue);
// bool QueuePop(LidarDataQueue *queue, StoragePacket *storage_packet);
// uint32_t QueueUsedSize(LidarDataQueue *queue);
// uint32_t QueueUnusedSize(LidarDataQueue *queue);
// bool QueueIsFull(LidarDataQueue *queue);
// bool QueueIsEmpty(LidarDataQueue *queue);
// uint32_t QueuePushAny(LidarDataQueue *queue, uint8_t *data, const uint64_t base_time);