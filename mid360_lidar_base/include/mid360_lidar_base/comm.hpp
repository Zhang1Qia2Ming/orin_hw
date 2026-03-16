#ifndef MID360_LIDAR_BASE_COMM_HPP_
#define MID360_LIDAR_BASE_COMM_HPP_

#include <queue>
#include <vector>
#include <cstdint>
#include <list>
#include <mutex>

namespace sensor_base {

// SDK related
typedef enum {
    kIndustryLidarType = 1,
    kVehicleLidarType = 2,
    kDirectLidarType = 4,
    kLivoxLidarType = 8
} LidarProtoType;


/* About Extrinsic Parameter */
typedef struct {
  float roll;  /**< Roll angle, unit: degree. */
  float pitch; /**< Pitch angle, unit: degree. */
  float yaw;   /**< Yaw angle, unit: degree. */
  int32_t x;   /**< X translation, unit: mm. */
  int32_t y;   /**< Y translation, unit: mm. */
  int32_t z;   /**< Z translation, unit: mm. */
} ExtParameter;

typedef float TranslationVector[3]; /**< x, y, z translation, unit: mm. */
typedef float RotationMatrix[3][3];

typedef struct {
  TranslationVector trans;
  RotationMatrix rotation;
} ExtParameterDetailed;

typedef struct {
  LidarProtoType lidar_type;
  uint32_t handle;
  ExtParameter param;
} LidarExtParameter;



typedef struct {
    float x;
    float y;
    float z;
    float intensity;
    uint8_t tag;
    uint8_t line;
    uint64_t offset_time;
} PointXyzlt;


typedef struct {
    LidarProtoType lidar_type;
    uint32_t handle;
    uint64_t base_time;
    uint32_t points_num;
    std::vector<PointXyzlt> points;
} StoragePacket;

typedef struct {
    StoragePacket *storage_packet;
    volatile uint32_t rd_idx;
    volatile uint32_t wr_idx;
    uint32_t mask;
    uint32_t size; /**< must be power of 2. */
} LidarDataQueue;

// LidarImuDataQueue imu_data;

typedef struct {
  uint32_t handle;
  uint8_t lidar_type; ////refer to LivoxLidarType
  uint32_t points_num;
  PointXyzlt* points;
} PointPacket;



inline static bool IsPowerOf2(uint32_t size) {
    return (size != 0) && ((size & (size - 1)) == 0);
}

inline static uint32_t RoundupPowerOf2(uint32_t size) {
    uint32_t power2_val = 0;
    for (int i = 0; i < 32; i++) {
        power2_val = ((uint32_t)1) << i;
        if (size <= power2_val) {
        break;
        }
    }
    return power2_val;
}


/** queue operate function */
bool InitQueue(LidarDataQueue *queue, uint32_t queue_size);
bool DeInitQueue(LidarDataQueue *queue);
void ResetQueue(LidarDataQueue *queue);
bool QueuePrePop(LidarDataQueue *queue, StoragePacket *storage_packet);
void QueuePopUpdate(LidarDataQueue *queue);
bool QueuePop(LidarDataQueue *queue, StoragePacket *storage_packet);
uint32_t QueueUsedSize(LidarDataQueue *queue);
uint32_t QueueUnusedSize(LidarDataQueue *queue);
bool QueueIsFull(LidarDataQueue *queue);
bool QueueIsEmpty(LidarDataQueue *queue);
uint32_t QueuePushAny(LidarDataQueue *queue, uint8_t *data, const uint64_t base_time);




typedef struct {
  float gyro_x;        /**< Gyroscope X axis, Unit:rad/s */
  float gyro_y;        /**< Gyroscope Y axis, Unit:rad/s */
  float gyro_z;        /**< Gyroscope Z axis, Unit:rad/s */
  float acc_x;         /**< Accelerometer X axis, Unit:g */
  float acc_y;         /**< Accelerometer Y axis, Unit:g */
  float acc_z;         /**< Accelerometer Z axis, Unit:g */
} RawImuPoint;

typedef struct {
  uint8_t lidar_type;
  uint32_t handle;
  uint8_t slot;
  // union {
  //   uint8_t handle;
  //   uint8_t slot;
  // };
  uint64_t time_stamp;
  float gyro_x;        /**< Gyroscope X axis, Unit:rad/s */
  float gyro_y;        /**< Gyroscope Y axis, Unit:rad/s */
  float gyro_z;        /**< Gyroscope Z axis, Unit:rad/s */
  float acc_x;         /**< Accelerometer X axis, Unit:g */
  float acc_y;         /**< Accelerometer Y axis, Unit:g */
  float acc_z;         /**< Accelerometer Z axis, Unit:g */
} ImuData;

class LidarImuDataQueue {
 public:
  void Push(ImuData* imu_data);
  bool Pop(ImuData& imu_data);
  bool Empty();
  void Clear();

 private:
  std::mutex mutex_;
  std::list<ImuData> imu_data_queue_;
};

} // namespace sensor_base

#endif // MID360_LIDAR_BASE_COMM_HPP_