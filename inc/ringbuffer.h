#ifndef RINGBUFFER_H
#define RINGBUFFER_H
#include <stdint.h>
#include "string.h"
#include <mutex>
#include <condition_variable>
//#define ringbufferlenght 1024
static std::mutex operatemutex;
struct RingBuffer
{
  std::mutex mutex;
  std::condition_variable condVar;
  bool dataReady;

  uint8_t *ringbuffer;
  volatile uint64_t head;
  volatile uint64_t tail;
  uint64_t ringbufferlenght;
  RingBuffer(uint64_t lenght) : head(0), tail(0), ringbufferlenght(lenght)
  {
    ringbuffer = new uint8_t[ringbufferlenght];
    memset(ringbuffer, 0, sizeof(*ringbuffer));
  }

  uint8_t SaveDataToBuffer(const char *data, size_t lenght)
  {

    if (head - tail > ringbufferlenght - lenght)
    {

      return 0;
    }
    if (lenght > ringbufferlenght - (head % ringbufferlenght))
    {
      memcpy(&ringbuffer[head % ringbufferlenght], data, ringbufferlenght - (head % ringbufferlenght));
      memcpy(&ringbuffer[0], data + ringbufferlenght - (head % ringbufferlenght), lenght - ringbufferlenght + (head % ringbufferlenght));
    }
    else
    {
      memcpy(&ringbuffer[head % ringbufferlenght], data, lenght);
    }
    head += lenght;

    return 1;
  }

  uint64_t GetDataFromBuffer(uint8_t *data, size_t lenght = 0)
  {

    uint64_t datalen=0;
    if (head <= tail)
    {
      return 0;
    }

    if (lenght == 0 || head - tail < lenght)
    {
      datalen = head - tail;
    }
    else
    {
      datalen = lenght;
    }
    if (datalen > ringbufferlenght - (tail % ringbufferlenght))
    {
      memcpy(data, &ringbuffer[tail % ringbufferlenght], ringbufferlenght - (tail % ringbufferlenght));
      memcpy(data + ringbufferlenght - (tail % ringbufferlenght), &ringbuffer[0], datalen - ringbufferlenght + (tail % ringbufferlenght));
    }
    else
    {
      memcpy(data, &ringbuffer[tail % ringbufferlenght], datalen);
    }
      tail += datalen;
    return datalen;
  }
//get whith data remian
  uint64_t GetDataFromBufferADD(uint8_t *data, size_t lenght = 0)
  {

    uint64_t datalen;
    if (head <= tail)
    {
      return 0;
    }

    if (lenght == 0 || head - tail < lenght)
    {
      datalen = head - tail + 20;
    }
    else
    {
      datalen = lenght + 20;
    }
    if (datalen > ringbufferlenght - ((tail - 20) % ringbufferlenght))
    {
      memcpy(data, &ringbuffer[tail % ringbufferlenght], ringbufferlenght - ((tail - 20) % ringbufferlenght));
      memcpy(data + ringbufferlenght - ((tail - 20) % ringbufferlenght), &ringbuffer[0], datalen - ringbufferlenght + ((tail - 20) % ringbufferlenght));
    }
    else
    {
      memcpy(data, &ringbuffer[(tail - 20) % ringbufferlenght], datalen);
    }

    return datalen;
  }
  uint64_t Gethead()
  {
    return head;
  }
  uint64_t Gettail()
  {
    return tail;
  }
  uint64_t Getlenght()
  {
    if (head >= tail)
    {
      return head - tail;
    }
    else
    {
      return 0;
    }
  }

  uint8_t *Getbuffer()
  {
    return ringbuffer;
  }
};
#pragma pack(1)

struct ACCRawData
{
  float acc_x;
  float acc_y;
  float acc_z;
};
struct GYRORawData
{
  float gyro_x;
  float gyro_y;
  float gyro_z;
};
struct MAGRawData
{
  float mag_x;
  float mag_y;
  float mag_z;
};

struct GPSRawData
{
  uint32_t latitude;
  uint32_t longitude;
  float velocity;
};
struct ATTITUDEQuaternionData
{
  float Q0;
  float Q1;
  float Q2;
  float Q3;
};

#pragma pack()

template <typename T>
struct RingBufferT
{

  std::mutex mutex;
  std::condition_variable condVar;
  bool dataReady;

  //    std::mutex mutex;
  uint8_t *ringbuffer;
  volatile uint64_t head;
  volatile uint64_t tail;
  uint64_t ringbufferlenght;
  RingBufferT(uint32_t count) : head(0), tail(0), ringbufferlenght(count * sizeof(T))
  {
    ringbuffer = new uint8_t[ringbufferlenght];
    memset(ringbuffer, 0, sizeof(ringbuffer));
  }

    // no copy, assignment, move, move assignment
    RingBufferT(const RingBufferT &) = delete;
    RingBufferT &operator=(const RingBufferT &) = delete;
    RingBufferT(RingBufferT &&) = delete;
    RingBufferT &operator=(RingBufferT &&) = delete;


  uint8_t SaveDataToBuffer(T &data)
  {
    size_t lenght = sizeof(T);

    if (head - tail > ringbufferlenght - lenght)
    {
      return 0;
    }
    if (lenght > ringbufferlenght - (head % ringbufferlenght))
    {
      memcpy(&ringbuffer[head % ringbufferlenght], &data, ringbufferlenght - (head % ringbufferlenght));
      memcpy(&ringbuffer[0], &data + ringbufferlenght - (head % ringbufferlenght), lenght - ringbufferlenght + (head % ringbufferlenght));
    }
    else
    {
      memcpy(&ringbuffer[head % ringbufferlenght], &data, lenght);
    }
    head += lenght;
    return 1;
  }
  uint8_t SaveDataToBuffer(T &data, size_t count)
  {
    size_t lenght = count * sizeof(T);

    if (head - tail > ringbufferlenght - lenght)
    {
      return 0;
    }
    if (lenght > ringbufferlenght - (head % ringbufferlenght))
    {
      memcpy(&ringbuffer[head % ringbufferlenght], &data, ringbufferlenght - (head % ringbufferlenght));
      memcpy(&ringbuffer[0], &data + ringbufferlenght - (head % ringbufferlenght), lenght - ringbufferlenght + (head % ringbufferlenght));
    }
    else
    {
      memcpy(&ringbuffer[head % ringbufferlenght], &data, lenght);
    }
    head += lenght;
    return 1;
  }

  uint64_t GetDataFromBuffer(uint8_t *data, size_t count)
  {
    size_t lenght = count * sizeof(T);
    uint64_t datalen;
    if (head <= tail)
      return 0;
    if (lenght == 0 || head - tail < lenght)
    {
      datalen = head - tail;
    }
    else
    {
      datalen = lenght;
    }
    if (datalen > ringbufferlenght - (tail % ringbufferlenght))
    {
      memcpy(data, &ringbuffer[tail % ringbufferlenght], ringbufferlenght - (tail % ringbufferlenght));
      memcpy(data + ringbufferlenght - (tail % ringbufferlenght), &ringbuffer[0], datalen - ringbufferlenght + (tail % ringbufferlenght));
    }
    else
    {
      memcpy(data, &ringbuffer[tail % ringbufferlenght], datalen);
    }
    tail += datalen;
    return datalen;
  }
  uint64_t Gethead()
  {
    return head;
  }
  uint64_t Gettail()
  {
    return tail;
  }
  uint64_t Getlenght()
  {
    if (head >= tail)
    {
      return head - tail;
    }
    else
    {
      return 0;
    }
  }

  uint8_t *Getbuffer()
  {
    return ringbuffer;
  }

  void push_front(T &item)
  {
    SaveDataToBuffer(item);
  }

  bool pop_back(T *pItem)
  {
    size_t lenght = sizeof(T);
    uint64_t datalen;
    if (head <= tail)
      return false;
    if (head - tail < lenght)
    {
      return false;
    }
    else
    {
      datalen = lenght;
    }
    if (datalen > ringbufferlenght - (tail % ringbufferlenght))
    {
      memcpy(pItem, &ringbuffer[tail % ringbufferlenght], ringbufferlenght - (tail % ringbufferlenght));
      memcpy(pItem + ringbufferlenght - (tail % ringbufferlenght), &ringbuffer[0], datalen - ringbufferlenght + (tail % ringbufferlenght));
    }
    else
    {
      memcpy(pItem, &ringbuffer[tail % ringbufferlenght], datalen);
    }
    tail += datalen;
    return true;
  }
  bool back(T *pItem)
  {
    size_t lenght = sizeof(T);
    uint64_t datalen;
    if (head <= tail)
      return false;
    if (head - tail < lenght)
    {
      return false;
    }
    else
    {
      datalen = lenght;
    }
    if (datalen > ringbufferlenght - (tail % ringbufferlenght))
    {
      memcpy(pItem, &ringbuffer[tail % ringbufferlenght], ringbufferlenght - (tail % ringbufferlenght));
      memcpy(pItem + ringbufferlenght - (tail % ringbufferlenght), &ringbuffer[0], datalen - ringbufferlenght + (tail % ringbufferlenght));
    }
    else
    {
      memcpy(pItem, &ringbuffer[tail % ringbufferlenght], datalen);
    }
    // tail+=datalen;
    return true;
  }

  bool is_not_empty()
  {
    return head > tail;
  }
};

#endif // RINGBUFFER_H
