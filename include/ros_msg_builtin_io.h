#ifndef SIM_ROS_PLUGIN__ROS_MSG_BUILTIN_IO__H
#define SIM_ROS_PLUGIN__ROS_MSG_BUILTIN_IO__H

#include <ros/ros.h>

struct ROSReadOptions
{
    bool uint8array_as_string;
    ROSReadOptions();
};

struct ROSWriteOptions
{
    bool uint8array_as_string;
    ROSWriteOptions();
};

void read__bool(int stack, uint8_t *value, const ROSReadOptions *opt = NULL);
void read__int8(int stack, int8_t *value, const ROSReadOptions *opt = NULL);
void read__uint8(int stack, uint8_t *value, const ROSReadOptions *opt = NULL);
void read__int16(int stack, int16_t *value, const ROSReadOptions *opt = NULL);
void read__uint16(int stack, uint16_t *value, const ROSReadOptions *opt = NULL);
void read__int32(int stack, int32_t *value, const ROSReadOptions *opt = NULL);
void read__uint32(int stack, uint32_t *value, const ROSReadOptions *opt = NULL);
void read__int64(int stack, int64_t *value, const ROSReadOptions *opt = NULL);
void read__uint64(int stack, uint64_t *value, const ROSReadOptions *opt = NULL);
void read__float32(int stack, float *value, const ROSReadOptions *opt = NULL);
void read__float64(int stack, double *value, const ROSReadOptions *opt = NULL);
void read__string(int stack, std::string *value, const ROSReadOptions *opt = NULL);
void read__time(int stack, ros::Time *value, const ROSReadOptions *opt = NULL);
void read__duration(int stack, ros::Duration *value, const ROSReadOptions *opt = NULL);
void write__bool(uint8_t value, int stack, const ROSWriteOptions *opt = NULL);
void write__int8(int8_t value, int stack, const ROSWriteOptions *opt = NULL);
void write__uint8(uint8_t value, int stack, const ROSWriteOptions *opt = NULL);
void write__int16(int16_t value, int stack, const ROSWriteOptions *opt = NULL);
void write__uint16(uint16_t value, int stack, const ROSWriteOptions *opt = NULL);
void write__int32(int32_t value, int stack, const ROSWriteOptions *opt = NULL);
void write__uint32(uint32_t value, int stack, const ROSWriteOptions *opt = NULL);
void write__int64(int64_t value, int stack, const ROSWriteOptions *opt = NULL);
void write__uint64(uint64_t value, int stack, const ROSWriteOptions *opt = NULL);
void write__float32(float value, int stack, const ROSWriteOptions *opt = NULL);
void write__float64(double value, int stack, const ROSWriteOptions *opt = NULL);
void write__string(std::string value, int stack, const ROSWriteOptions *opt = NULL);
void write__time(ros::Time value, int stack, const ROSWriteOptions *opt = NULL);
void write__duration(ros::Duration value, int stack, const ROSWriteOptions *opt = NULL);

#endif // SIM_ROS_PLUGIN__ROS_MSG_BUILTIN_IO__H
