#include <ros_msg_builtin_io.h>
#include <simLib/simLib.h>
#include <iostream>
#include <stubs.h>

ROSReadOptions::ROSReadOptions()
    : uint8array_as_string(false)
{
}

ROSWriteOptions::ROSWriteOptions()
    : uint8array_as_string(false)
{
}

void read__bool(int stack, uint8_t *value, const ROSReadOptions *opt)
{
    bool v;
    if(sim::getStackBoolValue(stack, &v) == 1)
    {
        *value = v;
        sim::popStackItem(stack, 1);
    }
    else
    {
        throw sim::exception("expected bool");
    }
}

void read__int8(int stack, int8_t *value, const ROSReadOptions *opt)
{
    int v;
    if(sim::getStackInt32Value(stack, &v) == 1)
    {
        *value = v;
        sim::popStackItem(stack, 1);
    }
    else
    {
        throw sim::exception("expected integer");
    }
}

void read__uint8(int stack, uint8_t *value, const ROSReadOptions *opt)
{
    int v;
    if(sim::getStackInt32Value(stack, &v) == 1)
    {
        *value = v;
        sim::popStackItem(stack, 1);
    }
    else
    {
        throw sim::exception("expected integer");
    }
}

void read__int16(int stack, int16_t *value, const ROSReadOptions *opt)
{
    int v;
    if(sim::getStackInt32Value(stack, &v) == 1)
    {
        *value = v;
        sim::popStackItem(stack, 1);
    }
    else
    {
        throw sim::exception("expected integer");
    }
}

void read__uint16(int stack, uint16_t *value, const ROSReadOptions *opt)
{
    int v;
    if(sim::getStackInt32Value(stack, &v) == 1)
    {
        *value = v;
        sim::popStackItem(stack, 1);
    }
    else
    {
        throw sim::exception("expected integer");
    }
}

void read__int32(int stack, int32_t *value, const ROSReadOptions *opt)
{
    int v;
    if(sim::getStackInt32Value(stack, &v) == 1)
    {
        *value = v;
        sim::popStackItem(stack, 1);
    }
    else
    {
        throw sim::exception("expected integer");
    }
}

void read__uint32(int stack, uint32_t *value, const ROSReadOptions *opt)
{
    int v;
    if(sim::getStackInt32Value(stack, &v) == 1)
    {
        *value = v;
        sim::popStackItem(stack, 1);
    }
    else
    {
        throw sim::exception("expected integer");
    }
}

void read__int64(int stack, int64_t *value, const ROSReadOptions *opt)
{
    // XXX: we represent Int64 as double - possible loss of precision!
    double v;
    if(sim::getStackDoubleValue(stack, &v) == 1)
    {
        *value = v;
        sim::popStackItem(stack, 1);
    }
    else
    {
        throw sim::exception("expected double");
    }
}

void read__uint64(int stack, uint64_t *value, const ROSReadOptions *opt)
{
    // XXX: we represent UInt64 as double - possible loss of precision!
    double v;
    if(sim::getStackDoubleValue(stack, &v) == 1)
    {
        *value = v;
        sim::popStackItem(stack, 1);
    }
    else
    {
        throw sim::exception("expected double");
    }
}

void read__float32(int stack, float *value, const ROSReadOptions *opt)
{
    float v;
    if(sim::getStackFloatValue(stack, &v) == 1)
    {
        *value = v;
        sim::popStackItem(stack, 1);
    }
    else
    {
        throw sim::exception("expected float");
    }
}

void read__float64(int stack, double *value, const ROSReadOptions *opt)
{
    double v;
    if(sim::getStackDoubleValue(stack, &v) == 1)
    {
        *value = v;
        sim::popStackItem(stack, 1);
    }
    else
    {
        throw sim::exception("expected double");
    }
}

void read__string(int stack, std::string *value, const ROSReadOptions *opt)
{
    char *str;
    int strSize;
    if((str = sim::getStackStringValue(stack, &strSize)) != NULL && strSize > 0)
    {
        *value = std::string(str, strSize);
        sim::popStackItem(stack, 1);
        sim::releaseBuffer(str);
    }
    else
    {
        throw sim::exception("expected string");
    }
}

void read__time(int stack, ros::Time *value, const ROSReadOptions *opt)
{
    double v;
    if(sim::getStackDoubleValue(stack, &v) == 1)
    {
        *value = ros::Time(v);
        sim::popStackItem(stack, 1);
    }
    else
    {
        throw sim::exception("expected double");
    }
}

void read__duration(int stack, ros::Duration *value, const ROSReadOptions *opt)
{
    double v;
    if(sim::getStackDoubleValue(stack, &v) == 1)
    {
        *value = ros::Duration(v);
        sim::popStackItem(stack, 1);
    }
    else
    {
        throw sim::exception("expected double");
    }
}

void write__bool(uint8_t value, int stack, const ROSWriteOptions *opt)
{
    bool v = value;
    sim::pushBoolOntoStack(stack, v);
}

void write__int8(int8_t value, int stack, const ROSWriteOptions *opt)
{
    int v = value;
    sim::pushInt32OntoStack(stack, v);
}

void write__uint8(uint8_t value, int stack, const ROSWriteOptions *opt)
{
    int v = value;
    sim::pushInt32OntoStack(stack, v);
}

void write__int16(int16_t value, int stack, const ROSWriteOptions *opt)
{
    int v = value;
    sim::pushInt32OntoStack(stack, v);
}

void write__uint16(uint16_t value, int stack, const ROSWriteOptions *opt)
{
    int v = value;
    sim::pushInt32OntoStack(stack, v);
}

void write__int32(int32_t value, int stack, const ROSWriteOptions *opt)
{
    int v = value;
    sim::pushInt32OntoStack(stack, v);
}

void write__uint32(uint32_t value, int stack, const ROSWriteOptions *opt)
{
    int v = value;
    sim::pushInt32OntoStack(stack, v);
}

void write__int64(int64_t value, int stack, const ROSWriteOptions *opt)
{
    // XXX: we represent Int64 as double - possible loss of precision!
    double v = value;
    sim::pushDoubleOntoStack(stack, v);
}

void write__uint64(uint64_t value, int stack, const ROSWriteOptions *opt)
{
    // XXX: we represent UInt64 as double - possible loss of precision!
    double v = value;
    sim::pushDoubleOntoStack(stack, v);
}

void write__float32(float value, int stack, const ROSWriteOptions *opt)
{
    float v = value;
    sim::pushFloatOntoStack(stack, v);
}

void write__float64(double value, int stack, const ROSWriteOptions *opt)
{
    double v = value;
    sim::pushDoubleOntoStack(stack, v);
}

void write__string(std::string value, int stack, const ROSWriteOptions *opt)
{
    const char *v = value.c_str();
    sim::pushStringOntoStack(stack, v, value.length());
}

void write__time(ros::Time value, int stack, const ROSWriteOptions *opt)
{
    double v = value.toSec();
    sim::pushDoubleOntoStack(stack, v);
}

void write__duration(ros::Duration value, int stack, const ROSWriteOptions *opt)
{
    double v = value.toSec();
    sim::pushDoubleOntoStack(stack, v);
}

