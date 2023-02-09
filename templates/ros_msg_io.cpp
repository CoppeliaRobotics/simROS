#include <ros_msg_io.h>
#include <simLib/simLib.h>
#include <stubs.h>
#include <cstring>

#py from parse_messages_and_services import get_msgs_info, get_msgs_srvs_info, TypeSpec
#py msgs = get_msgs_srvs_info(pycpp.params['messages_file'], pycpp.params['services_file'])
#py for msg, info in msgs.items():
void write__`info.typespec.normalized()`(const `info.typespec.ctype()`& msg, int stack, const ROSWriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
#py for n, t in info.fields.items():
#py if t.array:
#py if t.builtin and t.mtype in TypeSpec.fast_write_types:
        try
        {
            // write field '`n`' (using fast specialized function)
            sim::pushStringOntoStack(stack, "`n`", 0);
            sim::push`TypeSpec.fast_write_types[t.mtype]`TableOntoStack(stack, &(msg.`n`[0]), msg.`n`.size());
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field '`n`': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
#py elif t.builtin and t.mtype == 'uint8':
        try
        {
            // write field '`n`' (using fast specialized function)
            sim::pushStringOntoStack(stack, "`n`", 0);
            if(opt && opt->uint8array_as_string)
                sim::pushStringOntoStack(stack, (char*)&(msg.`n`[0]), msg.`n`.size());
            else
                sim::pushUInt8TableOntoStack(stack, &(msg.`n`[0]), msg.`n`.size());
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field '`n`': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
#py else:
        try
        {
            // write field '`n`'
            sim::pushStringOntoStack(stack, "`n`", 0);
            sim::pushTableOntoStack(stack);
            for(int i = 0; i < msg.`n`.size(); i++)
            {
                write__int32(i + 1, stack, opt);
                write__`t.normalized()`(msg.`n`[i], stack, opt);
                sim::insertDataIntoStackTable(stack);
            }
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field '`n`': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
#py endif
#py else:
        try
        {
            // write field '`n`'
            sim::pushStringOntoStack(stack, "`n`", 0);
            write__`t.normalized()`(msg.`n`, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field '`n`': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
#py endif
#py endfor
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__`info.typespec.normalized()`: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__`info.typespec.normalized()`(int stack, `info.typespec.ctype()` *msg, const ROSReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
#py for n, t in info.fields.items():
#py if t.array:
#py if t.builtin and t.mtype in TypeSpec.fast_write_types:
                else if(strcmp(str, "`n`") == 0)
                {
                    try
                    {
                        // read field '`n`' (using fast specialized function)
                        int sz = sim::getStackTableInfo(stack, 0);
                        if(sz < 0)
                            throw sim::exception("expected array");
                        if(sim::getStackTableInfo(stack, 2) != 1)
                            throw sim::exception("fast_write_type reader exception #1");
#py if t.array_size:
                        // field has fixed size -> no need to reserve space into vector
#py else:
                        msg->`n`.resize(sz);
#py endif
                        sim::getStack`TypeSpec.fast_write_types[t.mtype]`Table(stack, &(msg->`n`[0]), sz);
                        sim::popStackItem(stack, 1);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field `n`: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
#py elif t.builtin and t.mtype == 'uint8':
                else if(strcmp(str, "`n`") == 0)
                {
                    try
                    {
                        if(opt && opt->uint8array_as_string)
                        {
                            // read field '`n`' (uint8[]) as string
                            char *str;
                            int sz;
                            if((str = sim::getStackStringValue(stack, &sz)) != NULL && sz > 0)
                            {
                                /*
                                 * XXX: if an alternative version of simGetStackStringValue woudl exist
                                 * working on an externally allocated buffer, we won't need this memcpy:
                                 */
#py if t.array_size:
                                // field has fixed size -> no need to reserve space into vector
#py else:
                                msg->`n`.resize(sz);
#py endif
                                std::memcpy(&(msg->`n`[0]), str, sz);
                                sim::releaseBuffer(str);
                            }
                            else throw sim::exception("string read error when trying to read uint8[]");
                        }
                        else
			{
                            // read field '`n`' (using fast specialized function)
                            int sz = sim::getStackTableInfo(stack, 0);
                            if(sz < 0)
                                throw sim::exception("expected uint8 array");
                            if(sim::getStackTableInfo(stack, 2) != 1)
                                throw sim::exception("fast_write_type uint8[] reader exception #1");
#py if t.array_size:
                            // field has fixed size -> no need to reserve space into vector
#py else:
                            msg->`n`.resize(sz);
#py endif
                            sim::getStackUInt8Table(stack, &(msg->`n`[0]), sz);
                            sim::popStackItem(stack, 1);
			}
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field `n`: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
#py else: # array not fast func
                else if(strcmp(str, "`n`") == 0)
                {
                    try
                    {
                        // read field '`n`'
                        if(sim::getStackTableInfo(stack, 0) < 0)
                            throw sim::exception("expected array");
                        int oldsz1 = sim::getStackSize(stack);
                        sim::unfoldStackTable(stack);
                        int numItems1 = (sim::getStackSize(stack) - oldsz1 + 1) / 2;
                        for(int i = 0; i < numItems1; i++)
                        {
                            sim::moveStackItemToTop(stack, oldsz1 - 1); // move key to top
                            int j;
                            read__int32(stack, &j, opt);
                            sim::moveStackItemToTop(stack, oldsz1 - 1); // move value to top
                            `t.ctype()` v;
                            read__`t.normalized()`(stack, &v, opt);
#py if t.array_size:
                            msg->`n`[i] = (v);
#py else:
                            msg->`n`.push_back(v);
#py endif
                        }
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field `n`: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
#py endif
#py else: # not array
                else if(strcmp(str, "`n`") == 0)
                {
                    try
                    {
                        // read field '`n`'
                        read__`t.normalized()`(stack, &(msg->`n`), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field `n`: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
#py endif
#py endfor
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                simReleaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__`info.typespec.normalized()`: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

#py endfor
#py msgs = get_msgs_info(pycpp.params['messages_file'])
#py for msg, info in msgs.items():
void ros_callback__`info.typespec.normalized()`(const boost::shared_ptr<`info.typespec.ctype()` const>& msg, SubscriberProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__`info.typespec.normalized()`(*msg, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            simReleaseStack(stack); // don't throw
        std::string msg = "ros_callback__`info.typespec.normalized()`: ";
        msg += ex.what();
        simSetLastError(proxy->topicCallback.name.c_str(), msg.c_str());
    }
}

#py endfor
