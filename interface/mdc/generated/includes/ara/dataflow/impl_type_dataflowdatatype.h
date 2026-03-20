/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef ARA_DATAFLOW_IMPL_TYPE_DATAFLOWDATATYPE_H
#define ARA_DATAFLOW_IMPL_TYPE_DATAFLOWDATATYPE_H

#include "ara/dataflow/impl_type_header.h"
#include "impl_type_uint8.h"
#include "impl_type_string.h"
#include "impl_type_uint32.h"

namespace ara {
namespace dataflow {
struct DataFlowDataType {
    ::ara::dataflow::Header header;
    ::UInt8 channel_type;
    ::String channel_id;
    ::UInt32 recv_flow_stats_pps;
    ::UInt32 recv_flow_stats_bps;
    ::UInt32 send_flow_stats_pps;
    ::UInt32 send_flow_stats_bps;

    static bool IsPlane()
    {
        return false;
    }


    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun)
    {
        fun(header);
        fun(channel_type);
        fun(channel_id);
        fun(recv_flow_stats_pps);
        fun(recv_flow_stats_bps);
        fun(send_flow_stats_pps);
        fun(send_flow_stats_bps);
    }

    template<typename F>
    void enumerate(F& fun) const
    {
        fun(header);
        fun(channel_type);
        fun(channel_id);
        fun(recv_flow_stats_pps);
        fun(recv_flow_stats_bps);
        fun(send_flow_stats_pps);
        fun(send_flow_stats_bps);
    }

    template<typename F>
    void enumerate_internal(F& fun)
    {
        fun("header", header);
        fun("channel_type", channel_type);
        fun("channel_id", channel_id);
        fun("recv_flow_stats_pps", recv_flow_stats_pps);
        fun("recv_flow_stats_bps", recv_flow_stats_bps);
        fun("send_flow_stats_pps", send_flow_stats_pps);
        fun("send_flow_stats_bps", send_flow_stats_bps);
    }

    template<typename F>
    void enumerate_internal(F& fun) const
    {
        fun("header", header);
        fun("channel_type", channel_type);
        fun("channel_id", channel_id);
        fun("recv_flow_stats_pps", recv_flow_stats_pps);
        fun("recv_flow_stats_bps", recv_flow_stats_bps);
        fun("send_flow_stats_pps", send_flow_stats_pps);
        fun("send_flow_stats_bps", send_flow_stats_bps);
    }

    friend bool operator==(const ::ara::dataflow::DataFlowDataType& lhs, const ::ara::dataflow::DataFlowDataType& rhs) noexcept
    {
        return (lhs.header == rhs.header) && (lhs.channel_type == rhs.channel_type) && (lhs.channel_id == rhs.channel_id) && (lhs.recv_flow_stats_pps == rhs.recv_flow_stats_pps) && (lhs.recv_flow_stats_bps == rhs.recv_flow_stats_bps) && (lhs.send_flow_stats_pps == rhs.send_flow_stats_pps) && (lhs.send_flow_stats_bps == rhs.send_flow_stats_bps);
    }
};
} // namespace dataflow
} // namespace ara


#endif // ARA_DATAFLOW_IMPL_TYPE_DATAFLOWDATATYPE_H
