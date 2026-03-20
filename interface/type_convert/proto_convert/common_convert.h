
#include "proto_convert/proto_convert.h"

/*to ros*/
void HeaderToRos(Common::Header obj, proto_msgs::common_Header &msg) {
  msg.timestamp = obj.timestamp();
  msg.version = obj.version();
  msg.input_list.resize(obj.input_list().size());
  for (int i = 0; i < obj.input_list().size(); i++) {
    msg.input_list[i].input_type = obj.input_list(i).input_type();
    msg.input_list[i].in_ts_us = obj.input_list(i).in_ts_us();
    msg.input_list[i].out_ts_us = obj.input_list(i).out_ts_us();
  }
  msg.seq = obj.seq();
};

/*to proto*/
void HeaderToProto(Common::Header &obj, proto_msgs::common_Header msg) {
  obj.set_timestamp(msg.timestamp);
  obj.set_version(msg.version);
  obj.set_seq(msg.seq);
  for (int i = 0; i < msg.input_list.size(); i++) {
    Common::InputHistoryTimestamp *input_list = obj.add_input_list();
    input_list->set_input_type(msg.input_list[i].input_type);
    input_list->set_in_ts_us(msg.input_list[i].in_ts_us);
    input_list->set_out_ts_us(msg.input_list[i].out_ts_us);
  }
};
