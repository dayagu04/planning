std_msgs/Header header
common_Header msg_header
string frame_id # 图片的id
string format # 图片的编码格式 nv12 jpeg h264
string camera_view_angle # 相机视角
string camera_resolution # 相机分辨率信息
uint32 height # 图片的高
uint32 width # 图片的宽
uint64 exposure_starting_utc_time # 相机曝光开始UTC时间(微妙)
uint64 isp_arrival_utc_timestamp # 从ISP收到数据的到达UTC时间(微秒)
uint64 isp_arrival_monotonic_timestamp # 从ISP收到数据的到达内核时间(微秒)
uint32 exposure_duration # 相机的曝光时长(微秒)
uint32 isp_arrival_to_app_capture_duration_time # 相机从isp到达到应用层捕获的EOF时间(微妙)
int32  dmabuf_fd_index # 图片在dma ringbuffer里的序号
uint8[] data # 图片数据 <使用dma时，不用这个字段>