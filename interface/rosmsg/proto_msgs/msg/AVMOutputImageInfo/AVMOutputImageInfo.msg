std_msgs/Header ros_header
common_Header msg_header
uint32 nWidth                                          # 融合后的图像宽度；一般与引擎配置文件<avm_cfg.json>中的配置项nOffScreenRenderingImageWidth的值一致
uint32 nHeight	                                        # 融合后的图像高度；一般与引擎配置文件<avm_cfg.json>中的配置项nOffScreenRenderingImageHeight的值一致
uint32 nChannel                                        # 融合后的图像通道数；一般与引擎配置文件<avm_cfg.json>中的配置项nOffScreenRenderingImageChannel的值一致，RGB类型为3、RGBA类型为4，A1000平台为4
uint32 SizeBytes                                       # 融合后的图像数据大小(实际图像大小)；一般为nWidth*nHeight*nChannel
uint8[] pData 	                                        # 融合后的图像数据指针
uint64 TimeStamp 	                                    # 视觉车位时间戳
uint32 pixel_corner_points_num                         # 角点像素点数量
AVMPoint3si[] pixel_corner_points           # 角点像素点
uint32 pixel_limiter_points_num
AVMPoint3si[] pixel_limiter_points