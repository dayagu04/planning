string frame_id
uint64 measurement_time

uint32 height  # image height, that is, number of rows
uint32 width   # image width, that is, number of columns

uint64 encoding  # enum PixelFormat
uint32 step # Full row length in bytes
uint8[] data    # actual matrix data, size is (step * rows)
int32 dma_buf_fd 
int64 phy_addr 
