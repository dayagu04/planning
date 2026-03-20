float32 point0_x            
float32 point0_y             
float32 point1_x            
float32 point1_y           
float32 point2_x            
float32 point2_y            
float32 point3_x              
float32 point3_y             
float32 direction1             #车位入口A斜率
float32 direction2             #车位入口B斜率
uint32 entrance_vacant         #车位是否为空
float32 entrance_score         #车位入口得分     
uint32 entrance_type           #视觉车位类型
uint32 id                      #车位id
float32 confidence             #车位置信度       [0.0-1.0]
float32 confidence0            #角点置信度       [0.0-1.0]
float32 confidence1            #角点置信度       [0.0-1.0]
float32 confidence2            #角点置信度       [0.0-1.0]
float32 confidence3            #角点置信度       [0.0-1.0]
uint32 life_time               #生命周期