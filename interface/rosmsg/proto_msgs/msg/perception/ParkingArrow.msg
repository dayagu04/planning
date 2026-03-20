uint32 id                  #地面箭头id
Point2f[] begin_points_2d   #组成箭头起始点的2d点
Point2f[] end_points_2d      #组成箭头终止点的2d点
float32 confidence         #接地线感知置信度 [0.0-1.0]
uint32 life_time              #生命周期
float32[] begin_scores
uint32 begin_dim               
uint32 begin_num       
uint32 begin_reserved
float32[] end_scores
uint32 end_dim                
uint32 end_num
uint32 end_reserved