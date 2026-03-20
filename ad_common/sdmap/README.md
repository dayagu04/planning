# SDMap接口使用说明——（地图来源:四维图新）

## sdmap消息内容

当前导航的**路径**。

路径以数组的形式给出，数组的每一个元素代表路径上的一段道路(称为一个Segment)。每个Segment有相应的属性，主要有：

- id
- 长度
- 形状点（不保证是道路中心点）
- 这一段的限速
- 这一段的结尾是否是路口
- 这一段到下一段车辆的行驶方向
- 道路类型道路等级（是否为匝道等）
- 前驱，即in_link
- 后继，即out_link

## `SDMap`主要接口说明

1. LoadMapFromProto： 读消息
2. GetNearestRoad：根据自车位置找当前自车所在的Segment
3. GetPreviousRoadSegment
4. GetNextRoadSegment
5. GetCrossInfo：获取路径上接下来的2个路口的信息
6. GetRampInfo：获取路径下接下来最近的匝道的信息
7. GetMergeInfoList：获取路径上接下来所有的汇流点信息
8. GetSplitInfoList：获取路径上接下来所有的分流点信息
