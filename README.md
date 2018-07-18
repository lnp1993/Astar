# Astar
Astar_algorithm
代码主要参考https://blog.csdn.net/u012234115/article/details/47152137

该篇博客主要存在两个bug（可以尝试把地图改成20*20，程序可能直接崩）：

1、Astar.cpp在判断周围8个点是否是可以用于下一步判断中，109行&&改为||

2、只能进行一次运算，如果计算过一次路径，继续执行

       list<Point *> path=astar.GetPath(start,end,false);算出来的路径是错误的，主要是程序中一些list没有clear

最后场合不同代价因子不同，某些场合直移和斜移代价因子可以按相同计算，如海康威视2018软件精英挑战赛
