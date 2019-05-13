# tasks-allocation
hard working
重写一次代码，让代码更加精简且模块化。
在这里，我做了一些改变，
1. 我将某一子规划中心已调度的任务集T_schedule中的“任务交换”的邻域结构删除
2. 我添加了某一子规划中心已调度的任务集T_schedule中的任务按照最大冲突度进行删除的邻域结构
3. 我添加了某一子规划中心已调度的任务集T_schedule中的任务按照最小需求度进行删除的邻域结构
4. 我保留了某一子规划中心未调度的任务集T_unschedule中的任务按照最大需求度进行排序，需求度越
   大优先插入，按照最小冲突度原则选择子规划中心进行插入的操作
5. 为了使收敛曲线更加优美，删除任务的操作一次只进行1个。
6. 修改了平均冲突度，将其换成互不干扰度，在邻域结构删除和插入操作中，按照互不干扰度原则，结果确实比之前的要好一些。
7. 启发式任务分配算法2的最小冲突度修改为最大互不干扰度。
8. 接下来我想再进行一些改变，在每次调用uavs_func和sats_func得到的结果中，有每个观测资源的任务调度情况。
9. 无人机的分配结果比较简单，卫星的分配结果，由于是分配给各个轨道，我可以先给各个轨道分配编号，1-16属于卫星1，17-32属于卫星3，...
10. 在计算最大互不干扰度时，我想通过各观测资源已调度情况获得更准确的互不干扰度，这个确实可以改进,不过需要先保存一下。