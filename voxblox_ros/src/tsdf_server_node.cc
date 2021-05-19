#include "voxblox_ros/tsdf_server.h"

#include <gflags/gflags.h>

int main(int argc, char** argv) {
  /// 初始化ros节点，程序开始运行
  ros::init(argc, argv, "voxblox");
  /// 初始化gflags
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  /// 设置ros节点句柄
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  /// 创建TsdfServer对象，开始工作
  voxblox::TsdfServer node(nh, nh_private);

  ros::spin();
  return 0;
}
