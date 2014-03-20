FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/SpiderRobot/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/SpiderRobot/My2Num.h"
  "../msg_gen/cpp/include/SpiderRobot/MyChar.h"
  "../msg_gen/cpp/include/SpiderRobot/MyArray.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
