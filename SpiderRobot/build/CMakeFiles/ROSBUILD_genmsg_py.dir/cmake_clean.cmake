FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/SpiderRobot/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/SpiderRobot/msg/__init__.py"
  "../src/SpiderRobot/msg/_MyChar.py"
  "../src/SpiderRobot/msg/_My2Num.py"
  "../src/SpiderRobot/msg/_MyArray.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
