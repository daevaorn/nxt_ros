cmake_minimum_required(VERSION 2.8.3)

project(nxt_description)

file(DOWNLOAD http://foote-ros-pkg.googlecode.com/files/nxt_description.parts.tar.gz meshes/nxt_description.tar.gz
  EXPECTED_MD5 5b1eb23778428d33f8f6462e41d5c5e8)
#rosbuild_untar_file(meshes/nxt_description.tar.gz meshes/parts ALL)

add_custom_command(
   OUTPUT meshes/parts
   COMMAND tar xf meshes/nxt_description.tar.gz -C meshes/parts
   )

catkin_package()
