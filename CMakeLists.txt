cmake_minimum_required(VERSION 2.8.3)

project(bb_ioboard)

find_package(catkin REQUIRED COMPONENTS genmsg)

add_message_files(DIRECTORY msg FILES IOFromBoard.msg IOToBoard.msg)
generate_messages(DEPENDENCIES)
