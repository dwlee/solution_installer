#!/usr/bin/env python
import os
import tempfile
import subprocess
from xml.etree.ElementTree import parse
import shutil


def find_solution(taget_local_path, target_solution):
    for root, dirs, files in os.walk(taget_local_path):
        if 'package.xml' in files:
            pkg_xml = parse(os.path.join(root, 'package.xml')).getroot()
            pkg_name = pkg_xml.find('name')
            if pkg_name is not None and pkg_name.text == target_solution:
                return root

def get_src_from_remote(git_path, taget_local_path):
    if not os.path.isdir(taget_local_path):
        subprocess.check_call(["git", "clone", git_path, taget_local_path], stdout=subprocess.PIPE)
    else:
        os.chdir(taget_local_path)
        subprocess.check_call(["git", "pull"], stdout=subprocess.PIPE)


def get_ros_ws():
    cmake_prefix_path = os.getenv("CMAKE_PREFIX_PATH")
    workspace_path = ''
    if cmake_prefix_path is None or len(cmake_prefix_path) is 0:
        print "Not set workspace"
    else:
        for ws in cmake_prefix_path.split(':'):
            if os.path.isfile(os.path.join(ws, '.catkin')) and os.path.isfile(os.path.join(ws[:ws.rindex('/')], '.catkin_workspace')):
                workspace_path = ws[:ws.rindex('/')]
                break
    return workspace_path

    """
      if [ ! -z $CMAKE_PREFIX_PATH ]; then
        IFS=":" read -a workspaces <<< "$CMAKE_PREFIX_PATH"
        for ws in "${workspaces[@]}"; do
          if [ -f $ws/.catkin ]; then
    """

if __name__ == '__main__':

    selected_solution = "test_concert"
    repo_name = "dwlee/concert_common_solutions"
    taget_local_path = os.path.join(tempfile.gettempdir(), 'solution_repo')
    git_path = "https://github.com/%s.git" % (repo_name)
    
    get_src_from_remote(git_path, taget_local_path)
    ros_ws_src_path = os.path.join(get_ros_ws(), 'src')
    if not os.path.isdir(ros_ws_src_path):
        print "Do not set ROS workspace: [%s]" %ros_ws_src_path
    else:
        src = find_solution(taget_local_path, selected_solution)
        dst = os.path.join(ros_ws_src_path, selected_solution)
        if not os.path.isdir(dst):
            print "create solution in workpace: [%s]" % dst
        else:
            print "already existed solution in workpace: [%s]" % dst
            shutil.rmtree(dst)
        shutil.copytree(src, dst)
    # change working directory

    print "Bye Bye"
    

    
