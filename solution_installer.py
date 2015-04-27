#!/usr/bin/env python
import os
import tempfile
import subprocess
from xml.etree.ElementTree import parse
import shutil
import shlex
from os import environ
import signal


def logdebug(msg):
    print '\033[92m' + msg.replace('\n', '') + '\x1b[0m' 

def loginfo(msg):
    print msg.replace('\n', '')


def find_solution(taget_local_path, target_solution):
    for root, dirs, files in os.walk(taget_local_path):
        if 'package.xml' in files:
            pkg_xml = parse(os.path.join(root, 'package.xml')).getroot()
            pkg_name = pkg_xml.find('name')
            if pkg_name is not None and pkg_name.text == target_solution:
                return root


def get_src_from_gitbase(solution_repo_git_path, taget_local_path):
    if not os.path.isdir(taget_local_path):
        cmd = 'git clone %s %s' % (solution_repo_git_path, taget_local_path)
        run_command(cmd)
    else:
        os.chdir(taget_local_path)
        cmd = 'git pull --verbose'
        run_command(cmd)


def check_dependency(ws_path):
    cmd = 'rosdep install -r --from-paths %s --ignore-src --rosdistro indigo -y' % os.path.join(ws_path, 'src')
    run_command(cmd)


def build_workspace(ws_path):
    cmd = 'catkin_make -C %s' % ws_path
    run_command(cmd)


def launch_target_solution(ws_path, target_solution, concert_name):
    setup_bash_path = os.path.join(ws_path, 'devel', 'setup.bash')
    cmd = 'source %s ; rocon_launch %s %s --screen' % (setup_bash_path, target_solution, concert_name)
    run_command(cmd, multiple_cmd=True, executable='/bin/bash')


def run_command(command, multiple_cmd=False, executable=None):
    logdebug(command)
    if multiple_cmd:
        process = subprocess.Popen(command, shell=multiple_cmd, executable=executable, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    else:
        process = subprocess.Popen(shlex.split(command), executable=executable, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    while True:
        output = process.stdout.readline()
        if output == '' and process.poll() is not None:
            break
        if output:
            loginfo(output.strip())
    loginfo("Return Code of Process: [%s]" % process.returncode)
    for err in process.stderr:
        loginfo(err)
    


def get_ros_ws():
    cmake_prefix_path = os.getenv('CMAKE_PREFIX_PATH')
    workspace_path = ''
    if cmake_prefix_path is not None and len(cmake_prefix_path) is not 0:
        for ws in cmake_prefix_path.split(':'):
            if os.path.isfile(os.path.join(ws, '.catkin')) and os.path.isfile(os.path.join(ws[:ws.rindex('/')], '.catkin_workspace')):
                workspace_path = ws[:ws.rindex('/')]
                break
    return workspace_path

if __name__ == '__main__':

    target_solution = 'test_concert'
    concert_name = 'test.concert'
    solution_repo_name = 'dwlee/concert_common_solutions'

    taget_local_path = os.path.join(tempfile.gettempdir(), 'solution_repo')
    solution_repo_git_path = 'https://github.com/%s.git' % (solution_repo_name)
    try:
        ros_ws_path = get_ros_ws()
        if len(ros_ws_path):
            # get solution from repo
            get_src_from_gitbase(solution_repo_git_path, taget_local_path)
            ros_ws_src_path = os.path.join(ros_ws_path, 'src')
            if not os.path.isdir(ros_ws_src_path):
                loginfo('Do not set ROS workspace: [%s]' % ros_ws_src_path)
            else:
                src = find_solution(taget_local_path, target_solution)
                dst = os.path.join(ros_ws_src_path, target_solution)
                if not os.path.isdir(dst):
                    loginfo('Create solution in workpace: [%s]' % dst)
                else:
                    loginfo('Already existed solution in workpace: [%s]' % dst)
                    shutil.rmtree(dst)
                shutil.copytree(src, dst)
            # build solution
            check_dependency(ros_ws_path)
            build_workspace(ros_ws_path)
            # launch target solution
            launch_target_solution(ros_ws_path, target_solution, concert_name)
        else:
            loginfo('Do not set ROS workspace: [%s]' % ros_ws_path)
    except KeyboardInterrupt as e:
        loginfo('Stop all process: [%s]' % str(e))
    loginfo('Bye Bye')
