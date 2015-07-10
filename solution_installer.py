#!/usr/bin/env python
import sys
import os
import argparse
import tempfile
import subprocess
from xml.etree.ElementTree import parse
import shutil
import shlex


class SolutionInstaller():

    def __init__(self):
        self._cur_process = None

    def logdebug(self, msg):
        print '\033[92m' + msg.replace('\n', '') + '\x1b[0m'

    def logwarn(self, msg):
        print '\033[93m' + msg.replace('\n', '') + '\x1b[0m'

    def loginfo(self, msg):
        print str(msg)

    def _parse_argument(self, argv):
        parser = argparse.ArgumentParser(description="Solution installer")
        parser.add_argument('-t', '--target', default='', help='Target solution name')
        parser.add_argument('-r', '--repo', default='', help='github repository name <github repo owner>/<repository name>')
        parser.add_argument('-c', '--concert', default='concert.launch', help='concert launcher name')

        return parser.parse_args(argv[1:])

    def _find_solution(self, taget_local_path, target_solution):
        for root, dirs, files in os.walk(taget_local_path):
            if 'package.xml' in files:
                pkg_xml = parse(os.path.join(root, 'package.xml')).getroot()
                pkg_name = pkg_xml.find('name')
                if pkg_name is not None and pkg_name.text == target_solution:
                    return root

    def _get_src_from_gitbase(self, solution_repo_git_path, taget_local_path):
        if not os.path.isdir(taget_local_path):
            cmd = 'git clone %s %s' % (solution_repo_git_path, taget_local_path)
            self._run_command(cmd)
        else:
            os.chdir(taget_local_path)
            cmd = 'git pull --verbose'
            self._run_command(cmd)

    def _check_dependency(self, ws_path):
        cmd = 'rosdep install -r --from-paths %s --ignore-src --rosdistro indigo -y' % os.path.join(ws_path, 'src')
        self._run_command(cmd)

    def _build_workspace(self, ws_path):
        cmd = 'catkin_make -C %s' % ws_path
        self._run_command(cmd)

    def _setup_solution_env(self, ws_path):
        setup_bash_path = os.path.join(ws_path, 'devel', 'setup.bash')
        cmd = 'source %s'
        self._run_command(cmd, multiple_cmd=True, executable='/bin/bash')

    def _launch_target_solution(self, target_solution, concert_name):
        cmd = 'roslaunch %s %s --screen' % (target_solution, concert_name)
        self._run_command(cmd, multiple_cmd=True, executable='/bin/bash')

    def _run_command(self, command, multiple_cmd=False, executable=None):
        self.logdebug(command)
        if multiple_cmd:
            process = subprocess.Popen(command, shell=multiple_cmd, executable=executable, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        else:
            process = subprocess.Popen(shlex.split(command), executable=executable, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self._cur_process = process
        while True:
            output = process.stdout.readline()
            if output == '' and process.poll() is not None:
                break
            if output:
                self.loginfo(output.strip())
        self.logwarn("Return Code of Process: [%s]" % process.returncode)

    def _get_ros_ws(self):
        cmake_prefix_path = os.getenv('CMAKE_PREFIX_PATH')
        workspace_path = ''
        if cmake_prefix_path is not None and len(cmake_prefix_path) is not 0:
            for ws in cmake_prefix_path.split(':'):
                if os.path.isfile(os.path.join(ws, '.catkin')) and os.path.isfile(os.path.join(ws[:ws.rindex('/')], '.catkin_workspace')):
                    workspace_path = ws[:ws.rindex('/')]
                    break
        return workspace_path

    def shutdown(self):
        self._cur_process.terminate()

    def solution_install(self, argv):

        args = self._parse_argument(argv)
        concert_name = str(args.concert)
        target_solution = str(args.target)
        solution_repo_name = str(args.repo)

        self.logwarn('Check argument: [concert_name: %s][target_solution: %s][solution_repo_name: %s]' % (concert_name, target_solution, solution_repo_name))
        if len(concert_name) != 0 and len(target_solution) != 0 and len(solution_repo_name) != 0:
            taget_local_path = os.path.join(tempfile.gettempdir(), 'solution_repo')
            solution_repo_git_path = 'https://github.com/%s.git' % (solution_repo_name)
            ros_ws_path = self._get_ros_ws()
            if len(ros_ws_path):                
                self._get_src_from_gitbase(solution_repo_git_path, taget_local_path)
                ros_ws_src_path = os.path.join(ros_ws_path, 'src')
                if not os.path.isdir(ros_ws_src_path):
                    self.logwarn('Do not set ROS workspace: [%s]' % ros_ws_src_path)
                else:
                    src = self._find_solution(taget_local_path, target_solution)
                    dst = os.path.join(ros_ws_src_path, target_solution)
                    if not os.path.isdir(dst):
                        self.logwarn('Create solution in workpace: [%s]' % dst)
                    else:
                        self.logwarn('Already existed solution in workpace: [%s]' % dst)
                        shutil.rmtree(dst) ## change into overwrite way
                        shutil.copytree(src, dst)
                    self._check_dependency(ros_ws_path)
                    self._build_workspace(ros_ws_path)
                self._setup_solution_env(ros_ws_path)
                self._launch_target_solution(target_solution, concert_name)
            else:
                self.logwarn('Do not set ROS workspace: [%s]' % ros_ws_path)

        pass
if __name__ == '__main__':
    si = SolutionInstaller()
    try:
        si.solution_install(sys.argv)
    except KeyboardInterrupt as e:
        si.shutdown()
        si.logwarn('Stop process by interrupt: [%s]' % str(e))

    si.loginfo('Bye Bye')
