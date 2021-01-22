from __future__ import print_function
import subprocess, time
import rostopic


FIRST_LAUNCH = "/home/rosnuc/Critbot/robot/launch/critbot_init.launch"
SECOND_LAUNCH = "/home/rosnuc/Critbot/robot/launch/t265_tf_to_mavros.launch"
DEMANDED_TOPICS = ['/t265/odom/sample', '/mavros/vision_pose/pose']
MAX_COUNT = 10

def check_if_topic_is_running(topic):
    try:
        #subprocess.check_output(["rostopic list" ], shell=True)
        rostopic.get_topic_class(topic)
        return True
    except:
        return False


def launch_critbot():
    #, stdout=subprocess.PIPE
    first_launch_proc = subprocess.Popen(['roslaunch', FIRST_LAUNCH], stdout=subprocess.PIPE)

    #output = test.communicate()[0]
    #print(output)
    counter = 0
    while not check_if_topic_is_running(DEMANDED_TOPICS[0]) and counter <= MAX_COUNT:
        print("Roscore is still not working")
        counter += 1
        time.sleep(1)

    if counter == MAX_COUNT:
        print("MAX COUNT")
    #print("Found lines:\n" + str(check_node))
    #check_node = subprocess.check_output(["rostopic list" ], shell=True)

    # -w for exact match
    # -c for counting number of lines
    #p = subprocess.Popen(['grep','-w', '-c', '/rosout'], stdout=subprocess.PIPE, stdin=subprocess.PIPE)
    #grep_stdout = p.communicate(input = check_node)[0]
    #if p.poll():
    #    print("Process is still working")
    #    p.terminate()
    #print("found lines: " + str(grep_stdout))
    #, stdout=subprocess.PIPE
    second_launch_proc = subprocess.Popen(['roslaunch', SECOND_LAUNCH], stdout=subprocess.PIPE)
    print("Kill process %s" %second_launch_proc.pid)
    counter = 0
    while not check_if_topic_is_running(DEMANDED_TOPICS[1]) and counter <= MAX_COUNT:
        print("Second node is still not working")
        time.sleep(1)
        counter += 1

    #time.sleep(5)
    #print("Here!")
    #print("%s" %first_launch_proc.poll())
    
    print("PID first launch %s" %first_launch_proc.pid)
    print("PID second launch %s" %second_launch_proc.pid)
    '''
    if not first_launch_proc.poll():
        print("Kill process %s" %first_launch_proc.pid)
        first_launch_proc.terminate()

    if not second_launch_proc.poll():
        print("Kill process %s" %second_launch_proc.pid)
        second_launch_proc.terminate()
        #second_launch_proc.wait(5)
    '''

    #subprocess.call(['./kill_ros.sh'], shell=True)  


if __name__ == "__main__":
    launch_critbot()
