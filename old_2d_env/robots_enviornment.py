
import rospy
from env_lib import Enviornment
from beginner_tutorials.msg import robot_env
from beginner_tutorials.msg import env_position_broadcast
from beginner_tutorials.msg import ai_result


requested_move = (0,1)
def main():
    env = Enviornment("robot_algo.png")

    def ai_info_callback(data):
        global requested_move
        requested_move = (data.dirx,data.diry)

    def


    guard_publishers = [rospy.Publisher('guard{}'.format(i), robot_env, queue_size=10) for i in range(1)]
    agent_publisher = rospy.Publisher('agent', robot_env, queue_size=10)
    for x in range(2000):
        for g_idx,guard in enumerate(env.guards):
            move = guard.find_move(env.rgb_vals)
            env.update_robot_pos((guard.x,guard.y),move)
            guard.exec_move(move)
            guard_publishers.publish((guard.x,guard.y))

        ray_results = env.communicate_line_sight(env.agent)
        coords_seen = env.agent_sees_objects()
        move_dir = agent_decider.on_sight(ray_results,coords_seen)
        actual_dir = env.exec_agent_move(env.agent,move_dir)
        agent_reward_col = env.agent_collect_reward()

        agent_decider.on_move(actual_dir,agent_reward_col)
        #if x % 10 == 0:
        #    env.save_img("data/data{}.png".format(x))
        if env.check_game_over() == "FOUND_AGENT":
            print("guard found agent")
            env.save_img("data.png")
            break

    print("total reward: ",env.reward_collected)

main()
