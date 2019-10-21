from env_lib import Enviornment
from agent_ai import AgentDecisionMaker

def main():
    agent_decider = AgentDecisionMaker()
    env = Enviornment("robot_algo.png")
    for x in range(2000):
        for guard in env.guards:
            move = guard.find_move(env.rgb_vals)
            env.update_robot_pos((guard.x,guard.y),move)
            guard.exec_move(move)
        #print("updated")
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
