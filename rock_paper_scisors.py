import pygame
import argparse
import json
import time
import os
import shutil
import subprocess
import numpy as np
import random

class RPCChoice:
    def __init__(self,intval):
        self.feature = intval
    def random_choice():
        return RPCChoice(random.randrange(3))
    def random_alt(self):
        self.feature = random.randrange(3)

class DiskChoice:
    def __init__(self,featurevec):
        self.feature = featurevec
    def random_choice():
        rad = random.random()
        theta = random.random()*2*math.pi
        x = math.cos(theta)*rad
        y = math.sin(theta)*rad
        return DiskChoice(np.array([x,y]))
    def random_alt(self):
        ALT_DIST = 0.1
        self.feature += np.random.uniform(size=2)*ALT_DIST
        mag = np.sum(np.square(self.feature))
        if mag > 1:
            self.feature /= mag

class CombChoice:
    def __init__(self,num_choices):
        self.num_choices = num_choices
        self.feature = self.random_features()
    def random_choice():
        return CombChoice(np.random.randint(0,1+1,size=self.num_choices))
    def random_alt(self):
        idx_flip = np.random.randint(0,self.num_choices)
        self.feature[idx_flip] = 1-self.feature[idx_flip]

class RPC_CombChoice:
    def __init__(self,comb,rpc):
        self.num_choices = comb.num_choices
        self.comb = comb
        self.rpc = rpc
    def random_choice(num_choices):
        return RPC_CombChoice(CombChoice(num_choices),RPCChoice())
    def random_alt(self):
        if random.random() < 1./(self.num_choices+1):
            self.rpc.random_alt()
        else:
            self.comb.random_alt()

class CombObjective:
    def __init__(self,num_combs):
        self.num_combs = num_combs
        self.match_choice = CombChoice(self.num_combs)
    def evaluate(self,choice):
        return np.sum(np.equal(self.match_choice.feature,choice.feature).astype(np.int32))
    def optimal_response(self):
        return self.match_choice

class RPCObjective:
    def __init__(self):
        pass
    def evaluate(self,choicep1,choicep2):
        diffmod3 = (choicep1.feature - choicep2.feature)%3
        return diffmod3 if diffmod3 <= 1 else -1
    def optimal_response(self,choice,player):
        return (choice.feature+player)%3
    # def optimal_response_p1(self,choicep2):
    #     return (choicep2.feature+1)%3
    # def optimal_response_p2(self,choicep1):
    #     return (choicep1.feature-1)%3

class RPCCombObjective:
    def __init__(self,num_combs,mul_val):
        self.num_combs = num_combs
        self.comb_objectives = [CombObjective(num_combs) for _ in range(3)]
        self.rpc = RPCObjective()
        self.mul_val = mul_val
    def evaluate(self,choicep1,choicep2):
        valp1 = self.comb_objectives[choicep1.rpc.feature].evaluate(choicep1.comb)
        valp2 = self.comb_objectives[choicep1.rpc.feature].evaluate(choicep2.comb)
        rpc_val = self.rpc.evaluate(choicep1.rpc,choicep2.rpc)
        return valp1 - valp2 + rpc_val * self.mul_val
    def optimal_response_player(self,choice,player):
        rpc_choice = self.rpc.optimal_response(choicep1,player)
        match_choie = self.comb_objectives[rpc_choice.feature].optimal_response()
        new_choice = RPC_CombChoice(match_choice,rpc_choice)
        return new_choice
    # def optimal_response_p1(self,choicep1):
    #     rpc_choice = self.rpc.optimal_response_p1(choicep1)
    #     match_choie = self.comb_objectives[rpc_choice.feature].optimal_response()
    #     new_choice = RPC_CombChoice(match_choice,rpc_choice)
    #     return new_choice
    # def optimal_response_p2(self,choicep1):
    #     rpc_choice = self.rpc.optimal_response_p2(choicep1)
    #     match_choie = self.comb_objectives[rpc_choice.feature].optimal_response()
    #     new_choice = RPC_CombChoice(match_choice,rpc_choice)
    #     return new_choice

class NashChoiceMixture:
    def __init__(self,player1_choices,player2_choices,objective):
        pass
    def sample_player1(self):
        pass
    def sample_player2(self):

class UniformChoiceMixture:
    def __init__(self):
        self.choices = []
    def add(self,choice):
        self.choices.append(choice)
    def sample(self):
        return random.choice(self.choices)

def local_search_objective(choice_mixture,)

def direction_score(x1,x2):
    return direction_matrix[x1][x2]

class Game:
    def __init__(self,num_combs,num_additive,opposition_val):
        self.num_combs = num_combs
        self.num_additive = num_additive
        self.opposition_val = opposition_val
        self.target_value = np.random.randint(0,num_combs,size=(3,num_additive))

    def permute_choice(self,orig_choice):
        type_val = random.random()
        new_choice = np.copy(orig_choice)
        if type_val < 1./3.:
            new_choice[0] = random.randrange(3)
        else:
            idx = random.randrange(self.num_additive)
            new_choice[idx+1] = random.randrange(self.num_combs)
        return new_choice

    def sample_choice(self):
        return  np.concatenate([
            np.random.randint(0,3,size=(1,)),
            np.random.randint(0,self.num_combs,size=self.num_additive),
        ])

    def calc_outcome(self,player_one_choice,player_two_choice):
        '''
        return: -1 for player one losing, 0 for draw, 1 for player one winning
        '''
        p1_choice = player_one_choice[0]
        p2_choice = player_two_choice[0]
        direct_score = direction_score(p1_choice,p2_choice)
        add_val1 = np.sum(np.equal(player_one_choice[1:],self.target_value[p1_choice]).astype(np.float32))
        add_val2 = np.sum(np.equal(player_two_choice[1:],self.target_value[p2_choice]).astype(np.float32))
        result_score = direct_score * self.opposition_val + add_val1 - add_val2
        result_outcome = 0 if result_score == 0 else (-1 if result_score < 0 else 1)
        return result_outcome, result_score

    def optimal_response(self,mixture,player):
        outcomes = np.zeros((3,),dtype=np.int32)
        opt_responses = np.concatenate([np.arange(3).reshape(3,1),self.target_value],axis=1)
        for sidx in range(3):
            score = 0
            cur_response = opt_responses[sidx]
            outcome,_ = self.response_value(cur_response,mixture,player)
            outcomes[sidx] = outcome

        best_bin = np.argmax(outcomes)
        response = np.concatenate([np.array([best_bin]),self.target_value[best_bin]])
        return response

    def response_value(self,strat,mixture,player):
        total_score = 0
        total_record = 0
        for m in mixture:
            record,score = self.calc_outcome(strat,m)
            total_score += score*player
            total_record += record*player
        return total_record/len(mixture),total_score/len(mixture)


def main():
    parser = argparse.ArgumentParser(description='run ai enviornmnent')
    parser.add_argument('num_combs', type=int, help='number of combinations possible per item')
    parser.add_argument('num_additive_items', type=int, help='number of items to score additively')
    parser.add_argument('opposition_weight', type=int, help='number of items in opposition')
    args = parser.parse_args()
    game = Game(args.num_combs,args.num_additive_items,args.opposition_weight)
    console_version(game)

def console_version(game):
    NUM_RESPONSES = 20
    mixture1 = [game.sample_choice()]
    mixture2 = [game.sample_choice()]
    #mixture_total_value =
    for i in range(10):
        last_sample1 = mixture1[len(mixture1)-1]
        local_responses = [game.permute_choice(last_sample1) for _ in range(NUM_RESPONSES)]
        best_response_idx = np.argmax([game.response_value(resp,mixture2,1)[1] for resp in local_responses])
        best_response = local_responses[best_response_idx]
        mixture1.append(best_response)
        #print("Mixure value response: {} {}".format(*game.response_value(resp)))
        print("Player 1 Local response: {} {}".format(*game.response_value(best_response,mixture2,1)))
        print("Player 1 Optimal response: {} {}".format(*game.response_value(game.optimal_response(mixture2,1),mixture2,1)))
        print("Player 1 response: ",best_response)
        last_sample2 = mixture2[len(mixture2)-1]
        local_responses = [game.permute_choice(last_sample2) for _ in range(NUM_RESPONSES)]
        best_response_idx = np.argmin([game.response_value(resp,mixture1,-1)[1] for resp in local_responses])
        best_response = local_responses[best_response_idx]
        mixture2.append(best_response)
        #print("Mixure value response: {} {}".format(*game.response_value(resp)))
        print("Player 2 Local response: {} {}".format(*game.response_value(best_response,mixture1,-1)))
        print("Player 2 Optimal response: {} {}".format(*game.response_value(game.optimal_response(mixture1,-1),mixture1,-1)))
        print("Player 2 response: ",best_response)
        #time.sleep(0.1)



def gui_version():
    pygame.init()

    if not args.no_display:
        screen = pygame.display.set_mode([map_info.width, map_info.height])
    else:
        screen = pygame.Surface((map_info.width, map_info.height), pygame.SRCALPHA)

    running = True
    count = 1
    frame_count = 0
    while running:
        # Did the user click the window close button?
        #if not args.no_display:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        if not args.no_display:
            time.sleep(0.05)

        # Fill the background with white
        screen.fill((255, 255, 255))

        #screen.blit(guard_screen_out, (0,0))
        #screen.blit(agent_screen_out, (0,0))
        #draw_density_map(screen,map_info,agent_weightmap,visibilty_info['points'],frame_count,(0,0,255),10)
        #draw_density_map(screen,map_info,agent_weightmap,visibilty_info['points'],frame_count,(0,255,0),env_values.guard_linesight)


        #if not args.no_display:
        pygame.display.flip()
        #SAMPLE_RATE = 1
        #if args.produce_video and frame_count % SAMPLE_RATE == 0:
        #    pygame.image.save(screen, img_dir+"data{0:05d}.png".format(frame_count//SAMPLE_RATE))
        #frame_count += 1


    #if args.produce_video:
    #    save_video(img_dir,video_name)
    # Done! Time to quit.
    pygame.quit()


if __name__=="__main__":
    main()
