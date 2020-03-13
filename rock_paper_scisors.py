import pygame
import argparse
import json
import time
import os
import shutil
import subprocess
import numpy as np
import random
import copy
import nashpy

class RPCChoice:
    def __init__(self,intval):
        self.feature = intval
    def random_choice():
        return RPCChoice(random.randrange(3))
    def random_alt(self):
        return RPCChoice(random.randrange(3))

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
        new_feature = self.feature + np.random.uniform(size=2)*ALT_DIST
        mag = np.sum(np.square(new_feature))
        if mag > 1:
            new_feature /= mag
        return DiskChoice(new_feature)

class CombChoice:
    def __init__(self,num_choices):
        self.num_choices = num_choices
        self.feature = self.random_features()
    def random_choice():
        return CombChoice(np.random.randint(0,1+1,size=self.num_choices))
    def random_alt(self):
        idx_flip = np.random.randint(0,self.num_choices)
        new_feature = np.copy(self.feature)
        new_feature[idx_flip] = 1-new_feature[idx_flip]

class RPC_CombChoice:
    def __init__(self,comb,rpc):
        self.num_choices = comb.num_choices
        self.comb = comb
        self.rpc = rpc
    def random_choice(num_choices):
        return RPC_CombChoice(CombChoice(num_choices),RPCChoice())
    def random_alt(self):
        if random.random() < 1./(self.num_choices+1):
            return RPC_CombChoice(self.comb,self.rpc.random_alt())
        else:
            return RPC_CombChoice(self.comb.random_alt(),self.rpc)

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
        player = player*2-1
        return (choice.feature+player)%3

class RPCCombObjective:
    def __init__(self,num_combs,mul_val):
        self.num_combs = num_combs
        self.comb_objectives = [CombObjective(num_combs) for _ in range(3)]
        self.rpc = RPCObjective()
        self.mul_val = mul_val
    def random_choice(self,player):
        return RPC_CombChoice.random_choice(self.num_combs)
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
    def cyclic_responses(self):
        return [RPC_CombChoice(self.comb_objectives[i],RPCChoice(i)) for i in range(3)]


class NashChoiceMixture:
    def __init__(self,objective):
        self.player_strategies = [[],[]]
        self.objective_matrix = np.zeros((0,0))
        self.objective = objective

    def add_player_choice(self,player,strategy):
        self.player_strategies[player].append(strategy)
        other_player = player^1

        new_objective_row = []
        for other_strat in self.player_strategies[other_player]:
            strats = [None]*2
            strats[player] = strategy
            strats[other_player] = other_strat
            evaluation = self.objective(strats)
            new_objective_row.append(evaluation)
        self.objective_matrix = np.concatenate([self.objective_matrix,new_objective_row],axis=player)

    def sample_player_opponents(self,player,sample_size):
        matching_pennies = nash.Game(self.objective_matrix)
        equilibria = matching_pennies.support_enumeration()
        print(equilibria)
        strategy_support = equilibria[player]
        choices = random.choices(self.player_strategies[player],weights=strategy_support,k=sample_size)
        return choices


class UniformChoiceMixture:
    def __init__(self):
        self.player_strategies = [[],[]]
    def add_player_choice(self,player,strategy):
        self.player_strategies[player].append(strategy)
    def sample_player_opponents(self,player,sample_size):
        return random.choices(self.player_strategies[player],k=sample_size)

def evaluate_on_player(obj,c1,c2,player):
    if player > 0:
        p1,p2 = p2,p1
    return obj.evaluate(c1,c2)

def local_search_strategy(choice_mixture,objective,starter_strat,player,sample_size,num_local_searches):
    results = []
    opponent_list = choice_mixture.sample_player_opponents(player,sample_size)

    for _ in range(num_local_searches):
        cur_strat = starter_strat.random_alt()
        total_evaluation = sum((evaluate_on_player(objective,cur_strat,opponent_strat,player)
                                for opponent_strat in opponent_list))

        results.append((total_evaluation,cur_strat))
    best_strat = max(results,key=lambda x: x[0])[1]
    return best_strat

def global_search_strategy(choice_mixture,objective,starter_strat,player,sample_size):
    results = []
    opponent_list = choice_mixture.sample_player_opponents(player,sample_size)

    for cur_strat in objective.cyclic_responses():
        total_evaluation = sum((evaluate_on_player(objective,cur_strat,opponent_strat,player)
                                for opponent_strat in opponent_list))

        results.append((total_evaluation,cur_strat))
    best_strat = max(results,key=lambda x: x[0])[1]
    return best_strat

def run_game(choice_mixture,objective):
    latest_strats = []
    for p in range(2):
        rand_stratp1 = objective.random_choice(p)
        choice_mixture.add_player_choice(p,rand_stratp1)
        latest_strats.append(rand_stratp1)
    for i in range(100):
        for p in range(2):
            local_search_strategy(choice_mixture,objective,
            choice_mixture.add_player_choice(p,chosen_strat)

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
