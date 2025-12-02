# coding: utf-8
#
# Copyright 2021 The Technical University of Denmark
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#    http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from __future__ import annotations
import sys
import itertools
import numpy as np
from utils import pos_add, pos_sub, APPROX_INFINITY
from collections import deque, defaultdict

import domains.hospital.state as h_state
import domains.hospital.goal_description as h_goal_description
import domains.hospital.level as h_level


class HospitalZeroHeuristic:
    def __init__(self):
        pass

    def preprocess(self, level: h_level.HospitalLevel):
        # This function will be called a single time prior 
        # to the search allowing us to preprocess the level such as
        # pre-computing lookup tables or other acceleration structures
        pass

    def h(self, state: h_state.HospitalState, 
                goal_description: h_goal_description.HospitalGoalDescription) -> int:
        return 0
    

class HospitalGoalCountHeuristics:

    def __init__(self):
        self.goal_positions = {}

    def preprocess(self, level: h_level.HospitalLevel):
        # This function will be called a single time prior 
        # to the search allowing us to preprocess the level such as
        # pre-computing lookup tables or other acceleration structures
        pass

    def h(self, state: h_state.HospitalState, 
                goal_description: h_goal_description.HospitalGoalDescription) -> int:
        remaining = goal_description.num_sub_goals()

        for goal in goal_description.agent_goals:
            goal_pos = goal[0]
            expected_agent = goal[1]

            # print("agent at goal pos:", state.agent_at(goal_pos)[1])
            # print("expected agent at goal pos:", goal[1])
            if state.agent_at(goal_pos)[1] == expected_agent:
                # print("match found")
                remaining -= 1
            # print("remaining goals:", remaining)

        for goal in goal_description.box_goals:
            goal_pos = goal[0]
            expected_box = goal[1]

            if state.box_at(goal_pos)[1] == expected_box:
                # print("match found")
                remaining -= 1
        return remaining

class HospitalAdvancedHeuristics:

    def __init__(self):
        pass
    
    def __call__(self, state, goal_description):
        return self.h(state, goal_description)

    def preprocess(self, level: h_level.HospitalLevel):
        # This function will be called a single time prior to the search allowing us to preprocess the level such as
        # pre-computing lookup tables or other acceleration structures
        pass

    # sums the manhattan distance from each agent to its goal
    def h(self, state: h_state.HospitalState, goal_description: h_goal_description.HospitalGoalDescription) -> int:
        totalDistance = 0

        for (agentPos, agentLetter) in state.agent_positions:
            minDistance = float('inf')

            # a is not used for our purposes
            for (goalPos, goalLetter, a) in goal_description.goals:
                if goalLetter != agentLetter:
                    continue

                goalX, goalY = goalPos
                dist = abs(agentPos[0] - goalX) + abs(agentPos[1] - goalY)

                if dist < minDistance:
                    minDistance = dist

            if minDistance != float('inf'):
                totalDistance += minDistance

        for (boxPos, boxLetter) in state.box_positions:
            minDistance = float('inf')

            # a is not used for our purposes
            for (goalPos, goalLetter, a) in goal_description.goals:
                if goalLetter != boxLetter:
                    continue

                goalX, goalY = goalPos
                dist = abs(boxPos[0] - goalX) + abs(boxPos[1] - goalY)

                if dist < minDistance:
                    minDistance = dist

            if minDistance != float('inf'):
                totalDistance += minDistance
                
        return totalDistance
    
class MaxManhattan:

    def __init__(self):
        pass
    
    def __call__(self, state, goal_description):
        return self.h(state, goal_description)

    def preprocess(self, level: h_level.HospitalLevel):
        # This function will be called a single time prior to the search allowing us to preprocess the level such as
        # pre-computing lookup tables or other acceleration structures
        pass

    # find the maximum manhattan distance from the agents/boxes to their goals
    def h(self, state: h_state.HospitalState, goal_description: h_goal_description.HospitalGoalDescription) -> int:
        distances = []

        for (agentPos, agentLetter) in state.agent_positions:
            minDistance = float('inf')

            # a is not used for our purposes
            for (goalPos, goalLetter, a) in goal_description.goals:
                if goalLetter != agentLetter:
                    continue

                goalX, goalY = goalPos
                dist = abs(agentPos[0] - goalX) + abs(agentPos[1] - goalY)

                if dist < minDistance:
                    minDistance = dist

            if minDistance != float('inf'):
                distances.append(minDistance)
        
        # check box positions
        for (boxPos, boxLetter) in state.box_positions:
            minDistance = float('inf')

            # a is not used for our purposes
            for (goalPos, goalLetter, a) in goal_description.goals:
                if goalLetter != boxLetter:
                    continue

                goalX, goalY = goalPos
                dist = abs(boxPos[0] - goalX) + abs(boxPos[1] - goalY)

                if dist < minDistance:
                    minDistance = dist

            if minDistance != float('inf'):
                distances.append(minDistance)
        return max(distances)

#heuristic inspired by Hungarian algorithm but also taking into consideration Manhattan distance
#latter half (hungarian implementation) assisted by vscode copilot
class HungarianHeuristic:

    def __init__(self):
        pass

    def __call__(self, state, goal_description):
        return self.h(state, goal_description)

    def preprocess(self, level: h_level.HospitalLevel):
        # This function will be called a single time prior to the search allowing us to preprocess the level such as
        # pre-computing lookup tables or other acceleration structures
        pass

    def h(self, state, goal_description):
        # group boxes and goals by letter (Sokoban, box A to goal A)
        boxes_by_letter = {}
        for (pos,letter)in state.box_positions:
            boxes_by_letter.setdefault(letter, []).append(pos)
        goals_by_letter = {}
        for (pos, letter, _) in goal_description.goals:
            goals_by_letter.setdefault(letter, []).append(pos)

        #initializing total cost variable for later usage
        total_cost = 0
        #referring to the impossible state (when its used, this shouldn't be happening)
        INF = 10**9

        # we need to solve each sub-problem in sokoban on its own (ie A, then B, then C)
        for letter, box_positions in boxes_by_letter.items():
            if letter not in goals_by_letter:
                # make use of impossible state in case a letter that doesn't exist makes it in
                return INF

            goal_positions = goals_by_letter[letter]
            n = len(box_positions)
            m = len(goal_positions)

            if m < n:
                # in case there are mismatching box and goal counts
                return INF

            #in research, hungarian works off square matrices, so this is converting/building a matrix
            size = max(n, m)
            #in case of dead space
            cost = [[INF]*size for _ in range(size)]

            #now, actually attribute the boxes to their matching goals
            for i, (bx, by) in enumerate(box_positions):
                for j, (gx, gy) in enumerate(goal_positions):
                    #ASsinging box i to goal j 
                    cost[i][j] = abs(bx - gx) + abs(by - gy)

            # now, funneling this into the hungarian algorithm as defined in code
            assignment_cost = self.hungarian(cost)
            total_cost += assignment_cost

        return total_cost
    
    #Hungarian algorithm implementation
    #assign
    def hungarian(self, cost):
        n =len(cost)
        #r for row
        r =[0]* (n+1)
        #c for column
        c = [0]* (n+1)
        p= [0]* (n+1)
        way = [0]* (n+1)

        #loop designed to assign a box i to matching goal
        for i in range(1, n+1):
            p[0] =i
            j0 =0
            #holding on to lowest cost seen thus far
            minv= [float('inf')] * (n+1)
            #was used?
            used = [False] *(n+1)
            #inner loop
            while True:
                used[j0] = True
                i0 = p[j0]
                delta =float('inf')
                j1 = 0
                #working with assignments, continue keeping track of lowest cost
                for j in range(1, n+1):
                    if not used[j]:
                        cur = cost[i0-1][j-1] - r[i0] - c[j]
                        if cur < minv[j]:
                            minv[j] = cur
                            way[j] = j0
                        if minv[j] < delta:
                            delta = minv[j]
                            j1 = j
                # Updating so that tje optimal cost(assignments leading to that) are maintained
                for j in range(n+1):
                    if used[j]:
                        r[p[j]] += delta
                        c[j] -= delta
                    else:
                        minv[j] -= delta
                j0 =j1
                if p[j0] ==0:
                    break
            #backtracking and checking assignments against optimality
            while True:
                j1 = way[j0]
                p[j0] = p[j1]
                j0 = j1
                if j0== 0:
                    break

        return -c[0]
