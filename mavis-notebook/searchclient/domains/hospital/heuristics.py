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
        self.goal_positions = defaultdict(list)
        
    def preprocess(self, level: h_level.HospitalLevel):
        # This function will be called a single time prior 
        # to the search allowing us to preprocess the level such as
        # pre-computing lookup tables or other acceleration structures
        self.goal_positions = defaultdict(list)
        for(x,y), letter in level.goals.items():
            self.goal_positions[letter].append((x,y))
        

    def h(self, state: h_state.HospitalState, 
                goal_description: h_goal_description.HospitalGoalDescription) -> int:
        total_distance = 0
        for box_position, box_letter in state.boxes.items():
            if box_letter not in self.goal_positions:
                continue
        min_distance=min(
            abs(box_position[0]-gx) + abs(box_position[1]-gy)
            for(gx, gy) in self.goal_positions[box_letter]
        )
        total_distance+=min_distance
        unsatisfied_goals=[
            (position, letter) for position, letter in goal_description.goals.items()
            if state.boxes.get(position)!=letter
        ]
        total_distance+=len(unsatisfied_goals)
        return total_distance
    

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

        for goal in goal_description.goals:
            goal_pos = goal[0]
            expected_agent = goal[1]

            # print("agent at goal pos:", state.agent_at(goal_pos)[1])
            # print("expected agent at goal pos:", goal[1])
            if state.agent_at(goal_pos)[1] == expected_agent:
                # print("match found")
                remaining -= 1
            # print("remaining goals:", remaining)
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

        for boxPos, boxLetter in state.boxes.items():
            goals = [(x, y) for (x, y), letter in goal_description.goals.items()
                     if letter == boxLetter]
            if goals:
                minDistance = min(abs(boxPos[0] - goalX) + abs(boxPos[1] - goalY)
                               for goalX, goalY in goals)
                totalDistance += minDistance
        return totalDistance
