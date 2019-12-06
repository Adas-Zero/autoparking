#!/usr/bin/env python2

import sys

import torch
import torch.nn as nn
import torch.nn.functional as F
OUTPUT_PATH = "./trained_network.pth"
# Control Status

#Q-Network
class DQN(nn.Module):
    def __init__(self):
        super(DQN, self).__init__()
        self.input_layer = nn.Linear(19,64)
        self.hidden_layer = nn.Linear(64,32)
        self.ouput_layer = nn.Linear(32,6)

    def forward(self, x):
        x = F.relu(self.input_layer(x))
        x = F.relu(self.hidden_layer(x))
        x = F.relu(self.ouput_layer(x))
        return x

def select_action(state):
    return policy_net(state).max(1)[1].view(1, 1)

if __name__ == "__main__":
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    policy_net = DQN().to(device)
    policy_net.load_state_dict(torch.load(OUTPUT_PATH))
    policy_net.eval()
    #argument parsing needed
    action = select_action(sys.argv[1])
    print action


