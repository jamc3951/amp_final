#!/usr/bin/python2
# Base class for MDP models to inherit from

import numpy as np


class ModelSpec(object):
    def __init__(self, N=100, acts=5, obs=5):
        self.discount = 0.95 #0.9
        self.N = N
        self.acts = acts
        self.obs = obs

        # Dimensions: action, from_state, to_state
        self.px = np.zeros(shape=(self.acts, self.N, self.N),
                           dtype=np.float16)  # .tolist()

        # Normalize to something sane, but not interesting
        # Transitions:
        for i in range(0, self.acts):
            for j in range(0, self.N):
                self.px[i][j][j] = 1.0

        # Observations
        self.pz = np.ones(shape=(self.obs, self.N),
                          dtype=np.float16)  # .tolist()

        # Rewards
        self.R = np.zeros(shape=(self.acts, self.N),
                          dtype=np.float16)  # .tolist()
        self.R_values = np.zeros(shape=(self.N),dtype=np.float16)
