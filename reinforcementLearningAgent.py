from enums import *
import random
import numpy as np
from collections import deque
from tempfile import TemporaryFile

class Agent:
    def __init__(self, learning_rate=0.1, discount=0.95, exploration_rate=1.0, iterations=20000):
        self.q_table = np.zeros((4,184832)) # Spreadsheet (Q-table) for rewards accounting
        self.learning_rate = learning_rate # How much we appreciate new q-value over current
        self.discount = discount # How much we appreciate future reward over current
        self.exploration_rate = 1.0 # Initial exploration rate
        self.exploration_delta = 1.0 / iterations # Shift from exploration to explotation

    def get_next_action(self, state):
        if random.random() > self.exploration_rate: # Explore (gamble) or exploit (greedy)
            return self.greedy_action(state)
        else:
            return self.random_action()

    def greedy_action(self, state):
           return np.argmax(self.q_table[:,state]) if random.random() < 0.5 else np.random.rand(4)

    def random_action(self):
        return np.random.rand(4)
    
    def store_memory(history, action, reward, next_history, dead):
        q_table.append((history, action, reward, next_history, dead))

    def save_q_table(path):
        np.save(outfile, self.q_table)

    def train_memory_batch(memory, model, log_dir):
        mini_batch = random.sample(memory, FLAGS.batch_size)
        history = np.zeros((FLAGS.batch_size, ATARI_SHAPE[0],
                            ATARI_SHAPE[1], ATARI_SHAPE[2]))
        next_history = np.zeros((FLAGS.batch_size, ATARI_SHAPE[0],
                                 ATARI_SHAPE[1], ATARI_SHAPE[2]))
        target = np.zeros((FLAGS.batch_size,))
        action, reward, dead = [], [], []

        for idx, val in enumerate(mini_batch):
            history[idx] = val[0]
            next_history[idx] = val[3]
            action.append(val[1])
            reward.append(val[2])
            dead.append(val[4])

        actions_mask = np.ones((FLAGS.batch_size, ACTION_SIZE))
        next_Q_values = model.predict([next_history, actions_mask])

        # like Q Learning, get maximum Q value at s'
        # But from target model
        for i in range(FLAGS.batch_size):
            if dead[i]:
                target[i] = -1
                # target[i] = reward[i]
            else:
                target[i] = reward[i] + FLAGS.gamma * np.amax(next_Q_values[i])

        action_one_hot = get_one_hot(action, ACTION_SIZE)
        target_one_hot = action_one_hot * target[:, None]

        #tb_callback = TensorBoard(log_dir=log_dir, histogram_freq=0,
        #                          write_graph=True, write_images=False)

        ''''''
        h = model.fit(
            [history, action_one_hot], target_one_hot, epochs=1,
            batch_size=FLAGS.batch_size, verbose=0)
            #batch_size=FLAGS.batch_size, verbose=0, callbacks=[tb_callback])

        #if h.history['loss'][0] > 10.0:
        #    print('too large')

        return h.history['loss'][0]

    def update(self, old_state, new_state, action, reward):
        # Old Q-table value
        old_value = self.q_table[action][old_state]
        # What would be our best next action?
        future_action = np.argmax(self.greedy_action(new_state))
        # What is reward for the best next action?
        future_reward = self.q_table[future_action][new_state]

        # Main Q-table updating algorithm
        new_value = old_value + self.learning_rate * (reward + self.discount * future_reward - old_value)
        self.q_table[action][old_state] = new_value

        # Finally shift our exploration_rate toward zero (less gambling)
        if self.exploration_rate > 0:
            self.exploration_rate -= self.exploration_delta