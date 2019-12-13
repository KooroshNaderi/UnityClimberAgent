import socket
import os
import numpy as np
import re
import argparse
from utils import learners
import threading
import time
from enum import Enum
# for PPO
from pathlib import Path
import scipy.signal
from policy import Policy
from value_function import NNValueFunction
from utils_scaler import Logger, Scaler
import matplotlib.pyplot as plt
import struct


class TrainingType(Enum):
    PPO = 0
    Supervised = 1
    MemoryPPO = 2


class MessageType(Enum):
    NullMsg = 0
    PredictionOpt = 1
    PredictionPPO = 2
    Validation = 3
    IncreaseIndex = 4
    Terminate = 5


class MySocket:
    def __init__(self):
        self._buffer_size = 12000

        print('Python: Setting server up...')
        try:
            # Establish communication socket
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._socket.bind(("localhost", 5555))

        except socket.error:
            raise socket.error("Couldn't launch new environment because worker number {} is still in use. "
                               "You may need to manually close a previously opened environment "
                               "or use a different worker number.".format(str(0)))

        self._socket.listen(1)
        self._conn, _ = self._socket.accept()
        self._conn.settimeout(3000)

        print('Python: Server set up successful.')

    def _recv_bytes(self):
        s = ""
        try:
            s = self._conn.recv(self._buffer_size)
            message_length = struct.unpack("I", bytearray(s[:4]))[0]
            s = s[4:]
            while len(s) != message_length:
                s += self._conn.recv(self._buffer_size)
        except Exception as inst:
            print(type(inst))
            print("Error in receiving bytes")
        # except socket.timeout as e:
        #     raise Exception("The environment took too long to respond.")
        return s.decode("utf-8")

    def parse_recv_data(self):
        s = self._recv_bytes()

        msg_type = MessageType.NullMsg
        data = np.array([])

        msg_id = s[0]
        msg_body = s[1:]
        if len(msg_body) > 0 and msg_body[0] == '|':
            msg_body = msg_body[1:]
        if len(msg_body) > 0 and msg_body[-1] == '|':
            msg_body = msg_body[:-1]

        if msg_id == 'Q' or msg_id == 'E':
            try:
                data = np.array(re.split('\|', msg_body), dtype=np.float32)

                data_row_size = data[0].astype('int32')
                data_col_size = data[1].astype('int32')
                data = data[1:]

                data = data.reshape([data_row_size, data_col_size + 1])

                data = data[:, 1:]
                if msg_id == 'Q':
                    msg_type = MessageType.PredictionOpt
                else:
                    msg_type = MessageType.PredictionPPO
            except Exception as inst:
                print(type(inst))
                print("Error in socket inquiry!")
        elif msg_id == 'A':
            # increase index!
            data = np.array([])
            msg_type = MessageType.IncreaseIndex
        elif msg_id == 'V':
            try:
                if len(msg_body) > 0:
                    data = np.array(re.split('\|', msg_body), dtype=np.float32).reshape(1, -1)

                    data_row_size = data[0, 0].astype('int32')
                    data_col_size = 4  # data[0, 6].astype('int32') + 4 + 1 + 1
                    data = data[0, 1:]

                    data = data.reshape([data_row_size, data_col_size])

                    msg_type = MessageType.Validation
            except Exception as inst:
                print(type(inst))
                print("Error in socket validation!")
        elif msg_id == 'T':
            # Terminate
            data = np.array([])
            msg_type = MessageType.Terminate

        return msg_type, data

    def send_bytes(self, msg):
        self._conn.send(self._append_length(msg.encode('utf-8')))

    @staticmethod
    def _append_length(message):
        return struct.pack("I", len(message)) + message


class TrainClimber:
    def __init__(self, base_dir, state_dim, action_dim, ref_data_manager):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.value_dim = 1

        self.scaler_input = Scaler(self.state_dim)
        self.scaler_policy = Scaler(self.action_dim)
        self.scaler_value = Scaler(self.value_dim)

        self.ref_data_manager = ref_data_manager

        self.policy_net = None
        self.value_net = None
        self.memory_net = None

        self.training_type = TrainingType.PPO

        # self.logger = Logger(logname="climbingEnv", now=time.strftime("%Y.%m.%d-%H.%M.%S"))
        self.base_dir = base_dir

        self.kl_targ = 0.003
        self.mini_batch_size = 20
        self.hid1_mult = 3
        self.policy_logvar = -1.0

        self._init_nets()

        self.memory_buffer = None
        self.learning_buffer = None
        self.reward_buffer = None
        self.random_indices_supervised = None

        self.step_save_model = 0
        self.summary_rews = []
        self.pre_learning_buffer_size = 0
        self.current_batch_data = 0

        self.counter_exp = 0
        self.trajecoty_counter = 0
        self.gamma = 0.995
        self.lam = 0.98
        self.trajectories = []

        self.is_min_max_set = False
        self.read_min_max()

        self.update_scaler = False
        self.max_counter_data = 0
        if not self.is_min_max_set:
            self.max_counter_data = 20
        if self.max_counter_data > 0 and self.update_scaler:
            self.max_counter_data = 1

    def save_min_max(self):
        c_data = np.concatenate((self.scaler_input.means.reshape(1, -1), self.scaler_input.vars.reshape(1, -1)), axis=0)
        np.savetxt("Data\MinMaxPPO.txt", c_data)
        self.is_min_max_set = True

    def read_min_max(self):
        file_str = "Data\MinMaxPPO.txt"
        if Path(file_str).is_file():
            c_data = np.genfromtxt(file_str, delimiter=' ', dtype=np.float32)
            _m = c_data[0, :]
            _v = c_data[1, :]
            self.scaler_input.means = _m[0:self.state_dim]
            self.scaler_input.vars = _v[0:self.state_dim]
            self.scaler_input.first_pass = False
            self.is_min_max_set = True

    def train(self):
        is_trained = False
        _s = 0
        if self.training_type == TrainingType.Supervised:
            is_trained = self._supervised_learning(self.ref_data_manager)
        elif self.training_type == TrainingType.PPO:
            is_trained = self._ppo_learning()
            # is_trained = self._my_ppo_learning(self.ref_data_manager)
        elif self.training_type == TrainingType.MemoryPPO:
            is_trained = self._ppo_memory_learning(self.ref_data_manager)

        # drawing reward values
        if self.reward_buffer is not None:
            _s = self.reward_buffer.shape[0]
        if _s > 256 and is_trained:
            self.summary_rews.append(np.mean(self.reward_buffer[0:_s]))
            plt.plot(np.arange(len(self.summary_rews)), self.summary_rews, label='Mean reward', color='C0',
                     linewidth=0.25, alpha=0.75)

            plt.xlabel('iterations')
            plt.ylabel('reward')
            plt.grid(True)
            plt.savefig("reward_plot.png")

            plt.pause(0.001)
            self.reward_buffer = self.reward_buffer[_s:]
        if is_trained:
            self._save_models()
        else:
            _s = np.zeros([1, self.state_dim])
            v_s = self.value_net.predict(_s)
            p_s = self.policy_net.sample(_s, False)
            if np.isnan(np.sum(p_s)) or np.isnan(v_s):
                self.value_net.restore()
                self.policy_net.restore()

    def normalize_input(self, observe):
        c_observe = np.copy(observe)
        scale, offset = self.scaler_input.get()
        return (c_observe - offset) * scale

    @staticmethod
    def normalize_reward(_reward):
        return _reward  # (_reward + 1e5) / 1e4

    @staticmethod
    def normalize_action(_action):
        return _action / 2.5

    def revert_action(self, _action):
        return np.clip(2.5 * _action, -2.5, 2.5).reshape(-1, self.action_dim)

    def predict_spline(self, input_feature, flag_query=False):
        memory_action = None
        spline_std = None
        if self.training_type == TrainingType.Supervised:
            if self.policy_net.bayesian_net:
                spline_mean, spline_std = self.policy_net.test_new_data(input_feature)
                if not (flag_query or np.random.uniform(0, 1) < 0.1):
                    spline_sample = np.random.normal(spline_mean, spline_std)
                    spline_mean = spline_sample
                spline_mean = self.revert_action(spline_mean)
                spline_std = spline_std * 2.5
                spline_std[spline_std < 0] = 0

                if np.isnan(np.sum(spline_mean) + np.sum(spline_std)):
                    spline_std = spline_std
            else:
                spline_mean = self.policy_net.test_new_data(input_feature)
                spline_mean, spline_std = self.revert_action(spline_mean)

        elif self.training_type == TrainingType.PPO:
            # sample range is from N(0,1)
            spline_mean = self.policy_net.sample(input_feature, flag_query or np.random.uniform(0, 1) < 0.25)
            spline_mean = self.revert_action(spline_mean)

        else:
            if self.memory_net.bayesian_net:
                memory_action, memory_std = self.memory_net.test_new_data(input_feature)
            else:
                memory_action = self.memory_net.test_new_data(input_feature)
            n_input_feature = np.concatenate((input_feature, memory_action), axis=1)
            spline_mean = self.policy_net.sample(n_input_feature, flag_query or np.random.uniform(0, 1) < 0.1)
            spline_mean = self.revert_action(spline_mean)
        return spline_mean, spline_std, memory_action

    def _save_models(self):
        self.step_save_model += 1
        if self.step_save_model > 10:
            self.step_save_model = 0
            try:
                if self.training_type == TrainingType.Supervised:
                    self.policy_net.save('my_model_networks')
                elif self.training_type == TrainingType.PPO:
                    _s = np.zeros([1, self.state_dim])
                    v_s = self.value_net.predict(_s)
                    p_s = self.policy_net.sample(_s, False)
                    if np.isnan(np.sum(p_s)) or np.isnan(v_s):
                        self.value_net.restore()
                        self.policy_net.restore()
                    else:
                        self.value_net.save()
                        self.policy_net.save()
                else:
                    self.value_net.save()
                    self.policy_net.save()
                    self.memory_net.save('my_memory_net')
            except Exception as inst:
                if self.training_type == TrainingType.Supervised:
                    self.policy_net.restore('my_model_networks')
                elif self.training_type == TrainingType.PPO:
                    self.value_net.restore()
                    self.policy_net.restore()
                else:
                    self.value_net.restore()
                    self.policy_net.restore()
                    self.memory_net.restore('my_memory_net')
                print("Error in saving neural networks!")
                print(type(inst))

    def _init_nets(self):
        if self.training_type == TrainingType.PPO:
            self.value_net = NNValueFunction(self.state_dim, self.hid1_mult, self.base_dir)
            self.value_net.restore()
            self.policy_net = Policy(self.state_dim, self.action_dim, self.kl_targ, self.hid1_mult,
                                     self.policy_logvar, self.base_dir, [0.2, 0.2])
            self.policy_net.restore()
        elif self.training_type == TrainingType.Supervised:
            self.policy_net = learners.NeuralNetLearner(tf_sess=None, input_size=self.state_dim,
                                                        output_size=self.action_dim,
                                                        base_dir=self.base_dir,
                                                        model_name='PolicyNet',
                                                        learning_rate=1e-4,
                                                        batch_size=self.mini_batch_size,
                                                        flag_densenet=False,
                                                        flag_baysiannet=True,
                                                        flag_gradient_scaling=False,
                                                        activation_output='')
            self.policy_net.restore('my_model_networks')
            # value_net = learners.NeuralNetLearner(tf_sess=policy_net.sess, input_size=input_policy_net,
            #                                      output_size=1,
            #                                      base_dir=base_dir,
            #                                      model_name='ValueNet',
            #                                      learning_rate=1e-4,
            #                                      batch_size=mini_batch_size,
            #                                      flag_densenet=False)

            # value_net.restore('my_model_networks')
        else:  # MemoryPPO
            self.value_net = NNValueFunction(self.state_dim + self.action_dim, self.hid1_mult, self.base_dir)
            # value_net.save()
            self.value_net.restore()
            self.policy_net = Policy(self.state_dim + self.action_dim, self.action_dim, self.kl_targ, self.hid1_mult,
                                     self.policy_logvar, self.base_dir, [0.2, 0.2])
            self.policy_net.restore()

            self.memory_net = learners.NeuralNetLearner(tf_sess=None, input_size=self.state_dim,
                                                        output_size=self.action_dim,
                                                        base_dir=self.base_dir,
                                                        model_name='MemoryNet',
                                                        learning_rate=1e-4,
                                                        batch_size=self.mini_batch_size,
                                                        flag_densenet=False,
                                                        flag_baysiannet=False,
                                                        flag_gradient_scaling=False,
                                                        activation_output='')
            self.memory_net.restore('my_memory_net')

    # def _original_ppo(self):
    @staticmethod
    def discount(x, gamma):
        """ Calculate discounted forward sum of a sequence at each point """
        return scipy.signal.lfilter([1.0], [1.0, -gamma], x[::-1])[::-1]

    def _add_disc_sum_rew(self, trajectories):
        """ Adds discounted sum of rewards to all time steps of all trajectories
        Args:
            trajectories: as returned by run_policy()
        Returns:
            None (mutates trajectories dictionary to add 'disc_sum_rew')
        """
        # gamma: discount
        for trajectory in trajectories:
            #           if self.gamma < 0.999:  # don't scale for gamma ~= 1
            #               rewards = trajectory['rewards'] * (1 - self.gamma)
            #           else:
            rewards = trajectory['rewards']
            disc_sum_rew = self.discount(rewards, self.gamma)
            trajectory['disc_sum_rew'] = disc_sum_rew

    def _add_value(self, trajectories):
        """ Adds estimated value to all time steps of all trajectories
        Args:
            trajectories: as returned by run_policy()
        Returns:
            None (mutates trajectories dictionary to add 'values')
        """
        # value_net: object with predict() method, takes observations and returns predicted state value
        for trajectory in trajectories:
            observes = trajectory['observes']
            values = self.value_net.predict(observes)
            if np.isnan(np.sum(values)):
                notifyme = 1
            trajectory['values'] = values

    def _add_gae(self, trajectories):
        """ Add generalized advantage estimator.
        https://arxiv.org/pdf/1506.02438.pdf
        Args:
            trajectories: as returned by run_policy(), must include 'values'
                key from add_value().
        Returns:
            None (mutates trajectories dictionary to add 'advantages')
        """
        # gamma: reward discount
        # lam: lambda (see paper).
        #      lam=0 : use TD residuals
        #      lam=1 : A =  Sum Discounted Rewards - V_hat(s)
        for trajectory in trajectories:
            # if self.gamma < 0.999:  # don't scale for gamma ~= 1
            #     rewards = trajectory['rewards'] * (1 - self.gamma)
            # else:
            rewards = trajectory['rewards']
            values = trajectory['values']
            # temporal differences
            tds = rewards - values
            if len(values.shape) > 0 and values.shape[0] > 1:
                tds += np.append(values[1:] * self.gamma, 0)
            advantages = self.discount(tds, self.gamma * self.lam)
            if np.isnan(np.sum(advantages)):
                notifyme = 1
            trajectory['advantages'] = advantages

    @staticmethod
    def _build_train_set(trajectories):
        """
        Args:
            trajectories: trajectories after processing by add_disc_sum_rew(),
                add_value(), and add_gae()
        Returns: 4-tuple of NumPy arrays
            observes: shape = (N, obs_dim)
            actions: shape = (N, act_dim)
            advantages: shape = (N,)
            disc_sum_rew: shape = (N,)
        """
        observes = np.concatenate([t['observes'] for t in trajectories])
        actions = np.concatenate([t['actions'] for t in trajectories])
        disc_sum_rew = np.concatenate([t['disc_sum_rew'] for t in trajectories])
        advantages = np.concatenate([t['advantages'] for t in trajectories])
        # not feeding nan!
        if np.isnan(np.sum(advantages)):
            advantages[np.isnan(advantages)] = 0

        # normalize advantages
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-6)

        return observes, actions, advantages, disc_sum_rew

    def _ppo_learning(self):
        m_size = self.counter_exp
        c_trajectory_idx = self.trajecoty_counter
        buffer_size = 4000

        if m_size >= buffer_size:
            trajectories = self.trajectories[0:c_trajectory_idx]

            self.current_batch_data += 1
            # update scale-r
            if self.current_batch_data < self.max_counter_data or self.update_scaler:
                observes = np.concatenate([t['unscaled_obs'] for t in trajectories])
                self.scaler_input.update(observes)
            # remove init data for scale-r
            if self.current_batch_data < self.max_counter_data:
                self.trajectories = self.trajectories[c_trajectory_idx:]
                self.trajecoty_counter -= c_trajectory_idx
                self.counter_exp -= m_size
                return False

            if (not self.is_min_max_set and self.current_batch_data >= self.max_counter_data) or self.update_scaler:
                self.save_min_max()

            self._add_value(trajectories)  # add estimated values to episodes
            self._add_disc_sum_rew(trajectories)  # calculated discounted sum of Rs
            self._add_gae(trajectories)  # calculate advantage
            # concatenate all episodes into single NumPy arrays
            observes, actions, advantages, disc_sum_rew = self._build_train_set(trajectories)

            # add various stats to training log:
            # log_batch_stats(observes, actions, advantages, disc_sum_rew, logger, episode)
            policy_loss, entropy, kl, beta, lr_multiplier \
                = self.policy_net.update(observes, actions, advantages)  # update policy
            val_loss, exp_var, old_exp_var = self.value_net.fit(observes, disc_sum_rew)  # update value function

            print('PL:', policy_loss, ',',
                  'PEn:', entropy, ',',
                  'KL:', kl, ',',
                  'B:', beta, ',',
                  'lr_mul:', lr_multiplier, ',',
                  'VL:', val_loss, ',',
                  'ExpVarNew:', exp_var, ',',
                  'ExpVarOld:', old_exp_var)

            # self.logger.write(display=True)  # write logger results to file and stdout

            self.trajectories = self.trajectories[c_trajectory_idx:]
            self.trajecoty_counter -= c_trajectory_idx
            self.counter_exp -= m_size
            return True
        return False

    def _supervised_learning(self, ref_data_manager):
        buffer_size = 64  # 256
        max_buffer_size = 10000
        m_size = 0
        if self.policy_net.current_training_epochs <= 0:
            return False
        if self.learning_buffer is not None:
            m_size = self.learning_buffer.shape[0]

        if m_size >= buffer_size:
            c_state = self.learning_buffer[:, 0:self.state_dim]
            c_action = self.learning_buffer[:, self.state_dim:self.state_dim + self.action_dim]
            # c_memory_action = self.learning_buffer[:, self.state_dim + self.action_dim:
            #                                       self.state_dim + 2 * self.action_dim]
            n_state = self.learning_buffer[:, self.state_dim + 2 * self.action_dim:
                                              2 * self.state_dim + 2 * self.action_dim]
            # reward = self.learning_buffer[:, 2 * self.state_dim + 2 * self.action_dim]
            # gamma_data_val = self.learning_buffer[:, -4]
            # control_cost = self.learning_buffer[:, -3]
            is_reached = self.learning_buffer[:, -2]
            # data_index = self.learning_buffer[:, -1]

            c_state[is_reached == 0, :] = n_state[is_reached == 0, :]
            ref_data_manager.update_input_feature_minmax(c_state)

            _state = ref_data_manager.get_normalized_input_feature(c_state)
            _action = ref_data_manager.get_normalized_policy_output(c_action)

            if self.learning_buffer.shape[0] < max_buffer_size / 2:
                c_state = ref_data_manager.get_cur_feature_state()
                c_state = ref_data_manager.get_normalized_input_feature(c_state)
                c_action = ref_data_manager.get_policy()
                c_action = ref_data_manager.get_normalized_policy_output(c_action)
                _state = np.concatenate((_state, c_state), axis=0)
                _action = np.concatenate((_action, c_action), axis=0)

            self.policy_net.train(train_data_input=_state, train_data_output=_action,
                                  advantage=None, train_on_batch=True)
            self.policy_net.switch_networks()
            if self.learning_buffer.shape[0] > max_buffer_size:
                remove_count = self.learning_buffer.shape[0] - max_buffer_size
                self.learning_buffer = self.learning_buffer[remove_count:, :]
            self.policy_net.current_training_epochs -= 1
            return True
        return False

    def _ppo_memory_learning(self, ref_data_manager):
        learning_buffer_size = 256
        memory_buffer_size = 2000 * 10

        if self.memory_net.current_training_epochs > 0:
            _state = ref_data_manager.get_cur_feature_state()
            c_state = ref_data_manager.get_normalized_input_feature(_state)

            c_action = ref_data_manager.get_policy()
            c_action = ref_data_manager.get_normalized_policy_output(c_action)

            if self.memory_buffer is not None:
                memory_state = self.memory_buffer[:, 0:self.state_dim]
                memory_action = self.memory_buffer[:, self.state_dim:]
                c_state = np.concatenate((c_state, memory_state), axis=0)
                c_action = np.concatenate((c_action, memory_action), axis=0)

            self.memory_net.train(train_data_input=c_state, train_data_output=c_action,
                                  advantage=None, train_on_batch=False)
            self.memory_net.switch_networks()
            self.memory_net.current_training_epochs -= 1

        m_size = 0
        if self.learning_buffer is not None:
            m_size = self.learning_buffer.shape[0]

        if m_size >= learning_buffer_size:
            c_state = self.learning_buffer[0:m_size, 0:self.state_dim]
            c_action = self.learning_buffer[0:m_size, self.state_dim:self.state_dim + self.action_dim]
            c_memory_action = self.learning_buffer[0:m_size, self.state_dim + self.action_dim:
                                                             self.state_dim + 2 * self.action_dim]
            # n_state = self.learning_buffer[0:m_size, self.state_dim + 2 * self.action_dim:
            #                                2 * self.state_dim + 2 * self.action_dim]
            reward = self.learning_buffer[0:m_size, 2 * self.state_dim + 2 * self.action_dim]
            # is_reached = self.learning_buffer[0:m_size, -2]
            # data_index = self.learning_buffer[0:m_size, -1]

            c_action = ref_data_manager.get_normalized_policy_output(c_action)

            _state = np.concatenate((c_state, c_memory_action), axis=1)
            _action = c_action

            v_s = reward
            self.value_net.fit(_state, v_s, self.logger)

            v_s = self.value_net.predict(_state)

            advantages = reward - v_s[0:m_size]
            advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-6)

            self.policy_net.update(_state, _action, advantages, self.logger)

            self.learning_buffer = self.learning_buffer[m_size:, :]

            # if memory_buffer is None:
            #    memory_buffer = np.concatenate((c_state, c_action), axis=1)
            # else:
            #    memory_buffer = np.concatenate((memory_buffer,
            #                                    np.concatenate((c_state, c_action), axis=1))
            #                                   , axis=0)
            if self.memory_buffer is not None and self.memory_buffer.shape[0] > memory_buffer_size:
                remove_count = self.memory_buffer.shape[0] - memory_buffer_size
                self.memory_buffer = np.delete(self.memory_buffer,
                                               np.arange(remove_count),
                                               axis=0)
            self.memory_net.current_training_epochs += 20
            return True
        return False

    def _ppo_learning_no_vs(self, ref_data_manager):
        buffer_size = 2000
        # train_num_online = 20

        m_size = 0
        m_size_supervised = 0
        if self.learning_buffer is not None:
            m_size = self.learning_buffer.shape[0]
        if self.random_indices_supervised is not None:
            m_size_supervised = min(20, self.random_indices_supervised.shape[0])

        add_opt_data = False
        if m_size >= buffer_size and m_size_supervised > 0:
            c_state = self.learning_buffer[:, 0:self.state_dim]
            c_action = self.learning_buffer[:, self.state_dim:self.state_dim + self.action_dim]
            # c_memory_action = self.learning_buffer[:, self.state_dim + self.action_dim:
            #                                        self.state_dim + 2 * self.action_dim]
            # n_state = self.learning_buffer[:, self.state_dim + 2 * self.action_dim:
            #                                2 * self.state_dim + 2 * self.action_dim]
            reward = self.learning_buffer[:, 2 * self.state_dim + 2 * self.action_dim]
            # gamma_data_val = self.learning_buffer[:, -4]
            # control_cost = self.learning_buffer[:, -3]
            # is_reached = self.learning_buffer[:, -2]
            data_index = self.learning_buffer[:, -1]

            ref_data_manager.scaler_value.update(reward.reshape(-1, 1))
            # ref_data_manager.scaler_input.update(c_state)
            ref_data_manager.scaler_policy.update(c_action)

            reward = ref_data_manager.get_normalized_value_output(reward)
            c_state = ref_data_manager.get_normalized_input_feature(c_state)

            advantages = np.zeros((reward.shape[0],))
            for i in range(m_size_supervised):
                reward_i = reward[data_index == self.random_indices_supervised[i]]
                avg_r_i = np.average(reward_i)
                advantages[data_index == self.random_indices_supervised[i]] = reward_i - avg_r_i

            if add_opt_data:
                off_state = ref_data_manager.get_policy_net_input_feature(
                    self.random_indices_supervised[0:m_size_supervised])
                off_state = ref_data_manager.get_normalized_input_feature(off_state)
                _state = np.concatenate((c_state, off_state), axis=0)
            else:
                off_state = []
                _state = c_state

            v_s = []
            if add_opt_data:
                off_reward = ref_data_manager.get_value_net_output(self.random_indices_supervised[0:m_size_supervised])
                off_reward = ref_data_manager.get_normalized_value_output(off_reward)
                off_reward = np.squeeze(off_reward, axis=1)
                off_v_s = v_s[m_size:]
                off_advantages = off_reward - off_v_s

                off_state = off_state[off_advantages > 0, :]
                _state = np.concatenate((c_state, off_state), axis=0)

                off_action = ref_data_manager.get_policy_net_output(self.random_indices_supervised[0:m_size_supervised])
                off_action = off_action[off_advantages > 0, :]
                _action = np.concatenate((c_action, off_action), axis=0)

                # off_advantages = off_advantages[off_advantages > 0]

                self.random_indices_supervised = self.random_indices_supervised[m_size_supervised:]
            else:
                # off_advantages = []
                _action = c_action

            # advantages = reward - v_s[0:m_size]
            # if add_opt_data:
            #    m_val_on = max(advantages)
            #    if off_advantages.shape[0] > 0:
            #        m_val_off = max(off_advantages)
            #        if m_val_off > abs(m_val_on) / 2.0:
            #            off_advantages = off_advantages / m_val_off
            #            off_advantages = off_advantages * (abs(m_val_on) / 2.0)
            #    advantages = np.concatenate((advantages, off_advantages), axis=0)
            advantages = advantages / advantages.std()

            self.policy_net.update(_state, _action, advantages, self.logger)

            self.learning_buffer = self.learning_buffer[m_size:, :]
            return True
        return False

    def _my_ppo_learning(self, ref_data_manager):
        buffer_size = 2000  # 256
        max_buffer_size = 5 * buffer_size  # 5000
        m_size = 0
        # m_size_supervised = 0
        if self.learning_buffer is not None:
            m_size = self.learning_buffer.shape[0] - self.pre_learning_buffer_size

        if m_size >= buffer_size and self.learning_buffer.shape[0] >= buffer_size:
            self.pre_learning_buffer_size = self.learning_buffer.shape[0]
            # saving = True
            c_state = self.learning_buffer[0:self.pre_learning_buffer_size, 0:self.state_dim]
            c_action = self.learning_buffer[0:self.pre_learning_buffer_size, self.state_dim:
                                                                             self.state_dim + self.action_dim]
            # c_memory_action = learning_buffer[0:self.pre_learning_buffer_size, self.state_dim + self.action_dim:
            #                                   self.state_dim + 2 * self.action_dim]
            # n_state = learning_buffer[0:self.pre_learning_buffer_size, self.state_dim + 2 * self.action_dim:
            #                           2 * self.state_dim + 2 * self.action_dim]
            reward = self.learning_buffer[0:self.pre_learning_buffer_size, 2 * self.state_dim + 2 * self.action_dim]
            # gamma_data_val = learning_buffer[0:self.pre_learning_buffer_size, -4]
            # control_cost = learning_buffer[0:self.pre_learning_buffer_size, -3]
            # is_reached = learning_buffer[0:self.pre_learning_buffer_size, -2]
            # data_index = learning_buffer[0:self.pre_learning_buffer_size, -1]

            self.current_batch_data += 1
            if self.current_batch_data < 10:
                ref_data_manager.scaler_value.update(reward[-buffer_size:].reshape(-1, 1))
                ref_data_manager.scaler_input.update(c_state[-buffer_size:, :])
                learning_buffer = self.learning_buffer[self.pre_learning_buffer_size:, :]
                self.pre_learning_buffer_size = learning_buffer.shape[0]
                return False

            reward = ref_data_manager.get_normalized_value_output(reward)
            print('min reward:', np.min(reward), 'max reward:', np.max(reward))
            c_state = ref_data_manager.get_normalized_input_feature(c_state)
            print('min state:', np.min(c_state), 'max state:', np.max(c_state))
            c_action = ref_data_manager.get_normalized_policy_output(c_action)

            v_s = self.value_net.predict(c_state)
            advantages = reward - v_s
            advantages = advantages / (advantages.std() + 1e-6)

            self.value_net.fit(c_state[-buffer_size:, :], reward[-buffer_size:], self.logger)

            c_state = c_state[advantages > 0, :]
            c_action = c_action[advantages > 0, :]
            advantages = advantages[advantages > 0]
            self.policy_net.update(c_state, c_action, advantages, self.logger)

            # o_buffer = self.learning_buffer[0:self.pre_learning_buffer_size, :]
            # o_buffer = o_buffer[is_reached == 1, :]
            # self.learning_buffer = np.concatenate((o_buffer, learning_buffer[self.pre_learning_buffer_size:, :]),
            #                                        axis=0)
            # self.pre_learning_buffer_size = o_buffer.shape[0]

            if self.learning_buffer.shape[0] > max_buffer_size:
                remove_count = self.learning_buffer.shape[0] - max_buffer_size
                self.learning_buffer = self.learning_buffer[remove_count:, :]
                self.pre_learning_buffer_size = self.learning_buffer.shape[0]
            return True
        return False

    @staticmethod
    def get_reward(dis, is_reached, control_cost):
        first_comp = np.exp(-4 * dis)

        res = np.ones([dis.shape[0], dis.shape[1]])
        res[dis > 0.1] = 0
        res[dis < 0] = 0
        first_comp[res == 1] = 1.0

        first_comp[dis < 0] = np.exp(4 * dis[dis < 0])
        reward = 0.6 * first_comp + is_reached * (0.4 + 0.15 * np.exp(-control_cost / 4000))
        return reward


np.set_printoptions(threshold=np.nan)

plt.ion()
np.random.seed(int(time.time()))

data_manager = None
climber_trainer = None
running = True


def loop_main():
    global data_manager
    global climber_trainer
    global running

    base_dir = os.getcwd()
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--cpp_call', help='Calling from C++ code?', default=False, type=bool)
    args = parser.parse_args()
    if args.cpp_call:
        base_dir = os.path.join(base_dir, 'Python')

    feature_dim = 240  # data_manager.get_state_size()
    policy_dim = 30  # data_manager.get_policy_size()
    value_dim = 1
    mini_batch_size = 64

    data_manager = learners.DataManager(batch_size=mini_batch_size,
                                        feature_state_size=feature_dim,
                                        policy_size=policy_dim,
                                        value_size=value_dim)

    climber_trainer = TrainClimber(base_dir=base_dir,
                                   state_dim=feature_dim,
                                   action_dim=policy_dim,
                                   ref_data_manager=data_manager)

    data_manager.load_folder_data(climber_trainer.training_type == TrainingType.PPO)

    # start socket thread
    thread_training = threading.Thread(target=loop_socket)
    thread_training.start()

    flag_page_opened = False
    while not flag_page_opened:
        try:
            plt.plot(0, 0, label='Mean reward', color='C0', linewidth=0.25, alpha=0.75)
            plt.pause(0.001)
            flag_page_opened = True
        except Exception as inst:
            print(type(inst))
            print("Error in plotting reward!")

    while running:
        data_manager.load_one_file_data(True, climber_trainer.training_type == TrainingType.PPO)
        climber_trainer.train()


def loop_socket():
    global data_manager
    global climber_trainer
    global running

    m_socket = MySocket()

    n_buffer_state_action = None
    received_batch_count = 0
    c_index_data = 0

    list_states = []
    list_actions = []
    list_rewards = []
    list_unscaled_state = []
    for i in range(0, 36):
        list_states.append(None)
        list_actions.append(None)
        list_rewards.append(None)
        list_unscaled_state.append(None)

    while running:
        # print('Python: Listening...')
        msg_type, data = m_socket.parse_recv_data()
        # print('Python: Received request: %s' % message)

        reply = 'N/A'
        if msg_type == MessageType.NullMsg:
            reply = 'N/A'
        elif msg_type == MessageType.PredictionOpt:
            # Query the network
            input_feature = data_manager.get_normalized_input_feature(data)
            spline, spline_std, memory_action = climber_trainer.predict_spline(input_feature)

            if spline_std is None:
                reply = np.array2string(spline[0, :], formatter={'float': lambda x: "%f" % x},
                                        precision=5, separator='|', max_line_width=100000)[1:-1]
            else:
                reply = np.array2string(np.concatenate((spline[0, :], spline_std[0, :])),
                                        formatter={'float': lambda x: "%f" % x},
                                        precision=5, separator='|', max_line_width=100000)[1:-1]
            reply = reply.replace('[', '')
            reply = reply.replace(']', '')
        elif msg_type == MessageType.PredictionPPO:
            data_size = data.shape[0]

            data_index = np.arange(0, data_size).reshape(-1, 1)

            not_normalized_input_feature = data
            input_feature = climber_trainer.normalize_input(data)  # data_manager.get_normalized_input_feature(data)
            flag_add = True

            spline, spline_std, memory_action = climber_trainer.predict_spline(input_feature)

            if memory_action is None:
                memory_action = spline

            if np.isnan(np.sum(spline)):
                print("Nan prediction happened in actions")
                flag_add = False

            if flag_add:
                n_buffer_state_action = np.concatenate((data_index,
                                                        not_normalized_input_feature,
                                                        spline,
                                                        memory_action.reshape(data_size, -1),
                                                        input_feature), axis=1)

                index_state_holds_policy = np.concatenate((n_buffer_state_action[:, 0].reshape(-1, 1),
                                                           data_manager.policy_size * np.ones((data_size, 1)),
                                                           spline), axis=1).reshape(1, -1)

                index_state_holds_policy = np.concatenate((np.array([data_size]).reshape(1, 1),
                                                           index_state_holds_policy), axis=1).reshape(1, -1)

                reply = np.array2string(index_state_holds_policy[0, :], formatter={'float': lambda x: "%f" % x},
                                        precision=5, separator='|', max_line_width=100000)[1:-1]
                reply = reply.replace('[', '')
                reply = reply.replace(']', '')
                received_batch_count += 1

        elif msg_type == MessageType.IncreaseIndex:
            c_index_data += 1
            reply = 'Increased'
        elif msg_type == MessageType.Validation:
            msg_siz = 3 + 1  # + 1 + climber_trainer.state_dim + 1
            try:
                if data.shape[1] == msg_siz and received_batch_count == 1:  # not saving and
                    # result of evaluation
                    num_trajectories = data.shape[0]

                    is_trajectory_done = data[:, 0].astype('Int32')

                    # is_addable = data[:, 1]

                    reward = data[:, 2].reshape(-1, 1)
                    # climber_trainer.normalize_reward(data[:, 2]).reshape(-1, 1)  # (36,1)

                    # is_reached = data[:, 3]

                    # state dim = (36,313)
                    c_state_not_normalized = n_buffer_state_action[:, 1:climber_trainer.state_dim + 1]
                    c_state = n_buffer_state_action[:, -climber_trainer.state_dim:]

                    # action dim = (36,66)
                    c_action = climber_trainer.normalize_action(n_buffer_state_action[:, climber_trainer.state_dim + 1:
                                                                                         climber_trainer.state_dim + 1
                                                                                         + climber_trainer.action_dim])
                    for t in range(0, num_trajectories):
                        flag_add_to_trajectory = True
                        # unscaled_obs
                        if list_unscaled_state[t] is None:
                            list_unscaled_state[t] = c_state_not_normalized[t, :].reshape(1, -1)
                        else:
                            pre_step_num = list_unscaled_state[t][-1, 0]
                            c_step_num = c_state_not_normalized[t, 0]
                            if c_step_num > pre_step_num:
                                list_unscaled_state[t] = np.concatenate((list_unscaled_state[t],
                                                                         c_state_not_normalized[t, :].reshape(1, -1)),
                                                                        axis=0)
                            else:
                                flag_add_to_trajectory = False
                        if flag_add_to_trajectory:
                            # state that trajectory t is in at current step
                            if list_states[t] is None:
                                list_states[t] = c_state[t, :].reshape(1, -1)
                            else:
                                list_states[t] = np.concatenate((list_states[t], c_state[t, :].reshape(1, -1)), axis=0)
                            # action that is done at current step for trajectory t
                            if list_actions[t] is None:
                                list_actions[t] = c_action[t, :].reshape(1, -1)
                            else:
                                list_actions[t] = np.concatenate((list_actions[t], c_action[t, :].reshape(1, -1)),
                                                                 axis=0)

                            # reward that the agent got for c_action at current step in trajectory t
                            if list_rewards[t] is None:
                                list_rewards[t] = reward[t, :].reshape(1, -1)
                            else:
                                list_rewards[t] = np.concatenate((list_rewards[t], reward[t, :].reshape(1, -1)), axis=0)
                        if is_trajectory_done[t] == 1 or not flag_add_to_trajectory:
                            trajectory = {'observes': list_states[t],
                                          'actions': list_actions[t],
                                          'rewards': np.squeeze(list_rewards[t], axis=1),
                                          'unscaled_obs': list_unscaled_state[t]}

                            climber_trainer.trajectories.append(trajectory)
                            climber_trainer.counter_exp += list_states[t].shape[0]
                            climber_trainer.trajecoty_counter += 1

                            report_reward = np.mean(list_rewards[t])

                            if climber_trainer.reward_buffer is None:
                                climber_trainer.reward_buffer = np.reshape(report_reward, (-1, 1))
                            else:
                                climber_trainer.reward_buffer = \
                                    np.concatenate((climber_trainer.reward_buffer.reshape(-1, 1),
                                                    np.reshape(report_reward, (-1, 1))), axis=0)

                            list_states[t] = None
                            list_actions[t] = None
                            list_rewards[t] = None
                            list_unscaled_state[t] = None

                    # control_cost = data[:, 4]

                    # s_index = 6
                    # n_feature_state = data[:, s_index:]

                    # off_data_index = n_buffer_state_action[buff_index, 0].astype('int32')

                    # reward = get_reward(dis.reshape(-1, 1),
                    # is_reached.reshape(-1, 1),
                    # control_cost.reshape(-1, 1))

                    # if climber_trainer.learning_buffer is None or climber_trainer.learning_buffer == np.array([]):
                    #    climber_trainer.learning_buffer = n_data[dis >= 0, :]
                    # else:
                    #    climber_trainer.learning_buffer = \
                    #        np.concatenate((climber_trainer.learning_buffer, n_data[dis >= 0, :]), axis=0)

                    # if climber_trainer.reward_buffer is None:
                    #    climber_trainer.reward_buffer = np.reshape(reward[dis >= 0], (-1, 1))
                    # else:
                    #    climber_trainer.reward_buffer = np.concatenate((climber_trainer.reward_buffer.reshape(-1, 1),
                    #                                                    np.reshape(reward[dis >= 0], (-1, 1))),
                    #                                                   axis=0)
                    if num_trajectories == n_buffer_state_action.shape[0]:
                        n_buffer_state_action = None
                        received_batch_count = 0

                    reply = 'Added'
                else:
                    for t in range(0, 36):
                        list_states[t] = None
                        list_actions[t] = None
                        list_rewards[t] = None
                        list_unscaled_state[t] = None

                    reply = 'N/A'
                    n_buffer_state_action = None
                    received_batch_count = 0
            except Exception as inst:
                print("Error in adding data to buffer!")
                print(type(inst))
        elif msg_type == MessageType.Terminate:
            # Terminate
            running = False
            reply = 'Terminated.'

        m_socket.send_bytes(reply)

        # print('Python: Send Respond: %s' % (reply + '\0').encode('utf-8'))


if __name__ == '__main__':
    thread_socket = threading.Thread(target=loop_main)
    thread_socket.start()
