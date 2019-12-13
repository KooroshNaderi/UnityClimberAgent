#!/usr/bin/env python
import os
import time
import tensorflow as tf
import numpy as np
from utils import nn, DenseNet
from utils_scaler import Scaler  # , Logger
from pathlib import Path
import matplotlib.pyplot as plt
from enum import Enum

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
tf.set_random_seed(int(time.time()))


class NormalizationType(Enum):
    NoNormalization = 0
    MeanStd = 1
    MinMax = 2
    Scaler = 3
    Stationary = 4


class DataManager:
    @staticmethod
    def dis_btw_points(_p1, _p2):
        p1 = np.copy(_p1)
        p2 = np.copy(_p2)
        dir_ = p1 - p2

        dis = np.zeros((dir_.shape[0], 4))
        for i in range(4):
            dis[:, i] = np.sqrt(np.sum(dir_[:, i * 3:(i + 1) * 3] ** 2, axis=1))

        return np.sum(dis, axis=1)

    @staticmethod
    def get_dis_diff(_c_holds_info, _state_data):
        c_holds_info = np.copy(_c_holds_info)
        state_data = np.copy(_state_data)

        if len(c_holds_info.shape) == 1:
            c_holds_info = np.reshape(c_holds_info, (-1, c_holds_info.shape[0]))
        if len(state_data.shape) == 1:
            state_data = np.reshape(state_data, (-1, state_data.shape[0]))

        bone_feature_size = 3 + 4 + 3 + 3 + 3
        c_index = 15 * bone_feature_size + 60
        ep = np.zeros((state_data.shape[0], 12))
        for i in range(4):
            ep[:, i * 3:(i + 1) * 3] = state_data[:, c_index + 1 + 4 * i:c_index + 4 * (i + 1)]

        c_hold_ids = c_holds_info[:, 0:4]

        c_index = 16
        t_hold_ids = c_holds_info[:, c_index:c_index + 4]
        _diff = np.copy(t_hold_ids)
        _diff[c_hold_ids == t_hold_ids] = 0
        _diff[c_hold_ids != t_hold_ids] = 1
        _diff[t_hold_ids == -1] = 0

        t_pos_holds = c_holds_info[:, c_index + 4:]
        c_pos_holds = c_holds_info[:, 4:c_index]

        dis = DataManager.dis_btw_points(t_pos_holds, c_pos_holds)

        dis[np.sum(_diff, axis=1) != 1] = -1
        return dis, _diff

    def __init__(self, batch_size, feature_state_size, policy_size, value_size):
        self.is_min_max_set = False
        self.is_normalized_data_added = False

        self.normalization_type_value = NormalizationType.Scaler
        self.normalization_type_policy = NormalizationType.Stationary
        self.normalization_type_input = NormalizationType.Scaler

        self.memory_size_limit = 10000

        self.batch_size = min(batch_size, self.memory_size_limit // 6)

        self.feature_state_size = feature_state_size
        self.state_size = None
        self.hold_info_size = None
        self.policy_size = policy_size
        self.value_size = value_size
        self.data_size = None

        self.min_data = None
        self.max_data = None
        self.mean_data = None
        self.std_data = None

        self.scaler_input = Scaler(self.feature_state_size)
        self.scaler_policy = Scaler(self.policy_size)
        self.scaler_value = Scaler(self.value_size)

        self._read_min_max()

        self.train_data = np.zeros((0, 0))  # self.memory_size_limit
        self.train_data_count = 0

        self.test_data = np.zeros((0, 0))  # self.memory_size_limit // 10
        self.test_data_count = 0

        # load optimization files
        self.folder_data_str = ['']
        self.file_counter = 0
        self.current_starting_index = 0
        self.index_folder = 0

    def load_folder_data(self, if_ppo):
        while self.index_folder < len(self.folder_data_str) - 1:
            flag_continue = True
            while flag_continue:
                pre_counter = self.file_counter
                file_counter = self.load_one_file_data(True, if_ppo)
                if pre_counter == file_counter:
                    flag_continue = False
            self.index_folder += 1
            self.current_starting_index = self.train_data_count
            self.memory_size_limit += 10000
            if self.file_counter > 0:
                self.file_counter = 0
        flag_continue = True
        while flag_continue:
            pre_counter = self.file_counter
            file_counter = self.load_one_file_data(True, if_ppo)
            if pre_counter == file_counter:
                flag_continue = False

        # update statistics
        self._update_statistics_val()
        self._write_min_max()

    def load_one_file_data(self, update_minmax, if_ppo):
        try:
            file_str = "Data" + self.folder_data_str[self.index_folder] + "\Data" + str(self.file_counter) + ".txt"
            if Path(file_str).is_file():
                c_data = np.genfromtxt(file_str, delimiter=',', dtype=np.float32)
                print("Loading: " + file_str)
                if c_data.shape[0] > 0:
                    index = 0

                    exp_count = c_data[:, index]
                    index = index + 1
                    succ_count = c_data[:, index]
                    index = index + 1

                    if self.state_size is None:
                        self.state_size = c_data[0, index].astype('int32')
                    if self.hold_info_size is None:
                        self.hold_info_size = c_data[0, index + self.state_size + 1].astype('int32')
                    if self.policy_size is None:
                        self.policy_size = c_data[0, index + self.state_size + 1
                                                  + self.hold_info_size + 1].astype('int32')
                    if self.value_size is None:
                        self.value_size = 1
                    if self.feature_state_size is None:
                        self.feature_state_size = c_data[0, index + self.state_size + 1
                                                         + self.hold_info_size + 1
                                                         + self.policy_size + 1 + 1].astype('int32')

                    c_holds_info = c_data[:, index + self.state_size + 2: index + self.state_size + 2
                                          + self.hold_info_size]
                    c_state = c_data[:, index + 1:index + self.state_size + 1]
                    c_policy = c_data[:, index + self.state_size + self.hold_info_size + 3:
                                      index + self.state_size + self.hold_info_size + self.policy_size + 3]
                    c_val = c_data[:, index + self.state_size + self.hold_info_size + self.policy_size + 3]
                    c_feature_state = c_data[:, index + self.state_size + self.hold_info_size + self.policy_size + 5:
                                             index + self.state_size + self.hold_info_size + self.policy_size + 5
                                             + self.feature_state_size]
                    n_feature_state = c_data[:, index + self.state_size + self.hold_info_size + self.policy_size + 5
                                             + self.feature_state_size + 1:]

                    # dis_holds, _diff = get_dis_diff(c_holds_info, c_state)

                    # all indices
                    chosen_indices = np.arange(0, c_state.shape[0])
                    # removing not-succeeded data
                    chosen_indices[exp_count != succ_count] = -1
                    chosen_indices[c_val < 0] = -1

                    exp_count = exp_count[chosen_indices >= 0]
                    succ_count = succ_count[chosen_indices >= 0]
                    c_holds_info = c_holds_info[chosen_indices >= 0]
                    c_state = c_state[chosen_indices >= 0]
                    c_policy = c_policy[chosen_indices >= 0]

                    c_val = c_val[chosen_indices >= 0]
                    if len(exp_count) > 0:
                        c_val = c_val / exp_count

                    c_feature_state = c_feature_state[chosen_indices >= 0]
                    n_feature_state = n_feature_state[chosen_indices >= 0]

                    success_data = np.zeros((exp_count.shape[0], 1))
                    success_data[exp_count == succ_count] = 1
                    success_data = success_data.reshape(-1, 1)
                    self.file_counter += 1

                    self.add_training_data(state_data=c_state,
                                           hold_info_data=c_holds_info,
                                           policy_data=c_policy,
                                           value_data=c_val,
                                           feature_state=c_feature_state,
                                           n_feature_state=n_feature_state,
                                           success=success_data,
                                           update_minmax=update_minmax,
                                           if_ppo=if_ppo,
                                           start_index=self.current_starting_index)
        except Exception as inst:
            print(type(inst))
            print("Error in loading data!")
        return self.file_counter

    def update_input_feature_minmax(self, input_raw_feature):
        n_size = input_raw_feature.shape[0]
        old_min_state = np.copy(self.min_data[-2 * self.feature_state_size:-self.feature_state_size])
        old_max_state = np.copy(self.max_data[-2 * self.feature_state_size:-self.feature_state_size])

        min_state = np.amin(np.concatenate((old_min_state.reshape(1, -1), input_raw_feature.reshape(n_size, -1)),
                                           axis=0),
                            axis=0)
        max_state = np.amax(np.concatenate((old_max_state.reshape(1, -1), input_raw_feature.reshape(n_size, -1)),
                                           axis=0),
                            axis=0)

        self.min_data[-2 * self.feature_state_size:-self.feature_state_size] = min_state
        self.max_data[-2 * self.feature_state_size:-self.feature_state_size] = max_state

        if np.sum(np.abs(min_state - old_min_state)) > 0 or np.sum(np.abs(max_state - old_max_state)) > 0:
            self._write_min_max()

    def _read_min_max(self):
        file_str = "Data\MinMax.txt"
        if Path(file_str).is_file():
            c_data = np.genfromtxt(file_str, delimiter=' ', dtype=np.float32)
            if self.state_size is None:
                self.state_size = c_data[0, 0].astype('int32')
            if self.hold_info_size is None:
                self.hold_info_size = c_data[0, 1].astype('int32')
            if self.policy_size is None:
                self.policy_size = c_data[0, 2].astype('int32')
            if self.value_size is None:
                self.value_size = c_data[0, 3].astype('int32')
            if self.feature_state_size is None:
                self.feature_state_size = c_data[0, 4].astype('int32')
            if self.data_size is None:
                self.data_size = self.state_size + self.hold_info_size + self.policy_size + self.value_size \
                                 + 2 * self.feature_state_size + 1

            self.min_data = c_data[0, 5:]
            self.max_data = c_data[1, 5:]
            self.mean_data = c_data[2, 5:]
            self.std_data = c_data[3, 5:]

            self.is_min_max_set = True

    def _write_min_max(self):
        if self.is_min_max_set:
            file_str = "Data\MinMax.txt"
            c_data = np.zeros((4, self.data_size + 5))
            # min data
            c_data[0, 0] = self.state_size
            c_data[0, 1] = self.hold_info_size
            c_data[0, 2] = self.policy_size
            c_data[0, 3] = self.value_size
            c_data[0, 4] = self.feature_state_size
            c_data[0, 5:] = self.min_data

            # max data
            c_data[1, 0] = self.state_size
            c_data[1, 1] = self.hold_info_size
            c_data[1, 2] = self.policy_size
            c_data[1, 3] = self.value_size
            c_data[1, 4] = self.feature_state_size
            c_data[1, 5:] = self.max_data

            # mean data
            c_data[2, 0] = self.state_size
            c_data[2, 1] = self.hold_info_size
            c_data[2, 2] = self.policy_size
            c_data[2, 3] = self.value_size
            c_data[2, 4] = self.feature_state_size
            c_data[2, 5:] = self.mean_data

            # std data
            c_data[3, 0] = self.state_size
            c_data[3, 1] = self.hold_info_size
            c_data[3, 2] = self.policy_size
            c_data[3, 3] = self.value_size
            c_data[3, 4] = self.feature_state_size
            c_data[3, 5:] = self.std_data

            np.savetxt(file_str, c_data)

    def _update_statistics_val(self):
        if not self.is_min_max_set and self.train_data_count > 0:
            self.min_data = np.amin(self.train_data, axis=0)
            self.max_data = np.amax(self.train_data, axis=0)
            self.mean_data = np.mean(self.train_data, axis=0)
            self.std_data = np.std(self.train_data, axis=0)
            self.is_min_max_set = True

    def get_normalized_value_output(self, _true_val):
        true_val = np.copy(_true_val)
        if self.normalization_type_value == NormalizationType.Scaler:
            scale, offset = self.scaler_value.get()
            true_val = (true_val - offset) * scale
        elif self.normalization_type_value == NormalizationType.Stationary:
            true_val = (true_val + 5 * 1e3) / (5 * 1e3)

        if not self.is_min_max_set:
            return true_val

        init_index = self.state_size + self.hold_info_size + 1 + self.policy_size
        if self.normalization_type_value == NormalizationType.MeanStd:
            c_mean = self.mean_data[init_index].reshape(1, -1)
            c_std = self.std_data[init_index].reshape(1, -1) + 1e-10
            true_val = (true_val - c_mean) / c_std
        elif self.normalization_type_value == NormalizationType.MinMax:
            true_val = (true_val - self.min_data[init_index]) \
                       / (self.max_data[init_index] - self.min_data[init_index] + 1e-10)

        return true_val

    def get_normalized_policy_output(self, policy_output):
        if self.normalization_type_policy == NormalizationType.Stationary:
            policy_output = policy_output / 2.5
            return policy_output

        if self.normalization_type_policy == NormalizationType.Scaler and not self.scaler_policy.first_pass:
            scale, offset = self.scaler_policy.get()
            policy_output = (policy_output - offset) * scale
            return policy_output

        if not self.is_min_max_set:
            return policy_output

        init_index_policy = self.state_size + self.hold_info_size + 1
        if self.normalization_type_policy == NormalizationType.MeanStd:
            c_mean = self.mean_data[init_index_policy:init_index_policy + self.policy_size].reshape(1, -1)
            c_std = self.std_data[init_index_policy:init_index_policy + self.policy_size].reshape(1, -1) + 1e-10

            policy_output = (policy_output - c_mean) / c_std
        elif self.normalization_type_policy == NormalizationType.MinMax:
            c_min = self.min_data[init_index_policy:init_index_policy + self.policy_size].reshape(1, -1)
            c_std = (self.max_data[init_index_policy:init_index_policy + self.policy_size]
                     - c_min).reshape(1, -1) + 1e-10
            policy_output = (policy_output - c_min) / c_std

        return policy_output

    def normalize_input(self, input_data):
        if not self.is_min_max_set:
            return input_data

        # normalization of input feature
        init_index = self.state_size + self.hold_info_size + 1 + self.policy_size + 1
        if self.normalization_type_input == NormalizationType.MeanStd:
            c_mean = self.mean_data[init_index:].reshape(1, -1)
            c_std = self.std_data[init_index:].reshape(1, -1) + 1e-10

            input_data[:, init_index:] = (input_data[:, init_index:] - c_mean) / c_std
        elif self.normalization_type_input == NormalizationType.MinMax:
            input_data[:, init_index:] = (input_data[:, init_index:] - self.min_data[init_index:]) \
                                         / (self.max_data[init_index:] - self.min_data[init_index:] + 1e-10)
        elif self.normalization_type_input == NormalizationType.Scaler:
            scale, offset = self.scaler_input.get()
            input_data[:, -self.feature_state_size:] = (input_data[:, -self.feature_state_size:] - offset) * scale
            input_data[:, -2 * self.feature_state_size:-self.feature_state_size] \
                = (input_data[:, -2 * self.feature_state_size:-self.feature_state_size] - offset) * scale

        # normalization of value
        init_index = self.state_size + self.hold_info_size + 1 + self.policy_size
        input_data[:, init_index] = self.get_normalized_value_output(input_data[:, init_index])

        # normalization of policy
        init_index_policy = self.state_size + self.hold_info_size + 1
        input_data[:, init_index_policy:init_index_policy + self.policy_size] = \
            self.get_normalized_policy_output(input_data[:, init_index_policy:init_index_policy+self.policy_size])

        return input_data

    def normalize_whole_data(self):
        if not self.is_min_max_set:
            return
        if self.is_normalized_data_added:
            return
        if self.train_data_count <= 0:
            return

        self.train_data = self.normalize_input(self.train_data)
        self.test_data = self.normalize_input(self.test_data)
        self.is_normalized_data_added = True

    def add_training_data(self, state_data, hold_info_data, policy_data,
                          value_data, feature_state, n_feature_state, success,
                          update_minmax, if_ppo, start_index=0):
        if self.state_size is None:
            self.state_size = state_data.shape[1]
        if self.hold_info_size is None:
            self.hold_info_size = hold_info_data.shape[1]
        if self.policy_size is None:
            self.policy_size = policy_data.shape[1]
        if self.value_size is None:
            self.value_size = 1
        if self.feature_state_size is None:
            self.feature_state_size = feature_state.shape[1]
        if self.data_size is None:
            self.data_size = self.state_size + self.hold_info_size + self.policy_size + self.value_size \
                             + 2 * self.feature_state_size + 1

        value_data = value_data.reshape(-1, 1)

        new_data_count = state_data.shape[0]
        input_data = np.concatenate((state_data, hold_info_data, success, policy_data, value_data), axis=1)

        input_state_feature = feature_state

        # print(input_data.shape)
        # print(state_data.shape, policy_data.shape, np.expand_dims(value_data, 1).shape)
        input_data = np.concatenate((input_data, input_state_feature, n_feature_state), axis=1)

        idx_all = np.random.permutation(new_data_count)

        # train data
        new_train_data_size = new_data_count * 9 // 10
        idx_train = idx_all[:new_train_data_size]

        # update input scaler
        f_inputs = self.get_cur_feature_state()
        if self.scaler_input.first_pass and f_inputs.shape[0] > 1000:
            self.scaler_input.update(f_inputs)
        elif input_state_feature.shape[0] > 0:
            self.scaler_input.update(input_state_feature)
        if not if_ppo:
            # update policy scaler
            policy_outputs = self.get_policy()
            if self.scaler_policy.first_pass and policy_outputs.shape[0] > 1000:
                self.scaler_policy.update(policy_outputs)
            elif policy_data.shape[0] > 0:
                self.scaler_policy.update(policy_data)
            # update value scaler
            value_outputs = self.get_value()
            if self.scaler_value.first_pass and value_outputs.shape[0] > 1000:
                self.scaler_value.update(value_outputs)
            elif value_data.shape[0] > 0:
                self.scaler_value.update(value_data)

        if self.is_min_max_set and not update_minmax:
            self.normalize_whole_data()
            self.is_normalized_data_added = True
            input_data = self.normalize_input(input_data)

        if self.train_data_count + new_train_data_size > start_index + self.memory_size_limit:
            remove_count = self.train_data_count + new_train_data_size - (start_index + self.memory_size_limit)
            self.train_data = np.delete(self.train_data,
                                        start_index + np.arange(remove_count),
                                        axis=0)
            self.train_data_count -= remove_count

        if len(self.train_data) == 0:
            self.train_data = input_data[idx_train]
        else:
            self.train_data = np.append(self.train_data, input_data[idx_train], axis=0)
        self.train_data_count += new_train_data_size

        print('Data and New Data Count: %d, %d' % (self.train_data_count, new_train_data_size))

        # test data
        new_test_data_size = new_data_count * 1 // 10
        idx_test = idx_all[new_train_data_size:]
        if self.test_data_count + new_test_data_size > (start_index // 10) + (self.memory_size_limit // 10):
            remove_count = self.test_data_count + new_test_data_size \
                           - ((start_index // 10) + (self.memory_size_limit // 10))
            self.test_data = np.delete(self.test_data,
                                       (start_index // 10) + np.arange(remove_count),
                                       axis=0)
            self.test_data_count -= remove_count

        if len(self.test_data) == 0:
            self.test_data = input_data[idx_test]
        else:
            self.test_data = np.append(self.test_data, input_data[idx_test], axis=0)
        self.test_data_count += new_test_data_size

    def get_succeeded_data_indices(self):
        indices = np.arange(0, self.train_data.shape[0])
        indices = indices[self.train_data[:, self.state_size + self.hold_info_size] == 1]
        return indices.reshape(-1, 1)

    def get_cur_feature_state(self, index=None):
        if index is None:
            return self.train_data[:, -2 * self.feature_state_size:-self.feature_state_size]

        c_in_f = self.train_data[index, -2 * self.feature_state_size:-self.feature_state_size]
        if index.shape[0] > 0:
            c_in_f = c_in_f.reshape(index.shape[0], -1)
            return c_in_f

        return np.array([])

    def get_nxt_feature_state(self):
        return self.train_data[:, -self.feature_state_size:]

    def get_value(self, index=None):
        if index is None:
            return self.train_data[:, self.state_size + self.hold_info_size + 1 + self.policy_size:
                                   self.state_size + self.hold_info_size + 1 + self.policy_size+1]
        v = self.train_data[index, self.state_size + self.hold_info_size + 1 + self.policy_size:
                            self.state_size + self.hold_info_size + 1 + self.policy_size+1].reshape(1, -1)
        if index.shape[0] > 0:
            v = v.reshape(index.shape[0], -1)
        return v

    def update_policy(self, idx, data, is_reached):
        return
        #c_action = data[0, self.feature_state_size:self.feature_state_size + self.policy_size]
        #reward = data[0, 2 * self.feature_state_size + self.policy_size]

        #c_reward = self.train_data[idx, self.state_size + self.hold_info_size + 1 + self.policy_size]
        #c_reached = self.train_data[idx, self.state_size + self.hold_info_size]
        #eps = 0.01
        #if (reward > c_reward + eps and c_reached == is_reached) or (is_reached == 1 and c_reached != is_reached):
        #    self.train_data[idx, self.state_size + self.hold_info_size + 1
        #                    :self.state_size + self.hold_info_size + 1 + self.policy_size] = c_action
        #    self.train_data[idx, self.state_size + self.hold_info_size + 1 + self.policy_size] = reward
        #    self.train_data[idx, self.state_size + self.hold_info_size] = is_reached
        #return

    def get_policy(self, index=None):
        if index is None:
            return self.train_data[:, self.state_size + self.hold_info_size + 1
                                   : self.state_size + self.hold_info_size + 1 + self.policy_size]
        spline = self.train_data[index, self.state_size + self.hold_info_size + 1
                                 : self.state_size + self.hold_info_size + 1 + self.policy_size]
        if index.shape[0] > 0:
            spline = spline.reshape(index.shape[0], -1)
        return spline

    def get_state_hold_vector(self, index):
        if index >= 0 and index < self.train_data_count:
            state_hold = self.train_data[index, 0:self.state_size + self.hold_info_size]
            ret_vec = np.concatenate((np.expand_dims(self.state_size, axis=0), state_hold[0:self.state_size],
                                      np.expand_dims(self.hold_info_size, axis=0),
                                      state_hold[self.state_size:]))
            return ret_vec
        return []

    def get_normalized_input_feature(self, _input_feature):
        input_feature = np.copy(_input_feature)

        if self.normalization_type_input == NormalizationType.Scaler and not self.scaler_input.first_pass:
            scale, offset = self.scaler_input.get()
            input_feature = (input_feature - offset) * scale

        if not self.is_min_max_set:
            return input_feature

        # normalization of input feature
        if self.normalization_type_input == NormalizationType.MeanStd:
            mean_state = self.mean_data[-2 * self.feature_state_size:-self.feature_state_size].reshape(1, -1)
            std_state = self.std_data[-2 * self.feature_state_size:-self.feature_state_size].reshape(1, -1) + 1e-10
            input_feature = (input_feature - mean_state) / std_state

        elif self.normalization_type_input == NormalizationType.MinMax:
            min_state = self.min_data[-2 * self.feature_state_size:-self.feature_state_size]
            max_state = self.max_data[-2 * self.feature_state_size:-self.feature_state_size]
            input_feature = (input_feature - min_state) / (max_state - min_state + 1e-10)

        return input_feature

    def get_reverted_policy_output(self, policy_output, policy_std=None):
        std_ = 1.0
        if self.normalization_type_policy == NormalizationType.Stationary:
            policy_output = policy_output * 2.5
            std_ = 2.5
        elif self.normalization_type_policy == NormalizationType.Scaler and not self.scaler_policy.first_pass:
            scale, offset = self.scaler_policy.get()
            policy_output = (policy_output / scale) + offset
            std_ = 1 / scale

        if not self.is_min_max_set:
            policy_output = np.clip(policy_output, -2.5, 2.5).reshape(-1, self.policy_size)
            if policy_std is not None:
                policy_std = policy_std * std_
                policy_std[policy_std <= 0.0] = 1e-10
                policy_std = policy_std.reshape(-1, self.policy_size)
            return policy_output, policy_std

        if self.normalization_type_policy == NormalizationType.MeanStd:
            mean_out = self.mean_data[self.state_size + self.hold_info_size + 1
                                      :self.state_size + self.hold_info_size + 1 + self.policy_size].reshape(1, -1)
            std_out = self.std_data[self.state_size + self.hold_info_size + 1
                                    :self.state_size + self.hold_info_size + 1 + self.policy_size].reshape(1, -1)
            policy_output = policy_output * (std_out + 1e-10) + mean_out
            std_ = std_out + 1e-10
        elif self.normalization_type_policy == NormalizationType.MinMax:
            min_out = self.min_data[self.state_size + self.hold_info_size + 1
                                    :self.state_size + self.hold_info_size + 1 + self.policy_size]
            max_out = self.max_data[self.state_size + self.hold_info_size + 1
                                    :self.state_size + self.hold_info_size + 1 + self.policy_size]
            policy_output = policy_output * (max_out - min_out + 1e-10) + min_out
            std_ = (max_out - min_out) + 1e-10
        else:
            std_ = 1

        policy_output = np.clip(policy_output, -2.5, 2.5).reshape(-1, self.policy_size)
        if policy_std is not None:
            policy_std = policy_std * std_
            policy_std[policy_std <= 0.0] = 1e-10
            policy_std = policy_std.reshape(-1, self.policy_size)
        return policy_output, policy_std


class NeuralNetLearner:
    def __init__(self, tf_sess, input_size, output_size, base_dir,
                 model_name=None, learning_rate=1e-4,
                 batch_size=32,
                 checkpoint_name=None, flag_baysiannet=False, flag_densenet=False,
                 flag_gradient_scaling=False, activation_output=''):
        # adam, rmsprop, sgd, adadelta, adagrad, adagraddao, moment, ftrl, proxigrad, proxiada

        learning_algorithm = 'adam'
        self.current_training_epochs = 10
        self.algorithm_string = "lr" + '{:.0e}-'.format(1e-3) + learning_algorithm
        self.save_address_check_points = os.path.join(base_dir, 'check_points')
        if model_name is None:
            self.model_name = time.strftime("%Y.%m.%d-%H.%M.%S") + '-' + self.algorithm_string
        else:
            self.model_name = model_name + '-' + self.algorithm_string  # time.strftime("%Y.%m.%d-%H.%M.%S") +

        self.train_dropout_sd = 0.001

        self.epoch_actual = 0
        self.global_step = tf.Variable(0, trainable=False, name='global_step', dtype=tf.int64)
        self.learning_rate = learning_rate
        self.batch_size = batch_size

        self.input_size = input_size
        self.output_size = output_size

        if tf_sess is None:
            self.sess = tf.Session()
        else:
            self.sess = tf_sess

        self.bayesian_net = flag_baysiannet
        self.gradient_scaling = flag_gradient_scaling

        self.merged_summaries = []

        with tf.name_scope(self.model_name + "Input"):
            self.input = tf.placeholder(tf.float32, shape=[None, self.input_size], name=self.model_name + 'Input')

        with tf.name_scope(self.model_name + "True_Output"):
            if self.gradient_scaling:
                self.y_true = tf.placeholder(tf.float32, shape=[None, self.output_size + 1],
                                             name=self.model_name + 'True_Output')
                true_output = self.y_true[:, :self.output_size]
            else:
                self.y_true = tf.placeholder(tf.float32, shape=[None, self.output_size],
                                             name=self.model_name + 'True_Output')
                true_output = self.y_true

        with tf.name_scope(self.model_name + "Dropout"):
            self.dropout_sd = tf.placeholder(tf.float32, shape=(), name=self.model_name+'Dropout_Sd')

        self.outputs = []
        net_vars = []
        for i in range(2):
            scope = 'NeuralNetwork' + self.model_name + str(i)
            with tf.variable_scope(scope):
                activation = 'relu'
                if self.bayesian_net:
                    layers_info = [{'num_outputs': 100, 'activation': activation},
                                   {'num_outputs': 100, 'activation': activation},
                                   {'num_outputs': 100, 'activation': activation},
                                   {'num_outputs': 2 * self.output_size, 'activation': activation_output}]
                    self.outputs.append(nn.MLP(self.input, layers_info, self.dropout_sd, True,
                                               'NN' + self.model_name + str(i)).output)
                elif flag_densenet:
                    output, initOutput = DenseNet.denseNet(input=self.input, nLayers=3, nUnitsPerLayer=100,
                                                           nOutputUnits=self.output_size)
                    self.outputs.append(output)
                else:
                    layers_info = [{'num_outputs': 100, 'activation': activation},
                                   {'num_outputs': 80, 'activation': activation},
                                   {'num_outputs': 70, 'activation': activation},
                                   {'num_outputs': self.output_size, 'activation': activation_output}]
                    self.outputs.append(nn.MLP(self.input, layers_info, self.dropout_sd, True,
                                               'NN' + self.model_name + str(i)).output)
                net_vars.append(tf.get_collection(key=tf.GraphKeys.TRAINABLE_VARIABLES, scope=scope))

        self.reset_ops = []
        self.copy_from_ops = []
        for i in range(2):
            copy_ops = []
            for v in range(len(net_vars[i])):
                copy_ops.append(tf.assign(net_vars[1 - i][v], net_vars[i][v],
                                          name='COPY' + self.model_name + str(i)))
            self.copy_from_ops.append(copy_ops)
            self.reset_ops.append(tf.variables_initializer(net_vars[i], name='INIT' + self.model_name + str(i)))

        self.active_net_idx = 0

        self.costs = []
        for i in range(2):
            scope = 'Cost_' + self.model_name + str(i)
            with tf.name_scope(scope):
                if self.bayesian_net:
                    # Bayesian cost
                    # policyLoss=tf.reduce_mean(advantagesIn*tf.reduce_sum(0.5*tf.square(actionIn-policyMean)/policyVar+0.5*policyLogVar,axis=1))
                    # mean = self.outputs[i][:, :self.output_size]
                    # log_variance = self.outputs[i][:, self.output_size:]
                    # variance = tf.exp(log_variance)
                    # bayesian_cost = tf.square(tf.subtract(mean, true_output))
                    # bayesian_cost = tf.divide(bayesian_cost, 2 * variance)
                    # bayesian_cost = tf.add(bayesian_cost, log_variance)
                    # cost = bayesian_cost
                    policyMean = self.outputs[i][:, :self.output_size]
                    policyLogVar = self.outputs[i][:, self.output_size:]
                    policyVar = tf.exp(policyLogVar)
                    actionIn = true_output
                    bayesian_cost = \
                        tf.reduce_mean(0.5*tf.square(actionIn-policyMean)/(policyVar + 1e-10)+0.5*policyLogVar)
                    cost = bayesian_cost
                else:
                    # MSE cost
                    mse_cost = tf.squared_difference(true_output[:, 0:self.output_size],
                                                     self.outputs[i][:, 0:self.output_size])
                    # print(mse_cost.get_shape())
                    cost = tf.reduce_mean(mse_cost)

                if self.gradient_scaling:
                    print(cost.get_shape(), self.y_true[:, self.output_size].get_shape())
                    cost = tf.reduce_mean(self.y_true[:, self.output_size] * cost)
                    # cost = tf.reduce_mean(cost, axis=1)
                    # cost = tf.multiply(cost, self.y_true[:, self.output_size])

            self.costs.append(cost)
            # tf.summary.scalar(scope, self.costs[i])
            self.merged_summaries.append(tf.summary.scalar(scope, self.costs[i]))

        self.optimizers = []
        for i in range(2):
            if learning_algorithm == 'adam':
                self.optimizers.append(tf.train.AdamOptimizer(self.learning_rate).minimize(self.costs[i]))
            elif learning_algorithm == 'rmsprop':
                self.optimizers.append(tf.train.RMSPropOptimizer(self.learning_rate).minimize(self.costs[i]))
            elif learning_algorithm == 'sgd':
                self.optimizers.append(tf.train.GradientDescentOptimizer(self.learning_rate).minimize(self.costs[i]))
            elif learning_algorithm == 'adadelta':
                self.optimizers.append(tf.train.AdadeltaOptimizer(self.learning_rate).minimize(self.costs[i]))
            elif learning_algorithm == 'adagrad':
                self.optimizers.append(tf.train.AdagradOptimizer(self.learning_rate).minimize(self.costs[i]))
            elif learning_algorithm == 'adagraddao':
                self.optimizers.append(tf.train.AdagradDAOptimizer(self.learning_rate, self.global_step)
                                       .minimize(self.costs[i]))
            elif learning_algorithm == 'moment':
                self.optimizers.append(tf.train.MomentumOptimizer(self.learning_rate, 0.1).minimize(self.costs[i]))
            elif learning_algorithm == 'ftrl':
                self.optimizers.append(tf.train.FtrlOptimizer(self.learning_rate).minimize(self.costs[i]))
            elif learning_algorithm == 'proxigrad':
                self.optimizers.append(tf.train.ProximalGradientDescentOptimizer(self.learning_rate)
                                       .minimize(self.costs[i]))
            elif learning_algorithm == 'proxiada':
                self.optimizers.append(tf.train.ProximalAdagradOptimizer(self.learning_rate).minimize(self.costs[i]))

        self.is_writing = [False, False]

        self.writer = None
        # self.train_writer = None
        # self.test_writer = None
        self.base_address = os.path.join(base_dir, 'logs')
        self.save_address = None
        if self.base_address is not None:
            self.save_address = os.path.join(self.base_address, self.model_name + time.strftime("%Y.%m.%d-%H.%M.%S"))
            self.writer = tf.summary.FileWriter(self.save_address)
            if tf_sess is not None:
                self.writer.add_graph(self.sess.graph)
            # self.train_writer = tf.summary.FileWriter(os.path.join(self.save_address, 'train'))
            # self.train_writer.add_graph(self.sess.graph)
            # self.test_writer = tf.summary.FileWriter(os.path.join(self.save_address, 'test'))

        # self.merged_summaries = tf.summary.merge_all()
        self.merged_summaries = tf.summary.merge(self.merged_summaries)
        self.sess.run(tf.global_variables_initializer())

        if checkpoint_name is not None:
            self.restore(checkpoint_name)

    def train(self, train_data_input, train_data_output, advantage, train_on_batch=False):
        train_data_count = train_data_input.shape[0]
        if train_data_count < self.batch_size * 5 and not train_on_batch:  # self.training_epochs // 2:
            return False

        if advantage is None:
            advantage = np.ones((train_data_count, 1))

        output = train_data_output
        if output.shape.__len__() == 1:
            output = np.expand_dims(output, axis=1)

        # print(advantage.shape)

        if self.gradient_scaling:
            if advantage.shape.__len__() == 1:
                advantage = np.expand_dims(advantage, axis=1)
            # output = np.concatenate((output, advantage), axis=1)

        idx_all = np.random.permutation(train_data_count)
        for i in range(train_data_count // self.batch_size):
            idx = idx_all[i * self.batch_size: (i+1) * self.batch_size]

            output_i = output[idx, :]
            adv_i = advantage[idx, :]
            sum_adv = sum(adv_i)
            if self.gradient_scaling:
                # if sum_adv != 0:
                #    adv_i = adv_i / sum_adv
                output_i = np.concatenate((output_i, adv_i), axis=1)
            # print(output.shape)
            # print(self.train_data_output[idx, :].shape, np.expand_dims(adv_i, axis=1).shape)
            if sum_adv != 0:
                self.sess.run(self.optimizers[1 - self.active_net_idx],
                              feed_dict={self.input: train_data_input[idx, :],
                                         self.y_true: output_i,
                                         self.dropout_sd: self.train_dropout_sd})

        # self.switch_networks()

        self.epoch_actual += 1
        print('Epoch: %d' % self.epoch_actual)

        if self.writer is not None:
            idx = np.random.randint(low=train_data_count * 5 // 10,
                                    high=train_data_count, size=self.batch_size)

            output_i = output[idx, :]
            adv_i = advantage[idx, :]
            sum_adv = sum(adv_i)
            if self.gradient_scaling:
                # if sum_adv != 0:
                #    adv_i = adv_i / sum_adv
                output_i = np.concatenate((output_i, adv_i), axis=1)

            # print(train_data_input[idx, :].shape, output[idx, :].shape)
            if sum_adv != 0:
                s = self.sess.run(self.merged_summaries,
                                  feed_dict={self.input: train_data_input[idx, :],
                                             self.y_true: output_i,
                                             self.dropout_sd: 0})
                self.writer.add_summary(s, self.epoch_actual - 1)

        # self.switch_networks()

        '''
        if self.train_writer is not None and self.epoch_actual % 5 == 0:
            # Train error
            idx = self.select_mini_batch(train=True)
            s = self.sess.run(self.merged_summaries
                              , feed_dict={self.input: self.x_data[idx, :], self.y_true: self.y_data[idx, :]})
            self.train_writer.add_summary(s, self.epoch_actual)
            # Test error
            idx = self.select_mini_batch(train=False)
            s = self.sess.run(self.merged_summaries
                              , feed_dict={self.input: self.x_data[idx, :], self.y_true: self.y_data[idx, :]})
            self.test_writer.add_summary(s, self.epoch_actual)
        '''
        return True

    def test_new_data(self, test_data):
        while self.is_writing[self.active_net_idx]:
            print('Writing on active net!')
            continue

        if len(test_data.shape) == 1:
            test_data = np.reshape(test_data, (-1, self.input_size))

        output = self.sess.run(self.outputs[self.active_net_idx],
                               feed_dict={self.input: test_data, self.dropout_sd: 0})

        if self.bayesian_net:
            mean = output[:, :self.output_size]
            std = np.sqrt(np.exp(output[:, self.output_size:]))
            return mean, std

        return output

    def reset_networks(self):
        self.sess.run(self.reset_ops)

    def switch_networks(self):
        self.active_net_idx = 1 - self.active_net_idx

        self.is_writing[1 - self.active_net_idx] = True
        self.is_writing[self.active_net_idx] = False

        self.sess.run(self.copy_from_ops[self.active_net_idx])

        self.is_writing[1 - self.active_net_idx] = False
        self.is_writing[self.active_net_idx] = False

        print('Network switched at step %d' % self.epoch_actual)
        # self.sess.run(self.copy_from_ops[self.active_net_idx])
        # if self.epoch > 0 and self.data_count % 1000 == 0:
        # print('Network reset after gathering %d data' % self.data_count)

    def save(self, model_name=None):
        if self.base_address is None:
            print('Cannot save the network since save address is not defined.')
            return
        saver = tf.train.Saver()
        # Save the current checkpoint
        # # saver.save(self.sess, self.base_address)
        address = os.path.join(self.save_address_check_points,
                               (str(self.epoch_actual) if model_name is None else model_name) + '.ckpt')
        saver.save(self.sess, address, write_meta_graph=False)
        print('Saved network at %s' % address)

    def restore(self, model_name):
        saver = tf.train.Saver()
        latest_checkpoint = tf.train.latest_checkpoint(self.save_address_check_points)
        if latest_checkpoint:
            print("Loading model checkpoint {}...\n".format(latest_checkpoint))
            saver.restore(self.sess, latest_checkpoint)
        # model_address = os.path.join(self.base_address, model_name + '.ckpt')
        # if tf.train.latest_checkpoint(self.base_address):
        #    saver.restore(self.sess, model_address)
        #    print('Restored network from %s' % model_address)


def test_bayesian_learning():
    np.random.seed(int(time.time()))

    n_points_train = 10000
    n_points_test = 100

    x_data = np.arange(n_points_train) / n_points_train * np.pi * 2
    y_data = np.sin(x_data)
    cos_idx = np.random.randint(low=0, high=n_points_train, size=n_points_train//2)
    y_data[cos_idx] = np.cos(x_data[cos_idx])

    test_data = np.arange(n_points_test) / n_points_test * np.pi * 2

    learner = NeuralNetLearner(input_size=1,
                               output_size=1,
                               base_dir=None,
                               learning_rate=1e-3,
                               batch_size=32)
    #learner.add_training_data(x_data, y_data, True)

    plt.ion()

    while True:
     #   learner.train()
        if learner.bayesian_net:
            test_mean, test_std = learner.test_new_data(test_data)  # , False, False
            test_mean = test_mean[:, 0]
            test_std = test_std[:, 0]
            plt.clf()
            plt.axis([0, 2 * np.pi, -1.05, 1.05])
            plt.plot(x_data, y_data, 'bo', markersize=3, alpha=0.25)
            p = plt.plot(test_data, test_mean, 'g-', linewidth=2, alpha=0.75)
            plt.fill_between(test_data, test_mean - test_std / 2, test_mean + test_std / 2,
                             color=p[0].get_color(), alpha=0.25)
        else:
            test_mean = learner.test_new_data(test_data)  # , False, False
            plt.clf()
            plt.axis([0, 2 * np.pi, -1.05, 1.05])
            plt.plot(x_data, y_data, 'bo', markersize=3, alpha=0.25)
            plt.plot(test_data, test_mean, 'g-', linewidth=2, alpha=0.75)
        plt.pause(0.01)


if __name__ == '__main__':
    test_bayesian_learning()
