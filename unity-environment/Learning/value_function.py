"""
State-Value Function

Written by Patrick Coady (pat-coady.github.io)
"""
import os
import tensorflow as tf
import numpy as np
from sklearn.utils import shuffle


class NNValueFunction(object):
    """ NN-based state-value function """
    def __init__(self, obs_dim, hid1_mult, base_dir):
        """
        Args:
            obs_dim: number of dimensions in observation vector (int)
            hid1_mult: size of first hidden layer, multiplier of obs_dim
        """
        self.save_address_check_points = os.path.join(base_dir, 'check_points_value')
        self.global_step = 0

        self.replay_buffer_x = None
        self.replay_buffer_y = None
        self.obs_dim = obs_dim
        self.hid1_mult = hid1_mult
        self.epochs = 20
        self.lr = None  # learning rate set in _build_graph()
        # self._build_graph()
        # self.sess = tf.Session(graph=self.g)
        # self.sess.run(self.init)

        self.summary_writer = None
        summaries_dir = os.path.join(base_dir, 'logs')
        scope = "value"
        with tf.variable_scope(scope):
            # Build the graph
            self._build_graph()
            self.sess = tf.Session(graph=self.g)
            self.sess.run(self.init)
            if summaries_dir:
                summary_dir = os.path.join(summaries_dir, "summaries_{}".format(scope))
                if not os.path.exists(summary_dir):
                    os.makedirs(summary_dir)
                self.summary_writer = tf.summary.FileWriter(summary_dir)

    def save(self):
        if self.save_address_check_points is None:
            print('Cannot save the network since save address is not defined.')
            return
        with self.g.as_default():
            saver = tf.train.Saver()
            address = os.path.join(self.save_address_check_points, 'value_net.ckpt')
            saver.save(self.sess, address, write_meta_graph=False)
            print('Saved network at %s' % address)

    def restore(self):
        with self.g.as_default():
            saver = tf.train.Saver()
            latest_checkpoint = tf.train.latest_checkpoint(self.save_address_check_points)
            if latest_checkpoint:
                print("Loading model checkpoint {}...\n".format(latest_checkpoint))
                saver.restore(self.sess, latest_checkpoint)

    def _build_graph(self):
        """ Construct TensorFlow graph, including loss function, init op and train op """
        self.g = tf.Graph()
        with self.g.as_default():
            self.obs_ph = tf.placeholder(tf.float32, (None, self.obs_dim), 'obs_valfunc')
            self.val_ph = tf.placeholder(tf.float32, (None,), 'val_valfunc')
            # hid1 layer size is 10x obs_dim, hid3 size is 10, and hid2 is geometric mean
            hid1_size = 64  # self.obs_dim * self.hid1_mult  # default multipler 10 chosen empirically on 'Hopper-v1'
            hid3_size = 64  # 5  # 5 chosen empirically on 'Hopper-v1'
            hid2_size = 64  # int(np.sqrt(hid1_size * hid3_size))
            # heuristic to set learning rate based on NN size (tuned on 'Hopper-v1')
            self.lr = 1e-4  # 1e-2 / np.sqrt(hid2_size)  # 1e-3 empirically determined
            print('Value Params -- h1: {}, h2: {}, h3: {}, lr: {:.3g}'
                  .format(hid1_size, hid2_size, hid3_size, self.lr))
            # 3 hidden layers with tanh activations
            out = tf.layers.dense(self.obs_ph, hid1_size, tf.nn.relu,
                                  kernel_initializer=tf.random_normal_initializer(
                                      stddev=np.sqrt(1 / self.obs_dim)), name="h1")
            out = tf.layers.dense(out, hid2_size, tf.nn.relu,
                                  kernel_initializer=tf.random_normal_initializer(
                                      stddev=np.sqrt(1 / hid1_size)), name="h2")
            out = tf.layers.dense(out, hid3_size, tf.nn.relu,
                                  kernel_initializer=tf.random_normal_initializer(
                                      stddev=np.sqrt(1 / hid2_size)), name="h3")
            out = tf.layers.dense(out, 1,
                                  kernel_initializer=tf.random_normal_initializer(
                                      stddev=np.sqrt(1 / hid3_size)), name='output')

            self.out = tf.squeeze(out)
            self.loss = tf.reduce_mean(tf.square(self.out - self.val_ph))  # squared loss
            optimizer = tf.train.AdamOptimizer(self.lr)
            self.train_op = optimizer.minimize(self.loss)
            self.init = tf.global_variables_initializer()

            # Summaries for Tensorboard
            self.summaries = tf.summary.scalar("v_loss", self.loss)

        self.sess = tf.Session(graph=self.g)
        self.sess.run(self.init)

    def fit(self, x, y):
        """ Fit model to current data batch + previous data batch

        Args:
            x: features
            y: target
            logger: logger to save training loss and % explained variance
        """
        num_batches = max(x.shape[0] // 256, 1)
        batch_size = x.shape[0] // num_batches
        y_hat = self.predict(x)  # check explained variance prior to update
        old_exp_var = 1 - np.var(y - y_hat) / (np.var(y) + 1e-6)
        if self.replay_buffer_x is None:
            x_train, y_train = x, y
        else:
            x_train = np.concatenate([x, self.replay_buffer_x])
            y_train = np.concatenate([y, self.replay_buffer_y])
        self.replay_buffer_x = x
        self.replay_buffer_y = y
        for e in range(self.epochs):
            x_train, y_train = shuffle(x_train, y_train)
            for j in range(num_batches):
                start = j * batch_size
                end = (j + 1) * batch_size
                feed_dict = {self.obs_ph: x_train[start:end, :],
                             self.val_ph: y_train[start:end]}
                summaries, _, l = self.sess.run([self.summaries, self.train_op, self.loss], feed_dict=feed_dict)
                if self.summary_writer:
                    self.summary_writer.add_summary(summaries, self.global_step)
                self.global_step += 1

        y_hat = self.predict(x)
        loss = np.mean(np.square(y_hat - y))         # explained variance after update
        exp_var = 1 - np.var(y - y_hat) / (np.var(y) + 1e-6)  # diagnose over-fitting of val func

        # logger.log({'ValFuncLoss': loss,
        #            'ExplainedVarNew': exp_var,
        #            'ExplainedVarOld': old_exp_var})
        return loss, exp_var, old_exp_var

    def predict(self, x):
        """ Predict method """
        feed_dict = {self.obs_ph: x}
        y_hat = self.sess.run(self.out, feed_dict=feed_dict)

        return np.squeeze(y_hat)

    def close_sess(self):
        """ Close TensorFlow session """
        self.sess.close()
