import tensorflow as tf


def dropout(x, std=0.05):
    return tf.multiply(x, tf.truncated_normal(shape=tf.shape(x), mean=1.0, stddev=std, dtype=tf.float32))


def selu(x, _lambda=1.0507, _alpha=1.6733):
    return _lambda * tf.where(x >= 0.0, x, _alpha * tf.nn.elu(x))


class CNN:
    def __init__(self, input_data, layers_info, dropout_std, trainable, suffix):
        layer_input = input_data
        self.layer_conv = []

        for i in range(len(layers_info)):
            with tf.name_scope("Conv-" + str(i)):
                layer = CNN.new_conv_layer(layer_input, layers_info[i], dropout_std, trainable, suffix + str(i))
            self.layer_conv.append(layer)
            layer_input = layer

        self.output, self.output_length = self.flatten_layer(layer_input)

    @staticmethod
    # layer_info dictionary: {num_filter, size_filter, stride_filter, padding, activation, use_pooling}
    def new_conv_layer(input_data, layer_info, dropout_std, trainable, suffix):
        if layer_info['padding'] == 'FULL':
            cols_to_add = int(layer_info['size_filter'][1] / 2)
            input_data = tf.pad(input_data, [[0, 0], [0, 0], [cols_to_add, cols_to_add], [0, 0]])

        layer = tf.layers.conv2d(
            inputs=input_data,
            filters=layer_info['num_filter'],
            kernel_size=layer_info['size_filter'],
            strides=layer_info['stride_filter'],
            padding='SAME',
            data_format='channels_last',
            activation=None,
            use_bias=True,
            kernel_initializer=tf.contrib.layers.xavier_initializer_conv2d(),
            bias_initializer=tf.contrib.layers.xavier_initializer_conv2d(),
            kernel_regularizer=tf.nn.l2_loss,
            bias_regularizer=None,
            trainable=trainable,
            name='Conv2D-' + suffix
        )

        layer = dropout(layer, dropout_std)

        if layer_info['activation'] == 'relu':
            layer = tf.nn.relu(layer)
        elif layer_info['activation'] == 'tanh':
            layer = tf.nn.tanh(layer)
        elif layer_info['activation'] == 'selu':
            layer = selu(layer)

        if layer_info['use_pooling']:
            layer = tf.nn.max_pool(value=layer, ksize=[1, 1, 2, 1], strides=[1, 1, 2, 1], padding='Valid')

        tf.summary.histogram("Activations", layer)

        return layer

    @staticmethod
    def flatten_layer(layer):
        layer_shape = layer.get_shape()
        num_features = layer_shape[1:4].num_elements()
        layer_flat = tf.reshape(layer, [-1, num_features])
        return layer_flat, num_features


class MLP:
    def __init__(self, input_data, layers_info, dropout_std, trainable, suffix):
        layer = dropout(input_data, dropout_std)
        # layer = input_data
        for i in range(len(layers_info)):
            scope = suffix + '-FC' + str(i)
            with tf.name_scope(scope):
                layer = MLP.new_fc_layer(layer, layers_info[i], dropout_std if (i + 1 < len(layers_info)) else 0
                                         , trainable, scope)
        self.output = layer

    @staticmethod
    # layer_info format: [num_outputs, activation]
    def new_fc_layer(input_data, layer_info, dropout_std, trainable, suffix):
        layer = tf.contrib.layers.fully_connected(
            inputs=input_data
            , num_outputs=layer_info['num_outputs']
            , activation_fn=None
            , normalizer_fn=None
            , normalizer_params=None
            # , weights_initializer=tf.contrib.layers.xavier_initializer()
            , weights_initializer=tf.truncated_normal_initializer(stddev=0.05)
            , biases_initializer=tf.zeros_initializer()
            , weights_regularizer=tf.nn.l2_loss
            , biases_regularizer=None
            , reuse=None
            , variables_collections=None
            , outputs_collections=None
            , trainable=trainable
            , scope=suffix
        )

        if layer_info['activation'] == 'relu':
            layer = tf.nn.relu(layer)
        elif layer_info['activation'] == 'tanh':
            layer = tf.nn.tanh(layer)
        elif layer_info['activation'] == 'selu':
            layer = selu(layer)
        elif layer_info['activation'] == 'softmax':
            layer = tf.nn.softmax(layer)
        elif layer_info['activation'] == 'sigmoid':
            layer = tf.nn.sigmoid(layer)
        elif layer_info['activation'] == 'elu':
            layer = tf.nn.elu(layer)
        elif layer_info['activation'] == 'relu6':
            layer = tf.nn.relu6(layer)
        elif layer_info['activation'] == 'crelu':
            layer = tf.nn.crelu(layer)
        elif layer_info['activation'] == 'lrelu':
            layer = tf.nn.leaky_relu(layer)
        elif layer_info['activation'] == 'softplus':
            layer = tf.nn.softplus(layer)
        elif layer_info['activation'] == 'softsign':
            layer = tf.nn.softsign(layer)

        layer = dropout(layer, dropout_std)
        # tf.summary.histogram(suffix, layer)
        return layer
