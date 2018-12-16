

def decodeRecord(example_proto):
  keys_to_features = {'X':tf.FixedLenFeature((shape_of_npy_array), tf.float32),
                      'y': tf.FixedLenFeature((), tf.int64, default_value=0)}
  parsed_features = tf.parse_single_example(example_proto, keys_to_features)
 return parsed_features['X'], parsed_features['y']

def loadTFRecord(filenames):
    # https://stackoverflow.com/questions/45427637/numpy-to-tfrecords-is-there-a-more-simple-way-to-handle-batch-inputs-from-tfrec
    # Creates a dataset that reads all of the examples from filenames.
    dataset = tf.data.TFRecordDataset(filenames)

    # Parse the record into tensors.
    dataset = dataset.map(decodeRecord)

    # Shuffle the dataset
    dataset = dataset.shuffle(buffer_size=10000)

    # Repeat the input indefinitly
    dataset = dataset.repeat()

    # Generate batches
    dataset = dataset.batch(batch_size)

    # Create a one-shot iterator
    iterator = dataset.make_one_shot_iterator()

    # Get batch X and y
    X, y = iterator.get_next()
