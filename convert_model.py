import tensorflow as tf

# Load the Keras model
abp_model = tf.keras.models.load_model('ABP_model_tf')
wave_model = tf.keras.models.load_model('wave_model_tf')

# Convert the model to TensorFlow Lite format
converter = tf.lite.TFLiteConverter.from_keras_model(abp_model)
abp_tflite_model = converter.convert()

converter = tf.lite.TFLiteConverter.from_keras_model(wave_model)
wave_tflite_model = converter.convert()

# Save the TensorFlow Lite model
with open('ABP_model.tflite', 'wb') as f:
    f.write(abp_tflite_model)

with open('wave_model.tflite', 'wb') as f:
    f.write(wave_tflite_model)