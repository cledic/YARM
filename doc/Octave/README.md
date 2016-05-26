I use the accelerometer data, from the YARM board, to do some math with Octave.
The Yarm send the accelerometer sample to the PC, where an Octave script receive the sample using the USB serial connection.

After the reception of the serial stream, X, Y and Z, the Octave script calculate the FFT of the sample received and plot the result.
