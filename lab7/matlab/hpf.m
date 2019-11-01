%+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
% DESCRIPTION:
% High-pass filter
% Pass-band: 4000 - 24000 Hz, 3 dB ripple
% Stop-band: 0 - 2000 Hz, -40 dB attenuation
% Sampling Rate: 48 KHz
%+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
load hpfilter.mat;          % load the low pass filter in
hpcoef = hpfilter.Numerator;  % store the coefficients into an array

filename = '../labs/lab7/samples/LowFreqNoise_Loops1.mp3';
[audio2,Fs] = audioread(filename);

output = filter(hpfilter, audio2);   % filter the input audio
sound(output,Fs);                   % test the filtered output