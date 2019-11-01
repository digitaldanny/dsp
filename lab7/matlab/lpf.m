%+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
% DESCRIPTION:
% Low-pass filter
% Pass-band: 0 - 1400 Hz, 3 dB ripple
% Stop-band: 3k - 24k Hz, -40 dB attenuation
% Sampling Rate: 48 KHz
%+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
load lpfilter.mat;          % load the low pass filter in
lpcoef = lpfilter.Numerator;  % store the coefficients into an array

filename = '../labs/lab7/samples/HighFreqNoise_Crast_Test1.mp3';
[audio,Fs] = audioread(filename);

output = filter(lpfilter, audio);   % filter the input audio
sound(output,Fs);                   % test the filtered output