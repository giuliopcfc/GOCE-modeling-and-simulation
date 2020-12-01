function [psdx, freq] = FFTPSD(x,Fs)
% 
% Function to ompute the unilateral PSD of a signal using fft.
% Algorithm from: //it.mathworks.com/help/signal/ug/power-spectral-density-estimates-using-fft.html
% 
% INPUT:
%  x [N,1]          Signal
%  Fs               Sampling Frequency
% 
% OUTPUT:
%   psdx [N,1]      Unilateral PSD of signal x
%   freq [1,N]      Frequency axis
% 

N = length(x);
xdft = fft(x);
xdft = xdft(1:N/2+1);
psdx = (1/(Fs*N)) * abs(xdft).^2;
psdx(2:end-1) = 2*psdx(2:end-1);
freq = 0:Fs/length(x):Fs/2;

end