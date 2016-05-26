%
% pkg install -forge instrument-control
%

close; clear; clc;
pkg load instrument-control
pkg load signal

% load 'C:\\Users\\c.dicaprio\\Dropbox\\Octave file scripts\\ReadToTermination.m'

% Check if serial support exists
if (exist("serial") != 3)
    disp("No Serial Support");
endif

s1 = serial("\\\\.\\COM6");
pause(1); % Wait a second as it takes some ports a while to wake up

set(s1, 'baudrate', 115200);
set(s1, 'bytesize', 8);
set(s1, 'parity', 'n');
set(s1, 'stopbits', 1);
set(s1, 'timeout', 123); % 12.3 Seconds as an example here
% Optional commands, these can be 'on' or 'off'
%set(s1, 'requesttosend', 'off'); % Sets the RTS line
%set(s1, 'dataterminalready', 'off'); % Sets the DTR line
% Optional - Flush input and output buffers
srl_flush(s1);
srl_flush(s1);

N=170;    % numero di sample
Fs=100;   % freq. di campionamento (vedi conf. accelerometro)

while(1)
  %
  x=ReadToTermination(s1);
  y=ReadToTermination(s1);
  z=ReadToTermination(s1);

  %
  xx=strread(x);
  yy=strread(y);
  zz=strread(z);

  % visualizzazione dati ricevuti
  figure(1);
  clf;
  hold on;
  plot(xx, '-r');
  plot(yy, '-g');
  plot(zz, '-b');
  hold off;
  legend('X','Y','Z');
  title('Accelerometer data');
  drawnow();
  
  % calcolo FFT e visualizzazione
  figure(2);
  clf;
  hold on;
  xxf=fft(xx-mean(xx))/N;
  xxf=xxf(1:length(xx)/2+1);
  xxf(2:end) = 2* xxf(2:end);
  abs_value=abs(xxf);
  [xpks xidx]=max(abs_value);
  plot(abs_value, '-r');
  
  yyf=fft(yy-mean(yy))/N;
  yyf=yyf(1:length(yy)/2+1);
  yyf(2:end) = 2* yyf(2:end);
  abs_value=abs(yyf);
  [ypks yidx]=max(abs_value);
  plot(abs_value, '-g');
  
  zzf=fft(zz-mean(zz))/N;
  zzf=zzf(1:length(zz)/2+1);
  zzf(2:end) = 2* zzf(2:end);
  abs_value=abs(zzf);
  [zpks zidx]=max(abs_value);
  plot(abs_value, '-b');
  hold off;
  legend('X','Y','Z');
  %mlegend=sprintf("'X %d','Y %d','Z %d'",xidx,yidx,zidx);
  %legend=(mlegend);
  title('FFT calculation');
  drawnow();
end

fclose(s1);
