function Hd = filter
%FILTER Returns a discrete-time filter object.
% All frequency values are in Hz.
Fs = 8000;                 % Sampling Frequency
Fstop1 = 375;              % First Stopband Frequency
Fpass1 = 450;              % First Passband Frequency
Fpass2 = 1600;             % Second Passband Frequency
Fstop2 = 1750;             % Second Stopband Frequency
A = [0 1 0];               % Desire amplitude
dens = 20;               % Density Factor

rp = 0.4;                  % Stopband ripple
sa = -48;                  % Minimum stopband attenuation
dev1 = power(10,sa/20);
dev2 = (power(10,(rp/20))-1)/(power(10,(rp/20))+1); 
dev3 = power(10,sa/20);
DEV = [dev1, dev2, dev3];   % Deviation

% Calculate the order from the parameters using FIRPMORD.
[N, Fo, Ao, W] = firpmord([Fstop1 Fpass1 Fpass2 Fstop2], A,...
    DEV, Fs);

% Calculate the coefficients using the FIRPM function.
b  = firpm(N+6, Fo, Ao, W, {dens});
Hd = dfilt.dffir(b);
freqz(Hd);

nf = 2/Fs;                  % Normalize freq factor
line('XData', [Fstop1 Fstop1]*nf, 'YData', [-100 50], 'LineStyle', '-.',...
    'LineWidth',0.1,'Color',[1 0 0]);
line('XData', [Fstop2 Fstop2]*nf, 'YData', [-100 50], 'LineStyle', '-.',...
    'LineWidth',0.1,'Color',[1 0 0]);
line('XData', [Fpass1 Fpass1]*nf, 'YData', [-100 50], 'LineStyle', '-.',...
    'LineWidth',0.1,'Color',[1 0 0]);
line('XData', [Fpass2 Fpass2]*nf, 'YData', [-100 50], 'LineStyle', '-.',...
    'LineWidth',0.1,'Color',[1 0 0]);

line('XData', [0 1], 'YData', [0.4 0.4], 'LineStyle', '-.',...
    'LineWidth',0.1,'Color',[1 0 1]);
line('XData', [0 1], 'YData', [-0.4 -0.4], 'LineStyle', '-.',...
    'LineWidth',0.1,'Color',[1 0 1]);
line('XData', [0 1], 'YData', [-48 -48], 'LineStyle', '-.',...
    'LineWidth',0.1,'Color',[1 0 1]);

save filename.txt b -ASCII -DOUBLE -TABS

% [EOF]
