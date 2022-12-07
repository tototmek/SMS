K_u = 17;      % Wyznaczone w trakcie eksperymentu
T_u = 0.55;    % Odczytane na podstawie eksperymentu

K = 0.6 * K_u;
T_i = 0.5 * T_u;
T_d = 0.125 * T_u;

fprintf("K=%f, T_i=%f, T_d=%f", [K, T_i, T_d]);


% Nastawy Ziegler Nichols: 10.2, 0.275, 0.06875
% Nastawy met. inżynierska: 5.035, 0.587, 0.1
% Nastawy met. inżynierska: 8, 0.25, 0.1