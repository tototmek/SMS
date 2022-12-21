% Parametry regulatora
D = 40;
N = 15;
Nu = 5;
lambda = 0.1;

% Wczytanie odpowiedzi skokowej
s_data = load("step_response_scaled.mat");
s = s_data.y;

% Obliczenie macierzy M
M = zeros(N, Nu);
for i = 1:N
    M(i,1)=s(i);
end
for i=2:Nu
    M(i:N,i)=M(1:N-i+1,1);
end

% Obliczenie macierzy MP
MP=zeros(N,D-1);
for i=1:N
   for j=1:D-1
      MP(i,j)=s(i+j)-s(j);
   end
end

% Obliczenie macierzy K
K = (M'*M + lambda*eye(Nu, Nu))\M';

% Wyświetlenie wyników w postaci kodu C
fprintf("float K[Nu][N] = ");
matrix_to_ccode(K)
fprintf("\nfloat MP[N][D-1] = ");
matrix_to_ccode(MP)
fprintf("\n")