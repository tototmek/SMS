#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define D 30
#define N 20
#define Nu 10
#define lambda 10

#define sim_time 180

// Tak Nooby, napisałem DMCka w C
// Można korzystać do woli
// tylko pozmieniać kod!!!
// 🤓🤓🤓

// Funkcja do obliczania wyznacznika macierzy
float det(float matrix[][Nu], int n) 
{ 
    float d = 0;
    float submatrix[Nu][Nu];
    int sign = 1;
    if (n == 1) return matrix[0][0];
    for (int f=0; f < n; ++f) {
        int subi = 0;
        for (int i=1; i < n; ++i) {
            int subj = 0;
            for (int j=0; j < n; ++j) {
                if (j==f) continue;
                submatrix[subi][subj] = matrix[i][j];
                ++subj;
            }
            ++subi;
        }
        d += sign * matrix[0][f] * det(submatrix, n - 1);
        sign = -sign;
    }
    return d;
}

int main() {
    // Wczytywanie modelu odpowiedzi skokowej
    float s[60] = {0.0, 0.0, 0.0, 0.0252, 0.06790896, 0.122148105408, 0.1833221578454784, 0.2479477572227568, 0.3134299052923838, 0.37787861828914715, 0.43995915046593026, 0.49877016304147265, 0.553745083987218, 0.6045726509516768, 0.6511332683621048, 0.693448354534888, 0.7316403180450126, 0.7659011957575681, 0.7964683175775173, 0.8236056436849274, 0.8475896562614025, 0.8686988859786772, 0.8872063194484523, 0.9033740722965894, 0.917449827725374, 0.9296646359754195, 0.9402317490888716, 0.9493462304535176, 0.9571851320286341, 0.9639080758355963, 0.9696581118655522, 0.9745627533881085, 0.9787351138989687, 0.982275088596534, 0.9852705381461484, 0.9877984442605878, 0.9899260158676174, 0.9917117318248148, 0.9932063116720115, 0.9944536101078584, 0.9954914340069502, 0.9963522830773212, 0.9970640168744397, 0.9976504519831292, 0.9981318938714108, 0.9985256083054139, 0.9988462373685998, 0.9991061651124049, 0.999315837727112, 0.9994840428989779, 0.999618152741667, 0.9997243343793132, 0.9998077319320597, 0.9998726233253493, 0.999922555020719, 0.9999604574548631, 0.9999887436795842, 1.000009393420658, 1.0000240245200305, 1.000033953493674};
    // Obliczenia offline
    int i, j, k;
    // Obliczenie M
    float M[N][Nu] = {0};
    for (i=0; i < N; ++i)
        for (j=0; j < Nu; ++j)
            if (i-j >= 0) M[i][j] = s[i-j];
            else M[i][j] = 0;
    // Obliczenie transpozycji M
    float Mt[Nu][N] = {0};
    for (i=0; i < N; ++i)
        for (j=0; j < Nu; ++j)
            Mt[j][i] = M[i][j];
    // Obliczenie MP
    float MP[N][D-1] = {0};
    for (i=0; i < N; ++i)
        for (j=0; j < D-1; ++j)
            MP[i][j] = s[i+j+1] - s[j];
    // Wyznaczanie K
    float K[Nu][N] = {0};
    // Obliczenie Lambda
    float Lambda[Nu][Nu] = {0};
    for (i=0; i < Nu; ++i)
        for (j=0; j < Nu; ++j)
            if (i == j) Lambda[i][j] = lambda;
            else Lambda[i][j] = 0;
    float MtM[Nu][Nu] = {0};
    // Mnożenie Mt * M
	for(i=0; i < Nu; ++i)
		for(j=0; j < Nu; ++j)
			for(k=0; k < N; ++k) 
				MtM[i][j]+=Mt[i][k]*M[k][j];
    // Dodawanie MtM + Lambda
    for( i=0; i < Nu; ++i) 
		for(j=0; j < Nu; ++j) 
            MtM[i][j] += Lambda[i][j];
    // Odwracanie macierzy MtM+Lambda👊😔
    float d = det(MtM, Nu);
    float temp[Nu][Nu], inverse[Nu][Nu];
    int m, n;
    for (int q=0; q < Nu; ++q)
        for (int p=0; p < Nu; ++p) {
            m = 0;
            n = 0;
            for (i=0; i < Nu; ++i)
                for (j=0; j < Nu; ++j)
                    if (i != q && j != p) {
                        temp[m][n] = MtM[i][j];
                        if (n < (Nu - 2)) ++n;
                        else {
                            n = 0;
                            ++m;
                        }
                    }
            inverse[q][p] = pow(-1, q + p) * det(temp, Nu - 1);
        }
    for (i=0; i < Nu; ++i)
        for (j=0; j < Nu; ++j)
            temp[i][j] = inverse[j][i];
    for ( i=0; i < Nu; ++i)
        for (j=0; j < Nu; ++j)
            inverse[i][j] = temp[i][j] / d;
    // Na tym etapie błędy numeryczne są naprawdę spore
    // Ale mnie to średnio interesuje
    // Mnożenie (MtM+Lambda)^-1 * Mt
	for(i=0; i < Nu; ++i)
		for(j=0; j < N; ++j)
			for(k=0; k < Nu; ++k) 
				K[i][j]+=inverse[i][k]*Mt[k][j];
    // Jestem coraz bardziej sceptyczny xDDD
    // Dobra, sprawdziłem w matlabie, dobrze jest
    // https://www.kibrispdr.org/data/981/troll-face-meme-8.jpg
    // Inicjalizacja zmienych wykorzystywanych w obliczeniach online
    float dUP[D-1] = {0};
    float Y_zad[N] = {0};
    float Y[N] = {0};
    float Y0[N] = {0};
    float dU[Nu] = {0};
    //Pętla symulacji
    // Parametry obiektu (Jakiegoś losowego)
    float i1 = 0.09;
    float i2 = 0.28;
    float prev_dy, dy, ddy, real_dy;
    float y[sim_time] = {0};
    float u[sim_time] = {0};
    float y_zad = 1.0;
    for (int t=3; t < sim_time; ++t) {
        if (t > sim_time / 2) y_zad = 2.0;
        // Symulacja obiektu (Jakiś wymyślony)
        dy = (u[t-3] - y[t-1]) * i1;
        ddy = (dy - prev_dy) * i2;
        real_dy = prev_dy + ddy;
        y[t] = y[t-1] + real_dy;
        prev_dy = real_dy;
        // Obliczenia online DMC
        // Przesunięcie wektora dup 💯💯🔥
        for (i=D-2; i>0; --i)
            dUP[i] = dUP[i-1];
        // Nowy element wektora dup 👉🥵👈
        dUP[0] = u[t-1] - u[t-2];
        // Wyznaczenie wektora Y_zad
        for (i=0; i < N; ++i)
            Y_zad[i] = y_zad;
        // Wyznaczenie wektora Y
        for (i=0; i < N; ++i)
            Y[i] = y[t];
        // Wyznaczenie wektora Y0 - Mnożenie MP * dup
        for(i=0; i < N; ++i) Y0[i] = 0;
        for(i=0; i < N; ++i)
			for(k=0; k < D-1; ++k) 
				Y0[i]+=MP[i][k]*dUP[k];
        // Dodanie do wyniku Y
        for (i=0; i < N; ++i)
            Y0[i] += Y[i];
        // Obliczenie Y_zad - Y0
        for (i=0; i < N; ++i)
            Y_zad[i] -= Y0[i];
        // Mnożenie K razy wynik poprzedniego.
        for(i=0; i < Nu; ++i) dU[i] = 0;
        for(i=0; i < Nu; ++i)
			for(k=0; k < N; ++k) 
				dU[i]+=K[i][k]*Y_zad[k];
        // Prawo regulacji DMC
        u[t] = u[t-1] + dU[0];
    }
    // Zapisanie przebiegów testowych
    FILE *fptr;
    fptr = fopen("y.out", "w");
    if (fptr == NULL) {
        printf("Error!");
        exit(1);
    }
    for (int k=3; k < sim_time; ++k)
        fprintf(fptr, "%.4f\n", y[k]);
    fclose(fptr);
    return 0;
}