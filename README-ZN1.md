## Ziegle Nichols 1
# Penjelasan Program

    clc;
    clear;

    J = 0.01;
    b1 = 0.1;
    K = 0.01;
    R = 1;
    L1 = 0.5;

Konstanta BLDC motor yang sudah ditentukan dideklarasikan

    s = tf('s');
    sys = K/((J*s + b1)*(L1*s + R)+K^2);

Fungsi alih sistem BLDC motor dideklarasikan sesuai dengan rumus yang telah ditentukan

    figure(1);
    step(sys,3);
    grid on;
    title('Open Loop Response');
    [y,t] = step(sys);

Fungsi alih sistem diuji dengan step response untuk melihat grafik yang dihasilkan. 

    yp = diff(y);
    ypp = diff(y,2);
    t_infl = fzero(@(T) interp1(t(2:end-1),ypp,T,'linear','extrap'),0);
    y_infl = interp1(t,y,t_infl,'linear');
    hold on;
    plot(t_infl,y_infl,'ro');

Berdasarkan grafik yang dihasilkan, dapat dicari nilai dari inflection point

    h = mean(diff(t));
    dy = gradient(y, h);
    [~,idx] = max(dy);
    b2 = [t([idx-1,idx+1]) ones(2,1)] \ y([idx-1,idx+1]);  
    tv = [-b2(2)/b2(1); (1-b2(2))/b2(1)];                  
    f = [tv ones(2,1)] * b2;                               

    plot(tv, f, '-r','LineWidth',0.8)
    ylim([0 max(y)]);
    hold off;

Nilai persamaan garis (tangen line) dihitung kemudian ditambahkan kedalam grafik uji step response fungsi alih sys

    L2 = tv(1);
    T = tv(2)-tv(1);

    Ti = 2*L2
    Td = L2/2
    Kp = 1.2*T/L2;
    Ki = Kp/Ti;
    Kd = Kp*Td;

    C = pid(Kp,Ki,Kd)

Mendapatkan nilai L dan T untuk tuning PID dengan metode Ziegle Nichols 1. Kemudian dengan tabel tuning PID Ziegle Nichols 1 dapat diketaui nilai Kp, Ki, dan Kd

    syss = feedback(sys*C,1);

    figure(2);
    step(syss,3);
    title('Ziegler Nichols 1');
    grid on;

Hasil tuning PID diuji dengan step response untuk mengetahui responnya

# Hasil Uji Sistem

Uji sistem step response untuk fungsi alih BLDC motor

![er1](https://user-images.githubusercontent.com/68903409/192127146-0c36f22f-2aff-4164-89e9-da9b43d8b9cc.png)

Uji sistem step response untuk fungsi alih BLDC motor setelah dituning dengan Ziegler Nichols 1

![er2](https://user-images.githubusercontent.com/68903409/192127156-8c0d66f1-be47-490f-a482-a3665c226b53.png)
