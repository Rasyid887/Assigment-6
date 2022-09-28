## Search Grid
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

    Kp = [100,114,167];
    Ti = [0.10,0.2];
    Td = [0:0.3,0.04];

Paramter tuning untuk metode search grid dideklarasikan

    figure(1);
    hold all;
    index = 1;
    t = [0:0.01:3.5];
    for i=1:length(Kp)
        for j=1:length(Ti)
            for k=1:length(Td)
                C = Kp(i) + Kp(i)/(s*Ti(j)) + Kp(i)*Td(k)*s;
                syss = feedback(sys*C,1); 
                Cnum{index} = syss.num;
                Cden{index} = syss.den;
                kombinasi{index} = [Kp(i),Ti(j),Td(k)];
                [ys,ts] = step(syss);
                info{index} = stepinfo(syss);
                riseTime{index} = info{1,index}.RiseTime;
                settlingTime{index} = info{1,index}.SettlingTime;
                overshoot{index} = info{1,index}.Overshoot;

                step(syss);
                index = index + 1;
            end
        end
    end
    grid on;
    title('Step Respon Semua Kemungkinan');
    hold off;

Semua kombinasi dari paramter tuning yang sudah dideklarasikan di uji step response

    riseTime = cell2mat(riseTime);
    settlingTime = cell2mat(settlingTime);
    overshoot = cell2mat(overshoot);

    [sortRT,idxRT] = sort(riseTime);
    [sortST,idxST] = sort(settlingTime);
    [sortO,idxO] = sort(overshoot);

    pool_arr = [idxRT(1:3),idxST(1:3),idxO(1:3)];
    pool = mode(pool_arr);

Berdasarkan hasil uji masing-masing kombinasi parameter tuning, dicari nilai dengan parameter (rise time, settling time, dan overshoot) terbaik

    figure(2);
    sysss = tf(cell2mat(Cnum{1,pool}),cell2mat(Cden{1,pool}));
    step(sysss, 2.5);
    [ys,ts] = step(sysss);
    title('Step Response Kombinasi Terbaik');

    PID = cell2mat(kombinasi(pool));
    Kpfinal = PID(1)
    Tifinal = PID(2)
    Tdfinal = PID(3)

Parameter tuning terbaik kemudian diuji dengan step response untuk melihat grafiknya

# Hasil Uji Sistem

Uji sistem step response untuk masing-masing kombinsai parameter

![sg2](https://user-images.githubusercontent.com/68903409/192834747-3137c3e6-dd23-44df-8a07-b7813b3a6100.png)

Uji sistem step response untuk parameter terbaik yang didapatkan

![sg](https://user-images.githubusercontent.com/68903409/192834711-2d929b50-9459-4a20-9d6f-f03ee40103e2.png)
