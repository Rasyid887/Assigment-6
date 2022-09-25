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

    Kp = [4:6];
    Ti = [0.4:0.2:0.6];
    Td = [0:0.1:0.2];

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
    step(sysss);
    [ys,ts] = step(sysss);
    title('Step Response Kombinasi Terbaik');

    PID = cell2mat(kombinasi(pool));
    Kpfinal = PID(1)
    Tifinal = PID(2)
    Tdfinal = PID(3)

Parameter tuning terbaik kemudian diuji dengan step response untuk melihat grafiknya

# Hasil Uji Sistem

Uji sistem step response untuk masing-masing kombinsai parameter

![er3](https://user-images.githubusercontent.com/68903409/192127413-1cdffc71-44ac-415d-922f-c9fd3e4bcf01.png)

Uji sistem step response untuk parameter terbaik yang didapatkan

![er4](https://user-images.githubusercontent.com/68903409/192127421-9136c2ad-bc47-43c8-a2f9-79da1936f4b4.png)
